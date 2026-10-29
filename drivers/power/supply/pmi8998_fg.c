// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved. */

#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/* SOC */
#define BATT_MONOTONIC_SOC		0x009

/* BATT */
#define PARAM_ADDR_BATT_TEMP		0x150
#define BATT_INFO_JEITA_COLD		0x162
#define BATT_INFO_JEITA_COOL		0x163
#define BATT_INFO_JEITA_WARM		0x164
#define BATT_INFO_JEITA_HOT		0x165
#define PARAM_ADDR_BATT_VOLTAGE		0x1a0
#define PARAM_ADDR_BATT_CURRENT		0x1a2

/* MEMIF */
#define MEM_INTF_CFG			0x450
#define MEM_INTF_IMA_CTL		0x451
#define MEM_INTF_IMA_CFG		0x452
#define MEM_INTF_IMA_OPR_STS		0x454
#define MEM_INTF_IMA_EXP_STS		0x455
#define MEM_INTF_IMA_HW_STS		0x456
#define MEM_INTF_BEAT_COUNT		0x457
#define MEM_INTF_IMA_ERR_STS		0x45f
#define MEM_INTF_IMA_BYTE_EN		0x460
#define MEM_INTF_ADDR_LSB		0x461
#define MEM_INTF_RD_DATA0		0x467
#define MEM_INTF_WR_DATA0		0x463
#define MEM_IF_DMA_STS			0x470
#define MEM_IF_DMA_CTL			0x471

#define MEM_ACCESS_REQ			BIT(7)
#define IACS_SLCT			BIT(5)
#define IMA_CTL_MASK			GENMASK(7, 6)
#define IMA_WR_EN			BIT(6)
#define IACS_RDY			BIT(1)
#define BEAT_COUNT_MASK			GENMASK(3, 0)

/* SRAM words */
#define SRAM_NOM_CAP_WORD		58
#define SRAM_ACT_BATT_CAP_WORD		74
#define SRAM_CYCLE_COUNT_WORD		75
#define SRAM_BATT_SOC_WORD		91
#define SRAM_CC_SOC_WORD		95
#define SRAM_WORD_BYTES			4
#define SRAM_READ_TRIES			3

/*
 * Charge cycles, counted as stock does: one 16-bit counter per eighth of
 * the battery SOC range, two per SRAM word from SRAM_CYCLE_COUNT_WORD on,
 * each bumped when a charge rises more than 2/256 within its range.
 */
#define CYCLE_BUCKETS			8

/* CC_SOC is a signed fraction of the nominal capacity */
#define CC_SOC_FULL			GENMASK(29, 0)

#define BATT_TEMP_LSB_MASK		GENMASK(7, 0)
#define BATT_TEMP_MSB_MASK		GENMASK(2, 0)

struct pmi8998_fg_chip {
	struct device *dev;
	unsigned int base;
	struct regmap *regmap;
	struct notifier_block nb;

	struct power_supply *batt_psy;
	struct power_supply *chg_psy;
	int status;
	struct delayed_work status_changed_work;

	/* serialises IMA access to the FG SRAM */
	struct mutex sram_lock;
	int nom_cap_uah;
	int act_cap_uah;
	u16 cycles[CYCLE_BUCKETS];
	bool cycle_started[CYCLE_BUCKETS];
	u8 cycle_start_soc[CYCLE_BUCKETS];
	struct work_struct cycle_work;

	/* charge limit: stop at charge_end %, resume at charge_start % */
	int charge_start;
	int charge_end;
	struct work_struct charge_limit_work;
};

static int pmi8998_fg_clear_ima(struct pmi8998_fg_chip *chip, bool check_hw_sts);

/************************
 * IO FUNCTIONS
 * **********************/

/**
 * @brief pmi8998_fg_read() - Read multiple registers with regmap_bulk_read
 *
 * @param chip Pointer to chip
 * @param val Pointer to read values into
 * @param addr Address to read from
 * @param len Number of registers (bytes) to read
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_read(struct pmi8998_fg_chip *chip, u8 *val, u16 addr, int len)
{
	if (((chip->base + addr) & 0xff00) == 0)
		return -EINVAL;

	dev_vdbg(chip->dev, "%s: Reading 0x%x bytes from 0x%x", __func__, len, addr);

	return regmap_bulk_read(chip->regmap, chip->base + addr, val, len);
}

/**
 * @brief pmi8998_fg_write() - Write multiple registers with regmap_bulk_write
 *
 * @param chip Pointer to chip
 * @param val Pointer to write values from
 * @param addr Address to write to
 * @param len Number of registers (bytes) to write
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_write(struct pmi8998_fg_chip *chip, u8 *val, u16 addr, int len)
{
	bool sec_access = (addr & 0xff) > 0xd0;
	u8 sec_addr_val = 0xa5;
	int ret;

	if (((chip->base + addr) & 0xff00) == 0)
		return -EINVAL;

	dev_vdbg(chip->dev, "%s: Writing 0x%x to 0x%x", __func__, *val, addr);

	if (sec_access) {
		ret = regmap_bulk_write(chip->regmap,
					((chip->base + addr) & 0xff00) | 0xd0,
				&sec_addr_val, 1);
		if (ret)
			return ret;
	}

	return regmap_bulk_write(chip->regmap, chip->base + addr, val, len);
}

/**
 * @brief pmi8998_fg_masked_write() - like pmi8998_fg_write but applies
 * a mask first.
 *
 * @param chip Pointer to chip
 * @param val Pointer to write values from
 * @param addr Address to write to
 * @param len Number of registers (bytes) to write
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_masked_write(struct pmi8998_fg_chip *chip, u16 addr, u8 mask, u8 val)
{
	u8 reg;
	int ret;

	ret = pmi8998_fg_read(chip, &reg, addr, 1);
	if (ret)
		return ret;

	reg &= ~mask;
	reg |= val & mask;

	return pmi8998_fg_write(chip, &reg, addr, 1);
}

/*************************
 * BATTERY STATUS
 * ***********************/

/**
 * @brief pmi8998_fg_get_capacity() - Get remaining capacity of battery
 *
 * @param chip Pointer to chip
 * @param val Pointer to store value at
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_get_capacity(struct pmi8998_fg_chip *chip, int *val)
{
	u8 cap[2];
	int ret;

	ret = pmi8998_fg_read(chip, cap, BATT_MONOTONIC_SOC, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read capacity: %d", ret);
		return ret;
	}

	if (cap[0] != cap[1])
		cap[0] = cap[0] < cap[1] ? cap[0] : cap[1];

	*val = DIV_ROUND_CLOSEST((cap[0] - 1) * 98, 0xff - 2) + 1;

	return 0;
}

/*************************
 * BATTERY STATUS, GEN3
 * ***********************/

/**
 * @brief pmi8998_fg_get_temperature() - Get temperature of battery
 *
 * @param chip Pointer to chip
 * @param val Pointer to store value at
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_get_temperature(struct pmi8998_fg_chip *chip, int *val)
{
	int temp;
	u8 readval[2];
	int ret;

	ret = pmi8998_fg_read(chip, readval, PARAM_ADDR_BATT_TEMP, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read temperature: %d\n", ret);
		return ret;
	}

	temp = ((readval[1] & BATT_TEMP_MSB_MASK) << 8) |
		(readval[0] & BATT_TEMP_LSB_MASK);
	temp = DIV_ROUND_CLOSEST(temp * 10, 4);

	*val = temp - 2730;
	return 0;
}

/**
 * @brief pmi8998_fg_get_current() - Get current being drawn from battery
 *
 * @param chip Pointer to chip
 * @param val Pointer to store value at
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_get_current(struct pmi8998_fg_chip *chip, int *val)
{
	s16 temp;
	u8 readval[2];
	int ret;

	ret = pmi8998_fg_read(chip, readval, PARAM_ADDR_BATT_CURRENT, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read current: %d\n", ret);
		return ret;
	}

	//handle rev 1 too
	temp = (s16)(readval[1] << 8 | readval[0]);
	*val = div_s64((s64)temp * 488281, 1000);

	/*
	 * PSY API expects charging batteries to report a positive current, which is inverted
	 * to what the PMIC reports.
	 */
	*val = -*val;

	return 0;
}

/**
 * @brief pmi8998_fg_get_voltage() - Get voltage of battery
 *
 * @param chip Pointer to chip
 * @param val Pointer to store value at
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_get_voltage(struct pmi8998_fg_chip *chip, int *val)
{
	int temp;
	u8 readval[2];
	int ret;

	ret = pmi8998_fg_read(chip, readval, PARAM_ADDR_BATT_VOLTAGE, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read voltage: %d\n", ret);
		return ret;
	}

	//handle rev 1 too
	temp = readval[1] << 8 | readval[0];
	*val = div_u64((u64)temp * 122070, 1000);
	return 0;
}

/**
 * @brief pmi8998_fg_get_temp_threshold() - Get configured temperature thresholds
 *
 * @param chip Pointer to chip
 * @param psp Power supply property of temperature limit
 * @param val Pointer to store value at
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_get_temp_threshold(struct pmi8998_fg_chip *chip,
					 enum power_supply_property psp, int *val)
{
	u8 temp;
	u16 reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_MIN:
		reg = BATT_INFO_JEITA_COLD;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		reg = BATT_INFO_JEITA_HOT;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		reg = BATT_INFO_JEITA_COOL;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		reg = BATT_INFO_JEITA_WARM;
		break;
	default:
		return -EINVAL;
	}

	ret = pmi8998_fg_read(chip, &temp, reg, 1);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read JEITA property %d level: %d\n", psp, ret);
		return ret;
	}

	/* Resolution is 0.5C. Base is -30C. */
	*val = (((5 * temp) / 10) - 30) * 10;
	return 0;
}

/************************
 * SRAM ACCESS
 * **********************/

static int pmi8998_fg_iacs_ready(struct pmi8998_fg_chip *chip)
{
	int tries = 250;
	u8 sts;
	int ret;

	usleep_range(30, 35);
	for (;;) {
		ret = pmi8998_fg_read(chip, &sts, MEM_INTF_IMA_OPR_STS, 1);
		if (ret)
			return ret;
		if (sts & IACS_RDY)
			return 0;
		if (!--tries)
			break;
		usleep_range(5000, 7000);
	}

	ret = pmi8998_fg_clear_ima(chip, false);

	return ret ?: -EBUSY;
}

static int pmi8998_fg_set_sram_addr(struct pmi8998_fg_chip *chip, u16 word)
{
	u8 addr[2] = { word & 0xff, 0 };

	return pmi8998_fg_write(chip, addr, MEM_INTF_ADDR_LSB, 2);
}

/* One IMA read in single mode, the only mode PM660 supports reliably */
static int pmi8998_fg_sram_read_once(struct pmi8998_fg_chip *chip, u16 word,
				     u8 offset, u8 *val, int len)
{
	u8 start, end;
	int n, ret;

	ret = pmi8998_fg_masked_write(chip, MEM_INTF_IMA_CTL, IMA_CTL_MASK, 0);
	if (ret)
		return ret;

	ret = pmi8998_fg_iacs_ready(chip);
	if (ret)
		return ret;

	ret = pmi8998_fg_set_sram_addr(chip, word);
	if (ret)
		return ret;

	ret = pmi8998_fg_iacs_ready(chip);
	if (ret)
		return ret;

	ret = pmi8998_fg_read(chip, &start, MEM_INTF_BEAT_COUNT, 1);
	if (ret)
		return ret;

	while (len > 0) {
		n = min_t(int, len, SRAM_WORD_BYTES - offset);
		ret = pmi8998_fg_read(chip, val, MEM_INTF_RD_DATA0 + offset, n);
		if (ret)
			return ret;

		ret = pmi8998_fg_clear_ima(chip, false);
		if (ret)
			return ret;

		val += n;
		len -= n;
		offset = 0;

		if (len) {
			ret = pmi8998_fg_set_sram_addr(chip, ++word);
			if (ret)
				return ret;
		}

		ret = pmi8998_fg_iacs_ready(chip);
		if (ret)
			return ret;
	}

	ret = pmi8998_fg_read(chip, &end, MEM_INTF_BEAT_COUNT, 1);
	if (ret)
		return ret;

	/* The FG updated the SRAM while it was read */
	if ((start ^ end) & BEAT_COUNT_MASK)
		return -EAGAIN;

	return 0;
}

/* One IMA write in single mode, within one SRAM word */
static int pmi8998_fg_sram_write_once(struct pmi8998_fg_chip *chip, u16 word,
				      u8 offset, const u8 *val, int len)
{
	u8 byte_en = GENMASK(offset + len - 1, offset), dummy = 0;
	int ret;

	ret = pmi8998_fg_masked_write(chip, MEM_INTF_IMA_CTL, IMA_CTL_MASK,
				      IMA_WR_EN);
	if (ret)
		return ret;

	ret = pmi8998_fg_iacs_ready(chip);
	if (ret)
		return ret;

	ret = pmi8998_fg_set_sram_addr(chip, word);
	if (ret)
		return ret;

	ret = pmi8998_fg_write(chip, &byte_en, MEM_INTF_IMA_BYTE_EN, 1);
	if (ret)
		return ret;

	ret = pmi8998_fg_write(chip, (u8 *)val, MEM_INTF_WR_DATA0 + offset, len);
	if (ret)
		return ret;

	/* Writing WR_DATA3 starts the transaction; byte_en masks it out */
	if (!(byte_en & BIT(3))) {
		ret = pmi8998_fg_write(chip, &dummy, MEM_INTF_WR_DATA0 + 3, 1);
		if (ret)
			return ret;
	}

	ret = pmi8998_fg_clear_ima(chip, false);
	if (ret)
		return ret;

	return pmi8998_fg_iacs_ready(chip);
}

static int pmi8998_fg_sram_xfer(struct pmi8998_fg_chip *chip, u16 word,
				u8 offset, u8 *val, int len, bool write)
{
	int tries, polls, ret, rel;
	u8 cfg;

	if (offset >= SRAM_WORD_BYTES)
		return -EINVAL;

	mutex_lock(&chip->sram_lock);

	for (tries = 0; tries < SRAM_READ_TRIES; tries++) {
		/* MEM_ACCESS_REQ stays set until the previous access ends */
		for (polls = 0; polls < 5; polls++) {
			ret = pmi8998_fg_read(chip, &cfg, MEM_INTF_CFG, 1);
			if (ret || !(cfg & MEM_ACCESS_REQ))
				break;
			usleep_range(4000, 4100);
		}
		if (!ret && (cfg & MEM_ACCESS_REQ))
			ret = -EBUSY;

		if (!ret)
			ret = pmi8998_fg_masked_write(chip, MEM_INTF_CFG,
						      MEM_ACCESS_REQ | IACS_SLCT,
						      MEM_ACCESS_REQ | IACS_SLCT);
		if (!ret && write)
			ret = pmi8998_fg_sram_write_once(chip, word, offset,
							 val, len);
		else if (!ret)
			ret = pmi8998_fg_sram_read_once(chip, word, offset,
							val, len);

		rel = pmi8998_fg_masked_write(chip, MEM_INTF_CFG,
					      MEM_ACCESS_REQ | IACS_SLCT, 0);
		if (!ret)
			ret = rel;
		if (!ret)
			break;
	}

	mutex_unlock(&chip->sram_lock);

	if (ret)
		dev_err(chip->dev, "Failed to %s SRAM word %u: %d\n",
			write ? "write" : "read", word, ret);

	return ret;
}

static int pmi8998_fg_sram_read(struct pmi8998_fg_chip *chip, u16 word,
				u8 offset, u8 *val, int len)
{
	return pmi8998_fg_sram_xfer(chip, word, offset, val, len, false);
}

static int pmi8998_fg_sram_write(struct pmi8998_fg_chip *chip, u16 word,
				 u8 offset, u8 *val, int len)
{
	if (offset + len > SRAM_WORD_BYTES)
		return -EINVAL;

	return pmi8998_fg_sram_xfer(chip, word, offset, val, len, true);
}

static int pmi8998_fg_read_capacities(struct pmi8998_fg_chip *chip)
{
	u8 buf[2];
	int ret;

	ret = pmi8998_fg_sram_read(chip, SRAM_NOM_CAP_WORD, 0, buf, 2);
	if (ret)
		return ret;
	chip->nom_cap_uah = get_unaligned_le16(buf) * 1000;

	ret = pmi8998_fg_sram_read(chip, SRAM_ACT_BATT_CAP_WORD, 0, buf, 2);
	if (ret)
		return ret;
	chip->act_cap_uah = get_unaligned_le16(buf) * 1000;

	return 0;
}

/**
 * @brief pmi8998_fg_get_charge_counter() - Get the coulomb counter
 *
 * The FG integrates the battery current in CC_SOC, also while the SoC
 * sleeps, so differences of this value give the charge used over any
 * interval.
 *
 * @param chip Pointer to chip
 * @param val Pointer to store value at
 * @return int 0 on success, negative errno on error
 */
static int pmi8998_fg_read_cycles(struct pmi8998_fg_chip *chip)
{
	u8 buf[CYCLE_BUCKETS * 2];
	int i, ret;

	ret = pmi8998_fg_sram_read(chip, SRAM_CYCLE_COUNT_WORD, 0, buf,
				   sizeof(buf));
	if (ret)
		return ret;

	for (i = 0; i < CYCLE_BUCKETS; i++)
		chip->cycles[i] = get_unaligned_le16(&buf[i * 2]);

	return 0;
}

static int pmi8998_fg_store_cycle(struct pmi8998_fg_chip *chip, int bucket)
{
	u16 word = SRAM_CYCLE_COUNT_WORD + bucket / 2;
	u8 offset = (bucket % 2) * 2, buf[2];
	int ret;

	put_unaligned_le16(chip->cycles[bucket] + 1, buf);
	ret = pmi8998_fg_sram_write(chip, word, offset, buf, 2);
	if (ret)
		return ret;

	ret = pmi8998_fg_sram_read(chip, word, offset, buf, 2);
	if (ret)
		return ret;
	if (get_unaligned_le16(buf) != chip->cycles[bucket] + 1)
		return -EIO;

	chip->cycles[bucket]++;

	return 0;
}

/*
 * A bucket starts when charging reaches it and counts once charging stops
 * (full, unplugged or held by the charge limit) more than 2/256 above
 * where it started.
 */
static void pmi8998_fg_cycle_worker(struct work_struct *work)
{
	struct pmi8998_fg_chip *chip = container_of(work, struct pmi8998_fg_chip,
						    cycle_work);
	union power_supply_propval status, behaviour;
	int bucket, i, ret;
	bool charging;
	u8 buf[4], soc;

	if (!chip->chg_psy ||
	    power_supply_get_property(chip->chg_psy, POWER_SUPPLY_PROP_STATUS,
				      &status))
		return;

	/*
	 * The status follows a charge limit stop only seconds later, without
	 * a notification, so the behaviour the limit set decides.
	 */
	charging = status.intval == POWER_SUPPLY_STATUS_CHARGING;
	if (!power_supply_get_property(chip->chg_psy,
				       POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
				       &behaviour) &&
	    behaviour.intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO)
		charging = false;

	if (pmi8998_fg_sram_read(chip, SRAM_BATT_SOC_WORD, 0, buf, 4))
		return;
	soc = buf[3];
	bucket = soc / (256 / CYCLE_BUCKETS);
	dev_dbg(chip->dev, "cycles: soc %u bucket %d charging %d\n", soc,
		bucket, charging);

	if (charging) {
		if (!chip->cycle_started[bucket]) {
			chip->cycle_started[bucket] = true;
			chip->cycle_start_soc[bucket] = soc;
		}
		return;
	}

	for (i = 0; i < CYCLE_BUCKETS; i++) {
		if (!chip->cycle_started[i])
			continue;
		if (soc > chip->cycle_start_soc[i] + 2) {
			ret = pmi8998_fg_store_cycle(chip, i);
			dev_dbg(chip->dev, "cycles: bucket %d from soc %u: %d\n",
				i, chip->cycle_start_soc[i], ret);
			if (ret)
				dev_err(chip->dev, "Failed to store cycle %d: %d\n",
					i, ret);
		}
		chip->cycle_started[i] = false;
	}
}

static int pmi8998_fg_get_charge_counter(struct pmi8998_fg_chip *chip, int *val)
{
	u8 buf[4];
	int ret;

	if (!chip->nom_cap_uah)
		return -ENODATA;

	ret = pmi8998_fg_sram_read(chip, SRAM_CC_SOC_WORD, 0, buf, 4);
	if (ret)
		return ret;

	*val = div_s64((s64)(s32)get_unaligned_le32(buf) * chip->nom_cap_uah,
		       CC_SOC_FULL);

	return 0;
}

/************************
 * BATTERY POWER SUPPLY
 * **********************/

static enum power_supply_property pmi8998_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static int pmi8998_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct pmi8998_fg_chip *chip = power_supply_get_drvdata(psy);
	union power_supply_propval chg_status;
	int temp, ret = 0;

	dev_dbg(chip->dev, "Getting property: %d", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/*
		 * Ask the charger each time: it does not report every change
		 * of its charge state (charging to full, for one), so a
		 * status remembered from its last notification goes stale.
		 */
		if (chip->chg_psy &&
		    !power_supply_get_property(chip->chg_psy,
					       POWER_SUPPLY_PROP_STATUS,
					       &chg_status) &&
		    chg_status.intval != POWER_SUPPLY_STATUS_UNKNOWN) {
			val->intval = chg_status.intval;
		} else {
			/*
			 * Fall back to capacity and current-based
			 * status checking
			 */
			ret = pmi8998_fg_get_capacity(chip, &temp);
			if (ret) {
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				break;
			}
			if (temp == 100) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
				break;
			}

			ret = pmi8998_fg_get_current(chip, &temp);
			if (ret) {
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
				break;
			}
			if (temp < 0)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if (temp > 0)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = pmi8998_fg_get_capacity(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = pmi8998_fg_get_current(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = pmi8998_fg_get_voltage(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = pmi8998_fg_get_temperature(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
	case POWER_SUPPLY_PROP_TEMP_MAX:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = pmi8998_fg_get_temp_threshold(chip, psp, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = pmi8998_fg_get_charge_counter(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (!chip->act_cap_uah && !chip->nom_cap_uah)
			return -ENODATA;
		val->intval = chip->act_cap_uah ?: chip->nom_cap_uah;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		temp = 0;
		for (int i = 0; i < CYCLE_BUCKETS; i++)
			temp += chip->cycles[i];
		val->intval = DIV_ROUND_CLOSEST(temp, CYCLE_BUCKETS);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = chip->charge_start;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = chip->charge_end;
		break;
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}

	return ret;
}

static int pmi8998_fg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct pmi8998_fg_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		if (val->intval < 0 || val->intval >= chip->charge_end)
			return -EINVAL;
		chip->charge_start = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (val->intval <= chip->charge_start || val->intval > 100)
			return -EINVAL;
		chip->charge_end = val->intval;
		break;
	default:
		return -EINVAL;
	}

	schedule_work(&chip->charge_limit_work);

	return 0;
}

static int pmi8998_fg_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "qcom-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pmi8998_fg_props,
	.num_properties = ARRAY_SIZE(pmi8998_fg_props),
	.get_property = pmi8998_fg_get_property,
	.set_property = pmi8998_fg_set_property,
	.property_is_writeable = pmi8998_fg_property_is_writeable,
};

/********************
 * INIT FUNCTIONS
 * ******************/

static int pmi8998_fg_iacs_clear_sequence(struct pmi8998_fg_chip *chip)
{
	u8 temp;
	int ret;

	/* clear the error */
	ret = pmi8998_fg_masked_write(chip, MEM_INTF_IMA_CFG, BIT(2), BIT(2));
	if (ret) {
		dev_err(chip->dev, "Failed to write IMA_CFG: %d\n", ret);
		return ret;
	}

	temp = 0x4;
	ret = pmi8998_fg_write(chip, &temp, MEM_INTF_ADDR_LSB + 1, 1);
	if (ret) {
		dev_err(chip->dev, "Failed to write MEM_INTF_ADDR_MSB: %d\n", ret);
		return ret;
	}

	temp = 0x0;
	ret = pmi8998_fg_write(chip, &temp, MEM_INTF_WR_DATA0 + 3, 1);
	if (ret) {
		dev_err(chip->dev, "Failed to write WR_DATA3: %d\n", ret);
		return ret;
	}

	ret = pmi8998_fg_read(chip, &temp, MEM_INTF_RD_DATA0 + 3, 1);
	if (ret) {
		dev_err(chip->dev, "Failed to write RD_DATA3: %d\n", ret);
		return ret;
	}

	ret = pmi8998_fg_masked_write(chip, MEM_INTF_IMA_CFG, BIT(2), 0);
	if (ret) {
		dev_err(chip->dev, "Failed to write IMA_CFG: %d\n", ret);
		return ret;
	}

	return 0;
}

static int pmi8998_fg_clear_ima(struct pmi8998_fg_chip *chip, bool check_hw_sts)
{
	u8 err_sts, exp_sts, hw_sts;
	bool run_err_clr_seq = false;
	int ret;

	ret = pmi8998_fg_read(chip, &err_sts, MEM_INTF_IMA_ERR_STS, 1);
	if (ret) {
		dev_err(chip->dev, "Failed to read IMA_ERR_STS: %d\n", ret);
		return ret;
	}

	ret = pmi8998_fg_read(chip, &exp_sts,
			      MEM_INTF_IMA_EXP_STS, 1);
	if (ret) {
		dev_err(chip->dev, "Failed to read IMA_EXP_STS: %d\n", ret);
		return ret;
	}

	if (check_hw_sts) {
		ret = pmi8998_fg_read(chip, &hw_sts,
				      MEM_INTF_IMA_HW_STS, 1);
		if (ret) {
			dev_err(chip->dev, "Failed to read IMA_HW_STS: %d\n", ret);
			return ret;
		}
		/*
		 * Lower nibble should be equal to upper nibble before SRAM
		 * transactions begins from SW side.
		 */
		if ((hw_sts & 0x0f) != hw_sts >> 4) {
			dev_dbg(chip->dev, "IMA HW not in correct state, hw_sts=%x\n",
				hw_sts);
			run_err_clr_seq = true;
		}
	}

	if (exp_sts & (BIT(0) | BIT(1) | BIT(3) |
		BIT(4) | BIT(5) | BIT(6) |
		BIT(7))) {
		dev_dbg(chip->dev, "IMA exception bit set, exp_sts=%x\n", exp_sts);
		run_err_clr_seq = true;
	}

	if (run_err_clr_seq) {
		ret = pmi8998_fg_iacs_clear_sequence(chip);
		if (!ret)
			return -EAGAIN;
	}

	return 0;
}

/*
 * Run from the battery from charge_end down to charge_start, then charge
 * again; in between keep whatever the charger does, so the battery cycles
 * between the two while the phone stays plugged in. Inhibiting charging
 * with the input on is not enough: the battery still takes ~16 mA and
 * creeps up. An end threshold of 100 is no limit.
 */
static void pmi8998_fg_charge_limit_worker(struct work_struct *work)
{
	struct pmi8998_fg_chip *chip = container_of(work, struct pmi8998_fg_chip,
						    charge_limit_work);
	union power_supply_propval val;
	int capacity, behaviour;

	if (!chip->chg_psy || pmi8998_fg_get_capacity(chip, &capacity))
		return;

	if (chip->charge_end < 100 && capacity >= chip->charge_end)
		behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE;
	else if (chip->charge_end == 100 || capacity <= chip->charge_start)
		behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	else
		return;

	if (!power_supply_get_property(chip->chg_psy,
				       POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &val) &&
	    val.intval == behaviour)
		return;

	dev_info(chip->dev, "%s charging at %d%% (limit %d-%d%%)\n",
		 behaviour == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO ? "resume" : "stop",
		 capacity, chip->charge_start, chip->charge_end);

	val.intval = behaviour;
	power_supply_set_property(chip->chg_psy,
				  POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, &val);
	schedule_work(&chip->cycle_work);
}

static irqreturn_t pmi8998_fg_handle_soc_delta(int irq, void *data)
{
	struct pmi8998_fg_chip *chip = data;

	schedule_work(&chip->charge_limit_work);
	schedule_work(&chip->cycle_work);

	/* Signal change in state of charge */
	power_supply_changed(chip->batt_psy);
	dev_dbg(chip->dev, "SOC changed");

	return IRQ_HANDLED;
}

static void pmi8998_fg_status_changed_worker(struct work_struct *work)
{
	struct pmi8998_fg_chip *chip = container_of(work, struct pmi8998_fg_chip,
						    status_changed_work.work);

	power_supply_changed(chip->batt_psy);
}

static int pmi8998_fg_notifier_call(struct notifier_block *nb, unsigned long val, void *v)
{
	struct pmi8998_fg_chip *chip = container_of(nb, struct pmi8998_fg_chip, nb);
	struct power_supply *psy = v;
	union power_supply_propval propval;
	int ret;

	if (psy == chip->chg_psy) {
		ret = power_supply_get_property(psy,
						POWER_SUPPLY_PROP_STATUS, &propval);
		chip->status = ret ? POWER_SUPPLY_STATUS_UNKNOWN : propval.intval;

		power_supply_changed(chip->batt_psy);
		schedule_work(&chip->charge_limit_work);
		schedule_work(&chip->cycle_work);

		if (chip->status == POWER_SUPPLY_STATUS_UNKNOWN) {
			/*
			 * REVISIT: Find better solution or remove current-based
			 * status checking once checking is properly implemented
			 * in charger drivers

			 * Sometimes it take a while for current to stabilize,
			 * so signal property change again later to make sure
			 * current-based status is properly detected.
			 */
			cancel_delayed_work_sync(&chip->status_changed_work);
			schedule_delayed_work(&chip->status_changed_work,
					      msecs_to_jiffies(1000));
		}
	}

	return NOTIFY_OK;
}

static void pmi8998_fg_unregister_notifier(void *data)
{
	struct pmi8998_fg_chip *chip = data;

	power_supply_unreg_notifier(&chip->nb);
	cancel_delayed_work_sync(&chip->status_changed_work);
	power_supply_put(chip->chg_psy);
}

static int pmi8998_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct pmi8998_fg_chip *chip;
	const __be32 *prop_addr;
	int irq;
	u8 dma_status;
	bool error_present;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	mutex_init(&chip->sram_lock);
	chip->charge_start = 0;
	chip->charge_end = 100;
	ret = devm_work_autocancel(chip->dev, &chip->charge_limit_work,
				   pmi8998_fg_charge_limit_worker);
	if (ret)
		return ret;
	ret = devm_work_autocancel(chip->dev, &chip->cycle_work,
				   pmi8998_fg_cycle_worker);
	if (ret)
		return ret;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Failed to locate the regmap\n");
		return -ENODEV;
	}

	/* Get base address */
	prop_addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!prop_addr) {
		dev_err(chip->dev, "Failed to read SOC base address from dt\n");
		return -EINVAL;
	}
	chip->base = be32_to_cpu(*prop_addr);

	/*
	 * Change the FG_MEM_INT interrupt to track IACS_READY
	 * condition instead of end-of-transaction. This makes sure
	 * that the next transaction starts only after the hw is ready.
	 * IACS_INTR_SRC_SLCT is BIT(3)
	 */
	ret = pmi8998_fg_masked_write(chip, MEM_INTF_IMA_CFG, BIT(3), BIT(3));
	if (ret) {
		dev_err(chip->dev,
			"Failed to configure interrupt sourete: %d\n", ret);
		return ret;
	}

	ret = pmi8998_fg_clear_ima(chip, true);
	if (ret && ret != -EAGAIN) {
		dev_err(chip->dev, "Failed to clear IMA exception: %d\n", ret);
		return ret;
	}

	/* Check and clear DMA errors */
	ret = pmi8998_fg_read(chip, &dma_status, MEM_IF_DMA_STS, 1);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read dma_status: %d\n", ret);
		return ret;
	}

	error_present = dma_status & (BIT(1) | BIT(2));
	ret = pmi8998_fg_masked_write(chip, MEM_IF_DMA_CTL, BIT(0),
				      error_present ? BIT(0) : 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to write dma_ctl: %d\n", ret);
		return ret;
	}

	ret = pmi8998_fg_read_capacities(chip);
	if (ret)
		dev_warn(chip->dev, "No capacity from SRAM, no charge counter: %d\n", ret);
	else
		dev_info(chip->dev, "nominal capacity %d mAh, learned %d mAh\n",
			 chip->nom_cap_uah / 1000, chip->act_cap_uah / 1000);

	ret = pmi8998_fg_read_cycles(chip);
	if (ret)
		dev_warn(chip->dev, "No cycle counts from SRAM: %d\n", ret);
	else
		dev_info(chip->dev, "cycle counts %*ph\n",
			 (int)sizeof(chip->cycles), chip->cycles);

	supply_config.drv_data = chip;

	chip->batt_psy = devm_power_supply_register(chip->dev,
						    &batt_psy_desc, &supply_config);
	if (IS_ERR(chip->batt_psy)) {
		if (PTR_ERR(chip->batt_psy) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to register battery\n");
		return PTR_ERR(chip->batt_psy);
	}

	platform_set_drvdata(pdev, chip);

	/* Get soc-delta IRQ */
	irq = of_irq_get_byname(pdev->dev.of_node, "soc-delta");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq soc-delta byname: %d\n",
			irq);
		return irq;
	}

	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					pmi8998_fg_handle_soc_delta,
					IRQF_ONESHOT, "soc-delta", chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request soc-delta IRQ: %d\n", ret);
		return ret;
	}

	/* keeps the charge limit while suspended on a charger */
	device_init_wakeup(chip->dev, true);
	enable_irq_wake(irq);

	/* Optional: Get charger power supply for status checking */
	chip->chg_psy = power_supply_get_by_reference(dev_fwnode(chip->dev),
						    "power-supplies");
	if (IS_ERR(chip->chg_psy)) {
		ret = PTR_ERR(chip->chg_psy);
		dev_warn(chip->dev, "Failed to get charger supply: %d\n", ret);
		chip->chg_psy = NULL;
	}

	if (chip->chg_psy) {
		INIT_DELAYED_WORK(&chip->status_changed_work,
				  pmi8998_fg_status_changed_worker);

		chip->nb.notifier_call = pmi8998_fg_notifier_call;
		ret = power_supply_reg_notifier(&chip->nb);
		if (ret) {
			dev_err(chip->dev,
				"Failed to register notifier: %d\n", ret);
			return ret;
		}

		ret = devm_add_action_or_reset(chip->dev,
					       pmi8998_fg_unregister_notifier,
					       chip);
		if (ret)
			return ret;

		schedule_work(&chip->charge_limit_work);
	}

	return 0;
}

static const struct of_device_id fg_match_id_table[] = {
	{ .compatible = "qcom,pmi8998-fg" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fg_match_id_table);

static struct platform_driver pmi8998_fg_driver = {
	.probe = pmi8998_fg_probe,
	.driver = {
		.name = "pmi8998-fg",
		.of_match_table = fg_match_id_table,
	},
};

module_platform_driver(pmi8998_fg_driver);

MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
MODULE_AUTHOR("Joel Selvaraj <jo@jsfamily.in>");
MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_DESCRIPTION("Qualcomm PMIC Fuel Gauge Driver");
MODULE_LICENSE("GPL v2");
