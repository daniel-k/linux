// SPDX-License-Identifier: GPL-2.0
/*
 * Type-C port of the SMB2 charger block in PMI8998 and PM660.
 *
 * Unlike PM8150B's separate Type-C peripheral, the SMB2 detection logic sits
 * in the charger's USBIN peripheral. It runs the attach state machine in
 * hardware and reports the result in TYPE_C_STATUS_1..5 through a single
 * "type-c-change" interrupt, which also fires on VBUS changes.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/tcpm.h>
#include <linux/workqueue.h>

#include "qcom_pmic_typec.h"
#include "qcom_pmic_typec_port.h"

#define TYPE_C_STATUS_1_REG				0x0b
#define UFP_TYPEC_RDSTD					BIT(7)
#define UFP_TYPEC_RD1P5					BIT(6)
#define UFP_TYPEC_RD3P0					BIT(5)

#define TYPE_C_STATUS_2_REG				0x0c
#define DFP_RA_OPEN					BIT(7)
#define DFP_RD_OPEN					BIT(3)
#define DFP_RD_RA_VCONN					BIT(2)
#define DFP_RD_RD					BIT(1)
#define DFP_RA_RA					BIT(0)

#define TYPE_C_STATUS_4_REG				0x0e
#define UFP_DFP_MODE_STATUS				BIT(7)
#define TYPEC_VBUS_STATUS				BIT(6)
#define TYPEC_DEBOUNCE_DONE_STATUS			BIT(4)
#define CC_ORIENTATION					BIT(1)
#define CC_ATTACHED					BIT(0)

#define TYPE_C_CFG_2_REG				0x59
#define EN_80UA_180UA_CUR_SOURCE			BIT(0)

#define TYPE_C_INTRPT_ENB_REG				0x67

#define TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG		0x68
#define VCONN_EN_ORIENTATION				BIT(6)
#define VCONN_EN_SRC					BIT(4)
#define VCONN_EN_VALUE					BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK			GENMASK(2, 0)
#define UFP_EN_CMD					BIT(2)
#define DFP_EN_CMD					BIT(1)
#define TYPEC_DISABLE_CMD				BIT(0)

struct pmic_typec_smb2 {
	struct device			*dev;
	struct tcpm_port		*tcpm_port;
	struct regmap			*regmap;
	u32				base;
	int				irq;
	struct regulator		*vdd_vbus;
	struct mutex			vbus_lock;
	bool				vbus_enabled;
	spinlock_t			lock;
	bool				debouncing_cc;
	struct delayed_work		cc_debounce_dwork;
	struct delayed_work		cc_kick_dwork;
};

static int smb2_read(struct pmic_typec_smb2 *port, unsigned int reg,
		     unsigned int *val)
{
	return regmap_read(port->regmap, port->base + reg, val);
}

static void qcom_pmic_typec_smb2_cc_debounce(struct work_struct *work)
{
	struct pmic_typec_smb2 *port =
		container_of(work, struct pmic_typec_smb2, cc_debounce_dwork.work);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	port->debouncing_cc = false;
	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 * A restart of the state machine that leaves the attach state as it was
 * raises no interrupt, so report the CC lines once it has settled.
 */
static void qcom_pmic_typec_smb2_cc_kick(struct work_struct *work)
{
	struct pmic_typec_smb2 *port =
		container_of(work, struct pmic_typec_smb2, cc_kick_dwork.work);

	tcpm_cc_change(port->tcpm_port);
}

static void qcom_pmic_typec_smb2_set_cc_debounce(struct pmic_typec_smb2 *port)
{
	port->debouncing_cc = true;
	schedule_delayed_work(&port->cc_debounce_dwork, msecs_to_jiffies(2));
}

static irqreturn_t qcom_pmic_typec_smb2_isr(int irq, void *dev_id)
{
	struct pmic_typec_smb2 *port = dev_id;
	unsigned long flags;
	bool cc_change;

	spin_lock_irqsave(&port->lock, flags);
	cc_change = !port->debouncing_cc;
	spin_unlock_irqrestore(&port->lock, flags);

	tcpm_vbus_change(port->tcpm_port);
	if (cc_change)
		tcpm_cc_change(port->tcpm_port);

	return IRQ_HANDLED;
}

static struct pmic_typec_smb2 *tcpc_to_smb2(struct tcpc_dev *tcpc)
{
	return tcpc_to_tcpm(tcpc)->pmic_typec_smb2;
}

static int qcom_pmic_typec_smb2_get_vbus(struct tcpc_dev *tcpc)
{
	struct pmic_typec_smb2 *port = tcpc_to_smb2(tcpc);
	unsigned int stat4 = 0;
	int ret;

	mutex_lock(&port->vbus_lock);
	smb2_read(port, TYPE_C_STATUS_4_REG, &stat4);
	ret = port->vbus_enabled || (stat4 & TYPEC_VBUS_STATUS);
	mutex_unlock(&port->vbus_lock);

	return ret;
}

static int qcom_pmic_typec_smb2_set_vbus(struct tcpc_dev *tcpc, bool on,
					 bool sink)
{
	struct pmic_typec *tcpm = tcpc_to_tcpm(tcpc);
	struct pmic_typec_smb2 *port = tcpm->pmic_typec_smb2;
	int ret = 0;

	mutex_lock(&port->vbus_lock);
	if (port->vbus_enabled == on)
		goto done;

	if (on)
		ret = regulator_enable(port->vdd_vbus);
	else
		ret = regulator_disable(port->vdd_vbus);
	if (ret)
		goto done;

	port->vbus_enabled = on;
	tcpm_vbus_change(tcpm->tcpm_port);

done:
	dev_dbg(port->dev, "set_vbus %d: %d\n", on, ret);
	mutex_unlock(&port->vbus_lock);

	return ret;
}

static int qcom_pmic_typec_smb2_get_cc(struct tcpc_dev *tcpc,
				       enum typec_cc_status *cc1,
				       enum typec_cc_status *cc2)
{
	struct pmic_typec_smb2 *port = tcpc_to_smb2(tcpc);
	enum typec_cc_status cc, other = TYPEC_CC_OPEN;
	unsigned int stat4, val;
	int ret;

	if (port->debouncing_cc)
		return -EBUSY;

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	ret = smb2_read(port, TYPE_C_STATUS_4_REG, &stat4);
	if (ret)
		return ret;

	if (!(stat4 & CC_ATTACHED) || !(stat4 & TYPEC_DEBOUNCE_DONE_STATUS))
		goto done;

	if (stat4 & UFP_DFP_MODE_STATUS) {
		/* Source: what terminates the partner's side */
		ret = smb2_read(port, TYPE_C_STATUS_2_REG, &val);
		if (ret)
			return ret;
		if (val & DFP_RA_RA) {
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			goto done;
		} else if (val & DFP_RD_RD) {
			*cc1 = TYPEC_CC_RD;
			*cc2 = TYPEC_CC_RD;
			goto done;
		}
		cc = TYPEC_CC_RD;
		if (val & DFP_RD_RA_VCONN)
			other = TYPEC_CC_RA;
	} else {
		/* Sink: the current the partner's Rp advertises */
		ret = smb2_read(port, TYPE_C_STATUS_1_REG, &val);
		if (ret)
			return ret;
		if (val & UFP_TYPEC_RD3P0)
			cc = TYPEC_CC_RP_3_0;
		else if (val & UFP_TYPEC_RD1P5)
			cc = TYPEC_CC_RP_1_5;
		else
			cc = TYPEC_CC_RP_DEF;
	}

	if (stat4 & CC_ORIENTATION) {
		*cc2 = cc;
		*cc1 = other;
	} else {
		*cc1 = cc;
		*cc2 = other;
	}

done:
	dev_dbg(port->dev, "get_cc: status_4 0x%02x cc1 %d cc2 %d\n",
		stat4, *cc1, *cc2);

	return 0;
}

static int qcom_pmic_typec_smb2_set_cc(struct tcpc_dev *tcpc,
				       enum typec_cc_status cc)
{
	struct pmic_typec_smb2 *port = tcpc_to_smb2(tcpc);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&port->lock, flags);

	switch (cc) {
	case TYPEC_CC_OPEN:
	case TYPEC_CC_RD:
		break;
	case TYPEC_CC_RP_DEF:
		ret = regmap_update_bits(port->regmap,
					 port->base + TYPE_C_CFG_2_REG,
					 EN_80UA_180UA_CUR_SOURCE, 0);
		break;
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		/* The OTG boost supplies at most 1.5 A here */
		ret = regmap_update_bits(port->regmap,
					 port->base + TYPE_C_CFG_2_REG,
					 EN_80UA_180UA_CUR_SOURCE,
					 EN_80UA_180UA_CUR_SOURCE);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (!ret)
		qcom_pmic_typec_smb2_set_cc_debounce(port);

	spin_unlock_irqrestore(&port->lock, flags);

	dev_dbg(port->dev, "set_cc %d: %d\n", cc, ret);

	return ret;
}

static int qcom_pmic_typec_smb2_set_polarity(struct tcpc_dev *tcpc,
					     enum typec_cc_polarity pol)
{
	return 0;
}

static int qcom_pmic_typec_smb2_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	struct pmic_typec_smb2 *port = tcpc_to_smb2(tcpc);
	unsigned int stat4, mask, val;
	int ret;

	ret = smb2_read(port, TYPE_C_STATUS_4_REG, &stat4);
	if (ret)
		return ret;

	/* VCONN goes on the CC line the partner does not use */
	if (on) {
		mask = VCONN_EN_ORIENTATION | VCONN_EN_VALUE;
		val = VCONN_EN_VALUE |
		      ((stat4 & CC_ORIENTATION) ? 0 : VCONN_EN_ORIENTATION);
	} else {
		mask = VCONN_EN_VALUE;
		val = 0;
	}

	ret = regmap_update_bits(port->regmap,
				 port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 mask, val);

	dev_dbg(port->dev, "set_vconn %d: status_4 0x%02x: %d\n", on, stat4, ret);

	return ret;
}

static int qcom_pmic_typec_smb2_start_toggling(struct tcpc_dev *tcpc,
					       enum typec_port_type port_type,
					       enum typec_cc_status cc)
{
	struct pmic_typec_smb2 *port = tcpc_to_smb2(tcpc);
	unsigned int reg = port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG;
	unsigned long flags;
	u8 mode = 0;
	int ret;

	switch (port_type) {
	case TYPEC_PORT_SRC:
		mode = DFP_EN_CMD;
		break;
	case TYPEC_PORT_SNK:
		mode = UFP_EN_CMD;
		break;
	case TYPEC_PORT_DRP:
		mode = 0;
		break;
	}

	spin_lock_irqsave(&port->lock, flags);
	qcom_pmic_typec_smb2_set_cc_debounce(port);

	/* Restart the state machine from idle so it toggles at least once */
	ret = regmap_update_bits(port->regmap, reg, TYPEC_POWER_ROLE_CMD_MASK,
				 TYPEC_DISABLE_CMD);
	if (!ret)
		ret = regmap_update_bits(port->regmap, reg,
					 TYPEC_POWER_ROLE_CMD_MASK, mode);
	spin_unlock_irqrestore(&port->lock, flags);

	schedule_delayed_work(&port->cc_kick_dwork, msecs_to_jiffies(50));

	dev_dbg(port->dev, "start_toggling %d: %d\n", port_type, ret);

	return ret;
}

static int qcom_pmic_typec_smb2_start(struct pmic_typec *tcpm,
				      struct tcpm_port *tcpm_port)
{
	struct pmic_typec_smb2 *port = tcpm->pmic_typec_smb2;
	int ret;

	/* VCONN under software control, off */
	ret = regmap_update_bits(port->regmap,
				 port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_SRC | VCONN_EN_VALUE, VCONN_EN_SRC);
	if (ret)
		return ret;

	/* type-c-change on every CC, debounce and VBUS change */
	ret = regmap_write(port->regmap, port->base + TYPE_C_INTRPT_ENB_REG, 0xff);
	if (ret)
		return ret;

	port->tcpm_port = tcpm_port;
	enable_irq(port->irq);

	return 0;
}

static void qcom_pmic_typec_smb2_stop(struct pmic_typec *tcpm)
{
	struct pmic_typec_smb2 *port = tcpm->pmic_typec_smb2;

	disable_irq(port->irq);
	cancel_delayed_work_sync(&port->cc_debounce_dwork);
	cancel_delayed_work_sync(&port->cc_kick_dwork);

	mutex_lock(&port->vbus_lock);
	if (port->vbus_enabled) {
		regulator_disable(port->vdd_vbus);
		port->vbus_enabled = false;
	}
	mutex_unlock(&port->vbus_lock);
}

int qcom_pmic_typec_port_smb2_probe(struct platform_device *pdev,
				    struct pmic_typec *tcpm,
				    const struct pmic_typec_port_resources *res,
				    struct regmap *regmap, u32 base)
{
	struct device *dev = &pdev->dev;
	struct pmic_typec_smb2 *port;
	int ret;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->vdd_vbus = devm_regulator_get(dev, "vdd-vbus");
	if (IS_ERR(port->vdd_vbus))
		return PTR_ERR(port->vdd_vbus);

	port->dev = dev;
	port->regmap = regmap;
	port->base = base;
	mutex_init(&port->vbus_lock);
	spin_lock_init(&port->lock);
	INIT_DELAYED_WORK(&port->cc_debounce_dwork,
			  qcom_pmic_typec_smb2_cc_debounce);
	INIT_DELAYED_WORK(&port->cc_kick_dwork, qcom_pmic_typec_smb2_cc_kick);

	port->irq = platform_get_irq_byname(pdev, "type-c-change");
	if (port->irq < 0)
		return port->irq;

	ret = devm_request_threaded_irq(dev, port->irq, NULL,
					qcom_pmic_typec_smb2_isr,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"type-c-change", port);
	if (ret)
		return ret;

	tcpm->pmic_typec_smb2 = port;

	tcpm->tcpc.get_vbus = qcom_pmic_typec_smb2_get_vbus;
	tcpm->tcpc.set_vbus = qcom_pmic_typec_smb2_set_vbus;
	tcpm->tcpc.set_cc = qcom_pmic_typec_smb2_set_cc;
	tcpm->tcpc.get_cc = qcom_pmic_typec_smb2_get_cc;
	tcpm->tcpc.set_polarity = qcom_pmic_typec_smb2_set_polarity;
	tcpm->tcpc.set_vconn = qcom_pmic_typec_smb2_set_vconn;
	tcpm->tcpc.start_toggling = qcom_pmic_typec_smb2_start_toggling;

	tcpm->port_start = qcom_pmic_typec_smb2_start;
	tcpm->port_stop = qcom_pmic_typec_smb2_stop;

	return 0;
}
