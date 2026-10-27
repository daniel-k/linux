// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM660L LCD bias (LCDB): a boost converter feeding a positive LDO
 * and a negative charge pump, which supply a LCD panel's VSP/VSN rails. One
 * module enable switches all three; the output voltages stay as the
 * bootloader programmed them.
 */

#include <linux/bits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define LCDB_INT_RT_STS			0x10
#define LCDB_VREG_OK			BIT(0)

#define LCDB_ENABLE_CTL1		0x46
#define LCDB_MODULE_EN			BIT(7)

struct qcom_lcdb {
	struct regmap *regmap;
	u16 base;
};

static int qcom_lcdb_enable(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);
	unsigned int sts;
	int ret;

	/* PM660L needs the module toggled once before it starts reliably */
	ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1,
			   LCDB_MODULE_EN);
	if (ret)
		return ret;

	ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1, 0);
	if (ret)
		return ret;

	ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1,
			   LCDB_MODULE_EN);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(lcdb->regmap,
				       lcdb->base + LCDB_INT_RT_STS, sts,
				       sts & LCDB_VREG_OK, 500, 20000);
	if (ret)
		dev_err(&rdev->dev, "output not OK after enable: %d\n", ret);

	return ret;
}

static int qcom_lcdb_disable(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);

	return regmap_write(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1, 0);
}

static int qcom_lcdb_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1, &val);
	if (ret)
		return ret;

	return !!(val & LCDB_MODULE_EN);
}

static const struct regulator_desc qcom_lcdb_desc = {
	.name = "lcdb",
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &(const struct regulator_ops) {
		.enable		= qcom_lcdb_enable,
		.disable	= qcom_lcdb_disable,
		.is_enabled	= qcom_lcdb_is_enabled,
	},
};

static int qcom_lcdb_probe(struct platform_device *pdev)
{
	struct regulator_config config = {};
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	struct qcom_lcdb *lcdb;
	u32 base;
	int ret;

	lcdb = devm_kzalloc(dev, sizeof(*lcdb), GFP_KERNEL);
	if (!lcdb)
		return -ENOMEM;

	lcdb->regmap = dev_get_regmap(dev->parent, NULL);
	if (!lcdb->regmap)
		return -ENODEV;

	ret = of_property_read_u32(dev->of_node, "reg", &base);
	if (ret)
		return ret;
	lcdb->base = base;

	config.dev = dev;
	config.of_node = dev->of_node;
	config.driver_data = lcdb;
	config.init_data = of_get_regulator_init_data(dev, dev->of_node,
						      &qcom_lcdb_desc);
	if (!config.init_data)
		return -ENOMEM;

	rdev = devm_regulator_register(dev, &qcom_lcdb_desc, &config);

	return PTR_ERR_OR_ZERO(rdev);
}

static const struct of_device_id qcom_lcdb_match_table[] = {
	{ .compatible = "qcom,pm660l-lcdb" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_lcdb_match_table);

static struct platform_driver qcom_lcdb_driver = {
	.probe = qcom_lcdb_probe,
	.driver = {
		.name = "qcom-lcdb-regulator",
		.of_match_table = qcom_lcdb_match_table,
	},
};
module_platform_driver(qcom_lcdb_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm PM660L LCD bias regulator driver");
