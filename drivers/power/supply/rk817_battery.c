#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/mfd/rk808.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>

enum charge_status {
	CHRG_OFF,
	DEAD_CHRG,
	TRICKLE_CHRG,
	CC_OR_CV_CHRG,
	CHARGE_FINISH,
	USB_OVER_VOL,
	BAT_TMP_ERR,
	BAT_TIM_ERR,
};

struct rk817_bat {
	struct platform_device *pdev;
	struct device *dev;
	struct i2c_client *client;

	struct power_supply *bat_ps;
	struct power_supply_battery_info info;

	struct regmap *regmap;
	struct regmap_field *rmap_fields[ARRAY_SIZE(rk817_bat_reg_fields)];

	/* FIXME: do i move it into other struct? */
	uint32_t voltage_k;
	uint32_t voltage_b;
	int res_div;
	int sample_res;
};

static int rk817_get_reg_hl(struct rk817_bat *battery, int fieldH, int fieldL)
{
	int tmp, ret;
	uint32_t out;

	ret = regmap_field_read(battery->rmap_fields[fieldL], &tmp);
	if (ret)
		return ret;
	out |= tmp;

	ret = regmap_field_read(battery->rmap_fields[fieldH], &tmp);
	if (ret)
		return ret;
	out |= tmp << 8;

	return out;
}

static void rk817_write_reg_hl(struct rk817_bat *battery, int fieldH, int fieldL, int val)
{
	uint8_t tmp;

	tmp = val & 0xff;
	regmap_field_write(battery->rmap_fields[fieldL], tmp);
	tmp = (val >> 8) & 0xff;
	regmap_field_write(battery->rmap_fields[fieldH], tmp);
}

static void rk817_bat_calib_vol(struct rk817_bat *battery)
{
	uint32_t vcalib0 = 0;
	uint32_t vcalib1 = 0;

	/* calibrate voltage */
	vcalib0 = rk817_get_reg_hl(battery, VCALIB0_H, VCALIB0_L);
	vcalib1 = rk817_get_reg_hl(battery, VCALIB1_H, VCALIB1_L);

	/* values were taken from BSP kernel */
	battery->voltage_k = (4025 - 2300) * 1000 / ((vcalib1 - vcalib0) ? (vcalib1 - vcalib0) : 1);
	battery->voltage_b = 4025 - (battery->voltage_k * vcalib1) / 1000;

}

static void rk817_bat_calib_cur(struct rk817_bat *battery)
{
	int ioffset = 0;
	/* calibrate current */
	ioffset = rk817_get_reg_hl(battery, IOFFSET_H, IOFFSET_L);
	rk817_write_reg_hl(battery, CAL_OFFSET_H, CAL_OFFSET_L, ioffset);
}

static int rk817_bat_get_prop(struct power_supply *ps,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct rk817_bat *battery = power_supply_get_drvdata(ps);
	uint32_t tmp = 0;
	int ret = 0;

	/* recalibrate voltage and current readings if we need to */
	/* BSP does both on CUR_CALIB_UPD, ignoring VOL_CALIB_UPD */
	ret = regmap_field_read(battery->rmap_fields[CUR_CALIB_UPD], &tmp);
	if (tmp == 0)
	{
		rk817_bat_calib_vol(battery);
		rk817_bat_calib_cur(battery);
		regmap_field_write(battery->rmap_fields[CUR_CALIB_UPD], 1);
	}

	switch (prop) {
		case POWER_SUPPLY_PROP_STATUS:
			ret = regmap_field_read(battery->rmap_fields[CHG_STS], &tmp);
			if (ret)
				return ret;
			switch(tmp) {
				case CHRG_OFF:
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					break;
				case DEAD_CHRG:
					val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
					break;
				case TRICKLE_CHRG:
				case CC_OR_CV_CHRG:
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
					break;
				case CHARGE_FINISH:
					val->intval = POWER_SUPPLY_STATUS_FULL;
					break;
				default:
					val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
					printk("Error getting battery value");
					return -EINVAL;

			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			ret = regmap_field_read(battery->rmap_fields[CHG_STS], &tmp);
			if (ret)
				return ret;
			switch(tmp) {
				case CHRG_OFF:
				case DEAD_CHRG:
					val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
					break;
				case TRICKLE_CHRG:
					val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
					break;
				case CC_OR_CV_CHRG:
					val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
					break;
				default:
					val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
					break;
			}
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			val->intval = battery->info.voltage_min_design_uv;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			tmp = rk817_get_reg_hl(battery, BAT_VOL_H, BAT_VOL_L);
			val->intval = 1000 * ((battery->voltage_k * tmp) / (1000 + battery->voltage_b));
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = battery->info.voltage_max_design_uv;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}



static enum power_supply_property rk817_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static const struct power_supply_desc rk817_desc = {
	.name = "rk817-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = rk817_bat_props,
	.num_properties = ARRAY_SIZE(rk817_bat_props),
	.get_property = rk817_bat_get_prop,
};

#ifdef CONFIG_OF
static const struct of_device_id rk817_bat_of_match[] = {
	{ .compatible = "rk817,battery", },
	{ },
};
MODULE_DEVICE_TABLE(of, rk817_bat_of_match);
#endif

static int rk817_bat_probe(struct platform_device *pdev)
{
	struct rk817_bat *battery;
	struct device *dev = &pdev->dev;
	struct power_supply_config pscfg = {};
	int i, ret;

	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	battery = devm_kzalloc(&pdev->dev, sizeof(battery), GFP_KERNEL);

	if (!battery)
		return -ENOMEM;

	battery->dev = &pdev->dev;
	battery->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	platform_set_drvdata(pdev, battery);

	for (i = 0; i < ARRAY_SIZE(rk817_bat_reg_fields); i++) {
		const struct reg_field *reg_fields = rk817_bat_reg_fields;

		battery->rmap_fields[i] = devm_regmap_field_alloc(dev, battery->regmap, reg_fields[i]);

		if (IS_ERR(battery->rmap_fields[i])) {
			dev_err(dev, "Unable to allocate regmap field\n");

			return PTR_ERR(battery->rmap_fields[i]);
		}
	}

	rk817_bat_calib_vol(battery);


	pscfg.drv_data = battery;
	pscfg.of_node = pdev->dev.of_node;

	/* some boards may use different values for sample resistor */
	ret = of_property_read_u32(pdev->dev.of_node, "sample_res", &battery->sample_res);
	if (ret < 0)
		dev_err(dev, "Error reading sample_res");
	battery->res_div = (battery->sample_res == 20) ? 1 : 2;

	battery->bat_ps = devm_power_supply_register(&pdev->dev, &rk817_desc, &pscfg);

	ret = power_supply_get_battery_info(battery->bat_ps, &battery->info);
	if (ret) {
		dev_err(dev, "Unable to get battery info: %d\n", ret);
		return ret;
	}

	return 0;
}


static struct platform_driver rk817_bat_driver = {
	.probe    = rk817_bat_probe,
	.driver   = {
		.name  = "rk817-battery",
		.of_match_table = rk817_bat_of_match,
	},
};
module_platform_driver(rk817_bat_driver);

MODULE_DESCRIPTION("Battery power supply driver for RK817 PMIC");
MODULE_AUTHOR("Maciej Matuszczyk <maccraft123mc@gmail.com>");
MODULE_LICENSE("GPL");

