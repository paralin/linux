// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * LED Driver for X-Powers AXP813 PMIC and similar.
 *
 * Copyright(c) 2020 Ondrej Jirman <megous@megous.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/axp20x.h>

#define AXP20X_CHGLED_CTRL_MASK		BIT(3)
#define AXP20X_CHGLED_CTRL_CHARGER	BIT(3)
#define AXP20X_CHGLED_CTRL_USER		0

#define AXP20X_CHRG_CTRL2_MODE		BIT(4)

#define AXP20X_CHGLED_USER_STATE_MASK		GENMASK(5, 4)
#define AXP20X_CHGLED_USER_STATE_OFF		(0 << 4)
#define AXP20X_CHGLED_USER_STATE_BLINK_SLOW	(1 << 4)
#define AXP20X_CHGLED_USER_STATE_BLINK_FAST	(2 << 4)
#define AXP20X_CHGLED_USER_STATE_ON		(3 << 4)

struct axp20x_led {
	struct led_classdev cdev;
	struct regmap *regmap;
};

static int axp20x_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct axp20x_led *led =
			container_of(led_cdev, struct axp20x_led, cdev);
	unsigned int val;

	val = value == LED_OFF ? AXP20X_CHGLED_USER_STATE_OFF :
		AXP20X_CHGLED_USER_STATE_ON;

	return regmap_update_bits(led->regmap, AXP20X_OFF_CTRL,
				  AXP20X_CHGLED_USER_STATE_MASK, val);

}

static ssize_t charger_control_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct axp20x_led *led =
		container_of(led_cdev, struct axp20x_led, cdev);
	unsigned int val;
	int ret;

	ret = regmap_read(led->regmap, AXP20X_OFF_CTRL, &val);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 val & AXP20X_CHGLED_CTRL_MASK ? 1 : 0);
}

static ssize_t charger_control_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct axp20x_led *led =
			container_of(led_cdev, struct axp20x_led, cdev);
	bool status;
	int ret;

	ret = kstrtobool(buf, &status);
	if (ret)
		return ret;

	ret = regmap_update_bits(led->regmap, AXP20X_OFF_CTRL,
				 AXP20X_CHGLED_CTRL_MASK,
				 status ? AXP20X_CHGLED_CTRL_CHARGER :
				 AXP20X_CHGLED_CTRL_USER);
	if (ret)
		return ret;

	return len;
}

static ssize_t charger_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct axp20x_led *led =
		container_of(led_cdev, struct axp20x_led, cdev);
	unsigned int val;
	int ret;

	ret = regmap_read(led->regmap, AXP20X_CHRG_CTRL2, &val);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 val & AXP20X_CHRG_CTRL2_MODE ? 1 : 0);
}

static ssize_t charger_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct axp20x_led *led =
		container_of(led_cdev, struct axp20x_led, cdev);
	unsigned int mode;
	int ret;

	ret = kstrtouint(buf, 0, &mode);
	if (ret)
		return ret;

	if (mode > 1)
		return -ERANGE;

	ret = regmap_update_bits(led->regmap, AXP20X_CHRG_CTRL2,
				 AXP20X_CHRG_CTRL2_MODE,
				 mode ? AXP20X_CHRG_CTRL2_MODE : 0);
	if (ret)
		return ret;

	return len;
}

static DEVICE_ATTR_RW(charger_control);
static DEVICE_ATTR_RW(charger_mode);

static struct attribute *axp20x_led_attrs[] = {
	&dev_attr_charger_control.attr,
	&dev_attr_charger_mode.attr,
	NULL,
};

ATTRIBUTE_GROUPS(axp20x_led);

static int axp20x_led_probe(struct platform_device *pdev)
{
	struct axp20x_dev *axp20x;
	struct axp20x_led *led;
	int ret;

	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	axp20x = dev_get_drvdata(pdev->dev.parent);
	if (!axp20x)
		return -EINVAL;

	led = devm_kzalloc(&pdev->dev,
			   sizeof(struct axp20x_led),
			   GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	platform_set_drvdata(pdev, led);

	led->regmap = axp20x->regmap;

	led->cdev.name = "axp20x-chgarger-led";
	led->cdev.brightness_set_blocking = axp20x_led_set;
	led->cdev.brightness = LED_OFF;
	led->cdev.max_brightness = 1;
	led->cdev.groups = axp20x_led_groups,

	ret = devm_led_classdev_register(pdev->dev.parent, &led->cdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register device %s\n",
			led->cdev.name);
		return ret;
	}

	ret = regmap_update_bits(led->regmap, AXP20X_OFF_CTRL,
				 AXP20X_CHGLED_CTRL_MASK,
				 AXP20X_CHGLED_CTRL_CHARGER);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable charger control\n");
		return ret;
	}

	ret = axp20x_led_set(&led->cdev, led->cdev.brightness);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init led brightness\n");
		return ret;
	}

	return 0;
}

static void axp20x_led_shutdown(struct platform_device *pdev)
{
	struct axp20x_led *led = platform_get_drvdata(pdev);

	/* On shutdown, we want to give LED control back to the PMIC,
	 * so that it doesn't stay on, while the system is off.
	 */

	axp20x_led_set(&led->cdev, LED_OFF);

	regmap_update_bits(led->regmap, AXP20X_OFF_CTRL,
			   AXP20X_CHGLED_CTRL_MASK,
			   AXP20X_CHGLED_CTRL_CHARGER);
}

static int axp20x_led_remove(struct platform_device *pdev)
{
	axp20x_led_shutdown(pdev);

	return 0;
}

static const struct of_device_id axp20x_leds_of_match[] = {
	{ .compatible = "x-powers,axp813-charger-led", },
	{ },
};
MODULE_DEVICE_TABLE(of, axp20x_leds_of_match);

static struct platform_driver axp20x_led_driver = {
	.driver		= {
		.name	= "leds-axp20x",
		.of_match_table = axp20x_leds_of_match,
	},
	.probe		= axp20x_led_probe,
	.remove		= axp20x_led_remove,
	.shutdown	= axp20x_led_shutdown,
};

module_platform_driver(axp20x_led_driver);

MODULE_AUTHOR("Ondrej Jirman <megous@megous.com>");
MODULE_DESCRIPTION("LED driver for AXP813 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-axp20x");
