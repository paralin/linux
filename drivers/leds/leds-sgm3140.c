// SPDX-License-Identifier: GPL-2.0

#include <linux/gpio/consumer.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <media/v4l2-flash-led-class.h>

#define SGM3140_NAME "sgm3140"

struct sgm3140 {
	struct gpio_desc *flash_gpio;
	struct gpio_desc *enable_gpio;

	struct led_classdev_flash fled_cdev;
	struct v4l2_flash *v4l2_flash;

	struct timer_list powerdown_timer;
};

static struct sgm3140 *flcdev_to_sgm3140(struct led_classdev_flash *flcdev)
{
	return container_of(flcdev, struct sgm3140, fled_cdev);
}

int sgm3140_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	struct sgm3140 *priv = flcdev_to_sgm3140(fled_cdev);

	if (state) {
		gpiod_set_value_cansleep(priv->flash_gpio, 1);
		gpiod_set_value_cansleep(priv->enable_gpio, 1);
		mod_timer(&priv->powerdown_timer,
			  jiffies + msecs_to_jiffies(250));
	} else {
		gpiod_set_value_cansleep(priv->enable_gpio, 0);
		gpiod_set_value_cansleep(priv->flash_gpio, 0);
		del_timer_sync(&priv->powerdown_timer);
	}

	return 0;
}

struct led_flash_ops sgm3140_flash_ops = {
	.strobe_set = sgm3140_strobe_set,
};

int sgm3140_brightness_set(struct led_classdev *led_cdev,
			   enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct sgm3140 *priv = flcdev_to_sgm3140(fled_cdev);

	if (brightness == LED_OFF) {
		gpiod_set_value_cansleep(priv->enable_gpio, 0);
	} else {
		gpiod_set_value_cansleep(priv->enable_gpio, 1);
	}

	return 0;
}

static void sgm3140_powerdown_timer(struct timer_list *t)
{
	struct sgm3140 *priv = from_timer(priv, t, powerdown_timer);

	gpiod_set_value_cansleep(priv->enable_gpio, 0);
	gpiod_set_value_cansleep(priv->flash_gpio, 0);
}

#if IS_ENABLED(CONFIG_V4L2_FLASH_LED_CLASS)
static void sgm3140_init_v4l2_flash_config(struct sgm3140 *priv,
					   struct v4l2_flash_config *v4l2_sd_cfg)
{
	struct led_classdev *led_cdev = &priv->fled_cdev.led_cdev;
	struct led_flash_setting *s;

	strlcpy(v4l2_sd_cfg->dev_name, led_cdev->dev->kobj.name,
		sizeof(v4l2_sd_cfg->dev_name));

	s = &v4l2_sd_cfg->intensity;
	s->min = 0;
	s->max = 1;
	s->step = 1;
	s->val = 1;
}

#else
static void sgm3140_init_v4l2_flash_config(struct sgm3140 *priv,
					   struct v4l2_flash_config *v4l2_sd_cfg)
{
}
#endif

static int sgm3140_probe(struct platform_device *pdev)
{
	struct sgm3140 *priv;
	struct led_classdev *led_cdev;
	struct led_classdev_flash *fled_cdev;
	struct led_init_data init_data = {};
	struct device_node *child_node;
	struct v4l2_flash_config v4l2_sd_cfg;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->flash_gpio = devm_gpiod_get(&pdev->dev, "flash", GPIOD_OUT_LOW);
	ret = PTR_ERR_OR_ZERO(priv->flash_gpio);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request flash gpio: %d\n",
				ret);
		return ret;
	}

	priv->enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	ret = PTR_ERR_OR_ZERO(priv->enable_gpio);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to request enable gpio: %d\n",
				ret);
		return ret;
	}

	child_node = of_get_next_available_child(pdev->dev.of_node, NULL);
	if (!child_node) {
		dev_err(&pdev->dev, "No DT child node found for connected LED.\n");
		return -EINVAL;
	}

	timer_setup(&priv->powerdown_timer, sgm3140_powerdown_timer, 0);

	fled_cdev = &priv->fled_cdev;
	led_cdev = &fled_cdev->led_cdev;

	fled_cdev->ops = &sgm3140_flash_ops;
	// TODO ?
//	fled_cdev->timeout.max = 300000; /* 300ms */

	led_cdev->brightness_set_blocking = sgm3140_brightness_set;
	led_cdev->max_brightness = LED_ON;
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	init_data.fwnode = of_fwnode_handle(child_node);
	init_data.devicename = SGM3140_NAME;

	platform_set_drvdata(pdev, priv);

	/* Register in the LED subsystem */
	ret = led_classdev_flash_register_ext(&pdev->dev, fled_cdev, &init_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register flash device: %d\n",
			ret);
		goto err_flash_register;
	}

	sgm3140_init_v4l2_flash_config(priv, &v4l2_sd_cfg);

	/* Create V4L2 Flash subdev */
	priv->v4l2_flash = v4l2_flash_init(&pdev->dev, of_fwnode_handle(child_node),
					   fled_cdev, NULL,
					   &v4l2_sd_cfg);
	if (IS_ERR(priv->v4l2_flash)) {
		ret = PTR_ERR(priv->v4l2_flash);
		goto err_v4l2_flash_init;
	}

	return 0;

err_v4l2_flash_init:
	led_classdev_flash_unregister(fled_cdev);
err_flash_register:
	of_node_put(child_node);
	return ret;
}

static int sgm3140_remove(struct platform_device *pdev)
{
	struct sgm3140 *priv = platform_get_drvdata(pdev);

	del_timer_sync(&priv->powerdown_timer);

	v4l2_flash_release(priv->v4l2_flash);
	led_classdev_flash_unregister(&priv->fled_cdev);

	return 0;
}

static const struct of_device_id sgm3140_dt_match[] = {
	{ .compatible = "sgmicro,sgm3140" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sgm3140_dt_match);

static struct platform_driver sgm3140_driver = {
	.probe	= sgm3140_probe,
	.remove	= sgm3140_remove,
	.driver	= {
		.name	= "sgm3140",
		.of_match_table = sgm3140_dt_match,
	},
};

module_platform_driver(sgm3140_driver);

MODULE_AUTHOR("Luca Weiss <luca@z3ntu.xyz>");
MODULE_DESCRIPTION("SG Micro SGM3140 charge pump led driver");
MODULE_LICENSE("GPL v2");
