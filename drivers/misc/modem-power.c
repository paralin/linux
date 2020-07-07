/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Modem power sequencing driver.
 *
 * Ondrej Jirman <megous@megous.com>
 *
 * TODO:
 * - eg25
 *   - detect whether power regulator works by monitoring cts post power up
 *   - implement BH multiplexing of status/pwrkey GPIO
 *     - on powerdown turn off power to avoid re-start
 *   - detect powerdown via SW
 *     - cts going down for long periods of time (when status not avail)
 *     - status going down (when multiplexing with pwrkey or on 1.2)
 *   - implement rfkill (via enable pin)
 *   - implement efficient powerdown
 *     - monitor status if possible (1.1+)
 *     - monitor CTS on 1.0
 *   - implement suspend/resume hooks
 *     - assert sleep pin
 *   - implement resume via wakeup pin
 *     - enable wakeup on gpio
 */
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DRIVER_NAME "modem-power"

#define MPWR_IOCTL_RESET _IO('A', 0)
#define MPWR_IOCTL_POWERUP _IO('A', 1)
#define MPWR_IOCTL_POWERDN _IO('A', 2)
#define MPWR_IOCTL_STATUS _IOR('A', 3, int)

enum {
	MPWR_REQ_NONE = 0,
	MPWR_REQ_RESET,
	MPWR_REQ_PWDN,
	MPWR_REQ_PWUP,
};

struct mpwr_dev;

struct mpwr_gpio {
	const char* name;
	unsigned desc_off;
	int flags;
	bool required;
	int irq_flags;
	unsigned irq_off;
};

#define MPWR_GPIO_DEF(_name, _flags, _req) \
	{ .name = #_name, \
	  .desc_off = offsetof(struct mpwr_dev, _name##_gpio), \
	  .flags = _flags, \
	  .required = _req, \
	}

#define MPWR_GPIO_DEF_IRQ(_name, _flags, _req, _irq_flags) \
	{ .name = #_name, \
	  .desc_off = offsetof(struct mpwr_dev, _name##_gpio), \
	  .flags = _flags, \
	  .required = _req, \
	  .irq_flags = _irq_flags, \
	  .irq_off = offsetof(struct mpwr_dev, _name##_irq), \
	}

struct mpwr_variant {
	u32 reset_duration;
	unsigned int powerdown_delay;
	int (*power_init)(struct mpwr_dev* mpwr);
	int (*power_up)(struct mpwr_dev* mpwr);
	int (*power_down)(struct mpwr_dev* mpwr);
	int (*reset)(struct mpwr_dev* mpwr);
	const struct mpwr_gpio* gpios;
};

struct mpwr_dev {
	struct device *dev;
	const struct mpwr_variant* variant;

	/* power */
	struct regulator *regulator;

	/* outputs */
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwrkey_gpio;
	struct gpio_desc *sleep_gpio;
	struct gpio_desc *rts_gpio;
	struct gpio_desc *dtr_gpio;

	/* inputs */
	struct gpio_desc *cts_gpio;
	struct gpio_desc *status_gpio;
	struct gpio_desc *wakeup_gpio;
	int wakeup_irq;
	int status_irq;
	int cts_irq;

	bool status_pwrkey_multiplexed;

	/* config */
	struct cdev cdev;
	dev_t major;

	// change
	spinlock_t lock;
	wait_queue_head_t waitqueue;
	int got_wakeup;
	int is_open;
	struct workqueue_struct *wq;
	struct work_struct work;
	int last_request;
	int is_enabled;

	unsigned long flags[1];

	struct timer_list wd_timer;
        struct delayed_work dwork;

	unsigned int powerdown_delay;
};

enum {
	MPWR_F_POWERED,
	MPWR_F_CONNECTED,
	MPWR_F_FW_FAILED,
};

static int mpwr_generic_reset(struct mpwr_dev* mpwr)
{
	gpiod_set_value(mpwr->reset_gpio, 1);
	msleep(mpwr->variant->reset_duration);
	gpiod_set_value(mpwr->reset_gpio, 0);
	return 0;
}

// mg2723

static int mpwr_mg2723_power_init(struct mpwr_dev* mpwr)
{
	// if the device has power applied or doesn't have regulator
	// configured (we assume it's always powered) initialize GPIO
	// to shut it down initially
	if (!mpwr->regulator || regulator_is_enabled(mpwr->regulator)) {
		gpiod_set_value(mpwr->enable_gpio, 0);
		gpiod_set_value(mpwr->sleep_gpio, 1);
		gpiod_set_value(mpwr->reset_gpio, 1);
		gpiod_set_value(mpwr->pwrkey_gpio, 0);
	} else {
		// device is not powered, don't drive the gpios
		gpiod_direction_input(mpwr->enable_gpio);
		gpiod_direction_input(mpwr->reset_gpio);
		gpiod_direction_input(mpwr->sleep_gpio);
		gpiod_direction_input(mpwr->pwrkey_gpio);
	}

	return 0;
}

static int mpwr_mg2723_power_up(struct mpwr_dev* mpwr)
{
	int ret;

	// power up
	if (mpwr->regulator) {
		ret = regulator_enable(mpwr->regulator);
		if (ret < 0) {
			dev_err(mpwr->dev,
				"can't enable power supply err=%d", ret);
			return ret;
		}
	}

	gpiod_direction_output(mpwr->enable_gpio, 1);
	gpiod_direction_output(mpwr->sleep_gpio, 0);

	gpiod_direction_output(mpwr->reset_gpio, 1);
	msleep(mpwr->variant->reset_duration);
	gpiod_set_value(mpwr->reset_gpio, 0);

	return 0;
}

static int mpwr_mg2723_power_down(struct mpwr_dev* mpwr)
{
	gpiod_set_value(mpwr->enable_gpio, 0);
	gpiod_set_value(mpwr->sleep_gpio, 1);
	gpiod_set_value(mpwr->pwrkey_gpio, 0);
	msleep(50);

	if (mpwr->regulator) {
		regulator_disable(mpwr->regulator);

		gpiod_direction_input(mpwr->enable_gpio);
		gpiod_direction_input(mpwr->reset_gpio);
		gpiod_direction_input(mpwr->sleep_gpio);
		gpiod_direction_input(mpwr->pwrkey_gpio);
	} else {
		gpiod_set_value(mpwr->reset_gpio, 1);
	}

	return 0;
}

static const struct mpwr_gpio mpwr_mg2723_gpios[] = {
	MPWR_GPIO_DEF(enable, GPIOD_OUT_LOW, true),
	MPWR_GPIO_DEF(reset, GPIOD_OUT_HIGH, true),
	MPWR_GPIO_DEF_IRQ(wakeup, GPIOD_IN, true,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
	{ },
};

static const struct mpwr_variant mpwr_mg2723_variant = {
	.power_init = mpwr_mg2723_power_init,
	.power_up = mpwr_mg2723_power_up,
	.power_down = mpwr_mg2723_power_down,
	.reset = mpwr_generic_reset,
	.reset_duration = 300,
	.gpios = mpwr_mg2723_gpios,
};

// eg25

static int mpwr_eg25_power_up(struct mpwr_dev* mpwr)
{
	int ret;

	// power up
	if (mpwr->regulator) {
		if (regulator_is_enabled(mpwr->regulator)) {
			// if the regulator is still enabled
			dev_warn(mpwr->dev,
				 "regulator was already enabled during powerup");
		}

		ret = regulator_enable(mpwr->regulator);
		if (ret < 0) {
			dev_err(mpwr->dev,
				"can't enable power supply err=%d", ret);
			return ret;
		}
	} else {
		dev_err(mpwr->dev, "regulator required for eg25, none defined");
		return -ENODEV;
	}

	// drive default gpio signals during powerup
	gpiod_direction_output(mpwr->enable_gpio, 1);
	gpiod_direction_output(mpwr->sleep_gpio, 0);
	gpiod_direction_output(mpwr->reset_gpio, 0);
	gpiod_direction_output(mpwr->pwrkey_gpio, 0);

	// wait for powerup
	msleep(100);

	// send 200ms pwrkey pulse
	gpiod_set_value(mpwr->pwrkey_gpio, 1);
	msleep(200);
	gpiod_set_value(mpwr->pwrkey_gpio, 0);

	return 0;
}

static int mpwr_eg25_power_down(struct mpwr_dev* mpwr)
{
	// send 800ms pwrkey pulse
	gpiod_set_value(mpwr->pwrkey_gpio, 1);
	msleep(800);
	gpiod_set_value(mpwr->pwrkey_gpio, 0);

	// wait 30s for modem shutdown (usually shuts down in a few seconds)
	msleep(mpwr->powerdown_delay);

	// if it comes to powerdown we know we have a regulator configured
	// so we don't handle the else branch
	if (mpwr->regulator) {
		regulator_disable(mpwr->regulator);
		gpiod_direction_input(mpwr->enable_gpio);
		gpiod_direction_input(mpwr->reset_gpio);
		gpiod_direction_input(mpwr->sleep_gpio);
		gpiod_direction_input(mpwr->pwrkey_gpio);
	}

	return 0;
}

static const struct mpwr_gpio mpwr_eg25_gpios[] = {
	MPWR_GPIO_DEF(enable, GPIOD_OUT_LOW, true),
	MPWR_GPIO_DEF(reset, GPIOD_OUT_HIGH, true),
	MPWR_GPIO_DEF(pwrkey, GPIOD_OUT_LOW, true),
	MPWR_GPIO_DEF(sleep, GPIOD_OUT_LOW, false),
	MPWR_GPIO_DEF(dtr, GPIOD_OUT_LOW, false),
	MPWR_GPIO_DEF(rts, GPIOD_OUT_LOW, false),

	MPWR_GPIO_DEF_IRQ(status, GPIOD_IN, true,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
	MPWR_GPIO_DEF_IRQ(wakeup, GPIOD_IN, true,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
	MPWR_GPIO_DEF_IRQ(cts, GPIOD_IN, false,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
	{ },
};

static const struct mpwr_variant mpwr_eg25_variant = {
	.power_init = mpwr_mg2723_power_init,
	.power_up = mpwr_eg25_power_up,
	.power_down = mpwr_eg25_power_down,
	.reset = mpwr_generic_reset,
	.reset_duration = 20,
	.powerdown_delay = 30000,
	.gpios = mpwr_eg25_gpios,
};

static void mpwr_reset(struct mpwr_dev* mpwr)
{
	struct device *dev = mpwr->dev;
	int ret;

	if (!mpwr->is_enabled) {
		dev_err(dev, "reset requested but device is not enabled");
		return;
	}

	if (!mpwr->reset_gpio) {
		dev_err(dev, "reset is not configured for this device");
		return;
	}

	if (!mpwr->variant->reset) {
		dev_err(dev, "reset requested but not implemented");
		return;
	}

	//if (mpwr->wakeup_irq > 0)
		//disable_irq(mpwr->wakeup_irq);

	dev_info(dev, "resetting");
	ret = mpwr->variant->reset(mpwr);
	if (ret) {
		dev_err(dev, "reset failed");
	}

	//if (mpwr->wakeup_irq > 0)
		//enable_irq(mpwr->wakeup_irq);
}

static void mpwr_power_down(struct mpwr_dev* mpwr)
{
	struct device *dev = mpwr->dev;
	int ret;

	if (!mpwr->is_enabled)
		return;

	if (!mpwr->variant->power_down) {
		dev_err(dev, "power down requested but not implemented");
		return;
	}

	//if (mpwr->wakeup_irq > 0)
		//disable_irq(mpwr->wakeup_irq);

	dev_info(dev, "powering down");
	ret = mpwr->variant->power_down(mpwr);
	if (ret) {
		dev_err(dev, "power down failed");

		//if (mpwr->wakeup_irq > 0)
			//enable_irq(mpwr->wakeup_irq);
	} else {
		mpwr->is_enabled = 0;
	}
}

static void mpwr_power_up(struct mpwr_dev* mpwr)
{
	struct device *dev = mpwr->dev;
	int ret;

	if (mpwr->is_enabled)
		return;

	if (!mpwr->variant->power_up) {
		dev_err(dev, "power up requested but not implemented");
		return;
	}

	dev_info(dev, "powering up");
	ret = mpwr->variant->power_up(mpwr);
	if (ret) {
		dev_err(dev, "power up failed");
	} else {
		//if (mpwr->wakeup_irq > 0)
			//enable_irq(mpwr->wakeup_irq);

		mpwr->is_enabled = 1;
	}
}

static struct class* mpwr_class;

static void mpwr_work_handler(struct work_struct *work)
{
	struct mpwr_dev *mpwr = container_of(work, struct mpwr_dev, work);
	int last_request;

	spin_lock(&mpwr->lock);
	last_request = mpwr->last_request;
	mpwr->last_request = 0;
	spin_unlock(&mpwr->lock);

	if (last_request == MPWR_REQ_RESET) {
		mpwr_reset(mpwr);
	} else if (last_request == MPWR_REQ_PWDN) {
		mpwr_power_down(mpwr);
	} else if (last_request == MPWR_REQ_PWUP) {
		mpwr_power_up(mpwr);
	}
}

static bool mpwr_has_wakeup(struct mpwr_dev* mpwr)
{
	bool got_wakeup;
	spin_lock(&mpwr->lock);
	got_wakeup = mpwr->got_wakeup;
	spin_unlock(&mpwr->lock);
	return got_wakeup;
}

static ssize_t mpwr_read(struct file *fp, char __user *buf, size_t len,
			 loff_t *off)
{
	struct mpwr_dev* mpwr = fp->private_data;
	int ret;
	char tmp_buf[1] = {1};
	int got_wakeup;
	int non_blocking = fp->f_flags & O_NONBLOCK;

	// first handle non-blocking path
	if (non_blocking && !mpwr_has_wakeup(mpwr))
		return -EWOULDBLOCK;

	// wait for availability of wakeup
	ret = wait_event_interruptible(mpwr->waitqueue,
				       mpwr_has_wakeup(mpwr));
	if (ret)
		return ret;

	spin_lock(&mpwr->lock);

	got_wakeup = mpwr->got_wakeup;
	mpwr->got_wakeup = 0;

	if (!got_wakeup) {
		ret = -EIO;
	} else {
		if (copy_to_user(buf, tmp_buf, 1))
			ret = -EFAULT;
		else
			ret = 1;
	}

	spin_unlock(&mpwr->lock);
	return ret;
}

static ssize_t mpwr_write(struct file *fp, const char __user *buf,
			 size_t len, loff_t *off)
{
	struct mpwr_dev* mpwr = fp->private_data;
	int ret;
	char tmp_buf[1];
	int update = 0;

	if (len == 0)
		return 0;

	ret = copy_from_user(tmp_buf, buf, 1);
	if (ret)
		return -EFAULT;

	spin_lock(&mpwr->lock);

	switch (tmp_buf[0]) {
	case 'r':
		mpwr->last_request = MPWR_REQ_RESET;
		break;
	case 'u':
		mpwr->last_request = MPWR_REQ_PWUP;
		break;
	case 'd':
		mpwr->last_request = MPWR_REQ_PWDN;
		break;
	default:
		mpwr->last_request = MPWR_REQ_NONE;
		break;
	}

	update = mpwr->last_request != 0;

	spin_unlock(&mpwr->lock);

	if (update)
		queue_work(mpwr->wq, &mpwr->work);

	return 1;
}

static unsigned int mpwr_poll(struct file *fp, poll_table *wait)
{
	struct mpwr_dev* mpwr = fp->private_data;
	int ret = 0;

	poll_wait(fp, &mpwr->waitqueue, wait);

	spin_lock(&mpwr->lock);
	if (mpwr->got_wakeup)
		ret |= POLLIN | POLLRDNORM;
	spin_unlock(&mpwr->lock);

	return ret;
}

static long mpwr_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct mpwr_dev* mpwr = fp->private_data;
	unsigned long flags;
	int ret = -ENOSYS;
	void __user *parg = (void __user *)arg;
	int powered;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	spin_lock_irqsave(&mpwr->lock, flags);

	switch (cmd) {
	case MPWR_IOCTL_RESET:
		mpwr->last_request = MPWR_REQ_RESET;
		ret = 0;
		break;
	case MPWR_IOCTL_POWERUP:
		mpwr->last_request = MPWR_REQ_PWUP;
		ret = 0;
		break;
	case MPWR_IOCTL_POWERDN:
		mpwr->last_request = MPWR_REQ_PWDN;
		ret = 0;
		break;
	case MPWR_IOCTL_STATUS:
		powered = mpwr->is_enabled;
		spin_unlock_irqrestore(&mpwr->lock, flags);

		if (copy_to_user(parg, &powered, sizeof powered))
			return -EFAULT;

		return 0;
	}

	spin_unlock_irqrestore(&mpwr->lock, flags);

	if (ret == 0)
		queue_work(mpwr->wq, &mpwr->work);

	return ret;
}

static int mpwr_release(struct inode *ip, struct file *fp)
{
	struct mpwr_dev* mpwr = fp->private_data;

	spin_lock(&mpwr->lock);
	mpwr->is_open = 0;
	spin_unlock(&mpwr->lock);

	return 0;
}

static int mpwr_open(struct inode *ip, struct file *fp)
{
	struct mpwr_dev* mpwr = container_of(ip->i_cdev, struct mpwr_dev, cdev);

	fp->private_data = mpwr;

	spin_lock(&mpwr->lock);
	if (mpwr->is_open) {
		spin_unlock(&mpwr->lock);
		return -EBUSY;
	}

	mpwr->is_open = 1;
	spin_unlock(&mpwr->lock);

	nonseekable_open(ip, fp);
	return 0;
}

static const struct file_operations mpwr_fops = {
	.owner		= THIS_MODULE,
	.read		= mpwr_read,
	.write		= mpwr_write,
	.poll		= mpwr_poll,
	.unlocked_ioctl	= mpwr_ioctl,
	.open		= mpwr_open,
	.release	= mpwr_release,
	.llseek		= noop_llseek,
};

static irqreturn_t mpwr_gpio_isr(int irq, void *dev_id)
{
	struct mpwr_dev *mpwr = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&mpwr->lock, flags);

	if (irq == mpwr->wakeup_irq)
		mpwr->got_wakeup = 1;

	spin_unlock_irqrestore(&mpwr->lock, flags);

	if (irq == mpwr->wakeup_irq) {
		int val = gpiod_get_value(mpwr->wakeup_gpio);
		dev_err(mpwr->dev, "wakeup = %d\n", val);
	}

	if (irq == mpwr->cts_irq) {
		int val = gpiod_get_value(mpwr->cts_gpio);
		dev_err(mpwr->dev, "cts = %d\n", val);
	}

	if (irq == mpwr->status_irq) {
		int val = gpiod_get_value(mpwr->status_gpio);
		dev_err(mpwr->dev, "status = %d\n", val);
	}

	wake_up_interruptible(&mpwr->waitqueue);

	return IRQ_HANDLED;
}

static void mpwr_dwork(struct work_struct *work)
{
        //struct mpwr_dev *mpwr = container_of(work, struct mpwr_dev, dwork.work);

	//if (test_bit(ANX7688_F_FW_FAILED, anx7688->flags))
		//return;
}

static void mpwr_wd_timer_fn(struct timer_list *t)
{
	struct mpwr_dev *mpwr = from_timer(mpwr, t, wd_timer);

	//schedule_delayed_work(&mpwr->work, 0);

        //schedule_delayed_work(&anx7688->work, msecs_to_jiffies(10));

	mod_timer(t, jiffies + msecs_to_jiffies(50));
}

/* Sysfs attributes */

static ssize_t powered_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mpwr_dev *mpwr = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;
	unsigned int powered;

	spin_lock_irqsave(&mpwr->lock, flags);
	powered = !!mpwr->is_enabled;
	spin_unlock_irqrestore(&mpwr->lock, flags);

	return scnprintf(buf, PAGE_SIZE, "%u\n", powered);
}

static ssize_t powered_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct mpwr_dev *mpwr = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;
	bool status;
	int ret;

	ret = kstrtobool(buf, &status);
	if (ret)
		return ret;

	spin_lock_irqsave(&mpwr->lock, flags);
	mpwr->last_request = status ? MPWR_REQ_PWUP : MPWR_REQ_PWDN;
	spin_unlock_irqrestore(&mpwr->lock, flags);

	queue_work(mpwr->wq, &mpwr->work);

	return len;
}

static ssize_t powerdown_safety_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct mpwr_dev *mpwr = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;
	unsigned int delay;

	spin_lock_irqsave(&mpwr->lock, flags);
	delay = mpwr->powerdown_delay;
	spin_unlock_irqrestore(&mpwr->lock, flags);

	return scnprintf(buf, PAGE_SIZE, "%u\n", delay);
}

static ssize_t powerdown_safety_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct mpwr_dev *mpwr = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;
	unsigned int delay;
	int ret;

	ret = kstrtouint(buf, 0, &delay);
	if (ret)
		return ret;

	if (delay > 120000)
		return -ERANGE;

	spin_lock_irqsave(&mpwr->lock, flags);
	mpwr->powerdown_delay = delay;
	spin_unlock_irqrestore(&mpwr->lock, flags);

	return len;
}

static DEVICE_ATTR_RW(powered);
static DEVICE_ATTR_RW(powerdown_safety_delay);

static struct attribute *mpwr_attrs[] = {
	&dev_attr_powered.attr,
	&dev_attr_powerdown_safety_delay.attr,
	NULL,
};

static const struct attribute_group mpwr_group = {
	.attrs = mpwr_attrs,
};

static int mpwr_probe(struct platform_device *pdev)
{
	struct mpwr_dev *mpwr;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device *sdev;
	const char* cdev_name = NULL;
	int ret, i;

	mpwr = devm_kzalloc(dev, sizeof(*mpwr), GFP_KERNEL);
	if (!mpwr)
		return -ENOMEM;

	mpwr->variant = of_device_get_match_data(&pdev->dev);
	if (!mpwr->variant)
		return -EINVAL;
	mpwr->powerdown_delay = mpwr->variant->powerdown_delay;

	mpwr->dev = dev;
	platform_set_drvdata(pdev, mpwr);
	init_waitqueue_head(&mpwr->waitqueue);
	spin_lock_init(&mpwr->lock);
	INIT_WORK(&mpwr->work, &mpwr_work_handler);
        INIT_DELAYED_WORK(&mpwr->dwork, mpwr_dwork);

	ret = of_property_read_string(np, "char-device-name", &cdev_name);
	if (ret) {
		dev_err(dev, "char-device-name is not configured");
		return -EINVAL;
	}

	mpwr->status_pwrkey_multiplexed =
		of_property_read_bool(np, "status-pwrkey-multiplexed");

	mpwr->regulator = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(mpwr->regulator)) {
		ret = PTR_ERR(mpwr->regulator);
                if (ret != -ENODEV) {
			dev_err(dev, "can't get power supply err=%d", ret);
			return ret;
		}

		mpwr->regulator = NULL;
	}

	timer_setup(&mpwr->wd_timer, mpwr_wd_timer_fn, 0);
	mod_timer(&mpwr->wd_timer, jiffies + msecs_to_jiffies(1000));

	for (i = 0; mpwr->variant->gpios[i].name; i++) {
		const struct mpwr_gpio *io = &mpwr->variant->gpios[i];
		struct gpio_desc **desc = (struct gpio_desc **)((u8*)mpwr +
								io->desc_off);
		int *irq = (int*)((u8*)mpwr + io->irq_off);

		if (io->required)
			*desc = devm_gpiod_get(dev, io->name, io->flags);
		else
			*desc = devm_gpiod_get_optional(dev, io->name, io->flags);

		if (IS_ERR(*desc)) {
			dev_err(dev, "can't get %s gpio err=%ld", io->name,
				PTR_ERR(*desc));
			return PTR_ERR(*desc);
		}

		if (!*desc)
			continue;

		*irq = gpiod_to_irq(*desc);
		if (*irq <= 0) {
			dev_err(dev, "error converting %s gpio to irq: %d",
				io->name, ret);
			return *irq;
		}

		ret = devm_request_irq(dev, mpwr->wakeup_irq,
				       mpwr_gpio_isr,
				       io->irq_flags,
				       "modem-power-wakeup", mpwr);
		if (ret) {
			dev_err(dev, "error requesting %s irq: %d",
				io->name, ret);
			return ret;
		}

		//XXX: disable irq until we power up the modem
		//disable_irq(mpwr->wakeup_irq);
	}

	if (mpwr->status_pwrkey_multiplexed && mpwr->pwrkey_gpio) {
		dev_err(dev, "status and pwrkey is multiplexed, but pwrkey defined\n");
		return -EINVAL;
	}

	ret = devm_device_add_group(dev, &mpwr_group);
	if (ret)
		return ret;

	// create char device
	ret = alloc_chrdev_region(&mpwr->major, 0, 1, "modem-power");
	if (ret) {
		dev_err(dev, "can't allocate chrdev region");
		goto err_disable_regulator;
	}

	cdev_init(&mpwr->cdev, &mpwr_fops);
	mpwr->cdev.owner = THIS_MODULE;
	ret = cdev_add(&mpwr->cdev, mpwr->major, 1);
	if (ret) {
		dev_err(dev, "can't add cdev");
		goto err_unreg_chrev_region;
	}

	sdev = device_create(mpwr_class, dev, mpwr->major, mpwr, cdev_name);
	if (IS_ERR(sdev)) {
		ret = PTR_ERR(sdev);
		goto err_del_cdev;
	}

	mpwr->wq = alloc_workqueue("modem-power", WQ_SYSFS, 0);
	if (!mpwr->wq) {
		ret = -ENOMEM;
		dev_err(dev, "failed to allocate workqueue\n");
		goto err_free_dev;
	}

	mpwr->variant->power_init(mpwr);

	dev_info(dev, "modem power manager ready");

	return 0;

err_free_dev:
	device_destroy(mpwr_class, mpwr->major);
err_del_cdev:
	cdev_del(&mpwr->cdev);
err_unreg_chrev_region:
	unregister_chrdev(mpwr->major, "modem-power");
err_disable_regulator:
	cancel_work_sync(&mpwr->work);
	return ret;
}

static int mpwr_remove(struct platform_device *pdev)
{
	struct mpwr_dev *mpwr = platform_get_drvdata(pdev);

	del_timer_sync(&mpwr->wd_timer);
        cancel_delayed_work_sync(&mpwr->dwork);

	cancel_work_sync(&mpwr->work);
	destroy_workqueue(mpwr->wq);
	mpwr_power_down(mpwr);

	device_destroy(mpwr_class, mpwr->major);
	cdev_del(&mpwr->cdev);
	unregister_chrdev(mpwr->major, "modem-power");

	if (mpwr->wakeup_irq > 0)
		devm_free_irq(mpwr->dev, mpwr->wakeup_irq, mpwr);

	return 0;
}

static void mpwr_shutdown(struct platform_device *pdev)
{
	struct mpwr_dev *mpwr = platform_get_drvdata(pdev);

	cancel_work_sync(&mpwr->work);
	mpwr_power_down(mpwr);
}

static const struct of_device_id mpwr_of_match[] = {
	{ .compatible = "zte,mg3732",
	  .data = &mpwr_mg2723_variant },
	{ .compatible = "quectel,eg25",
	  .data = &mpwr_eg25_variant },
	{},
};
MODULE_DEVICE_TABLE(of, mpwr_of_match);

static struct platform_driver mpwr_platform_driver = {
	.probe = mpwr_probe,
	.remove = mpwr_remove,
	.shutdown = mpwr_shutdown,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = mpwr_of_match,
	},
};

static int __init mpwr_driver_init(void)
{
	int ret;

	mpwr_class = class_create(THIS_MODULE, "modem-power");
	if (IS_ERR(mpwr_class))
		return PTR_ERR(mpwr_class);

	ret = platform_driver_register(&mpwr_platform_driver);
	if (ret)
		class_destroy(mpwr_class);

	return ret;
}

static void __exit mpwr_driver_exit(void)
{
	platform_driver_unregister(&mpwr_platform_driver);
	class_destroy(mpwr_class);
}

module_init(mpwr_driver_init);
module_exit(mpwr_driver_exit);

MODULE_DESCRIPTION("Modem power manager");
MODULE_AUTHOR("Ondrej Jirman <megous@megous.com>");
MODULE_LICENSE("GPL v2");
