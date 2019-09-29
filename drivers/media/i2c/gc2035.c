/*
 * GC2035 sensor driver
 *
 * This program is free software; you can redistribute it and/or
 *modify it under the terms of the GNU General Public License as
 *published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 *kind, whether express or implied; without even the implied warranty
 *of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "gc2035.h"

#define GC2035_REG_CHIP_ID_HIGH		0xF0
#define GC2035_REG_CHIP_ID_LOW		0xF1

/* GC2035 has only one fixed colorspace per pixelcode */
struct gc2035_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

static const struct gc2035_datafmt gc2035_fmts[] = {
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},

};

enum gc2035_size {
	GC2035_SIZE_VGA,	/*  800 x 600 */
	GC2035_SIZE_UXGA,	/*  1600 x 1200 (2M) */
	GC2035_SIZE_LAST,
	GC2035_SIZE_MAX
};

enum cam_running_mode {
	CAM_RUNNING_MODE_NOTREADY,
	CAM_RUNNING_MODE_PREVIEW,
	CAM_RUNNING_MODE_CAPTURE,
	CAM_RUNNING_MODE_RECORDING,
};
enum cam_running_mode runmode;

static const struct v4l2_frmsize_discrete gc2035_frmsizes[GC2035_SIZE_LAST] = {
	{800, 600},
	{1600, 1200},
};

/* regulator supplies */
static const char * const gc2035_supply_name[] = {
	"DVDD",  /* Digital Core (1.8V) supply */
	"IOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
};
#define GC2035_NUM_SUPPLIES ARRAY_SIZE(gc2035_supply_name)

struct gc2035 {
	struct i2c_client *i2c_client;
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */

	int i_size;
	int i_fmt;
	short runmode;
	int init_needed;

	struct clk *xclk;
	struct regulator_bulk_data supplies[GC2035_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;

	struct mutex lock;
	bool streaming;
	int power_count;
};

/* Find a data format by a pixel code in an array */
static int gc2035_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gc2035_fmts); i++)
		if (gc2035_fmts[i].code == code)
			break;

	/* If not found, select latest */
	if (i >= ARRAY_SIZE(gc2035_fmts))
		i = ARRAY_SIZE(gc2035_fmts) - 1;

	return i;
}

static int gc2035_find_framesize(u32 width, u32 height)
{
	int i;

	for (i = 0; i < GC2035_SIZE_LAST; i++) {
		if ((gc2035_frmsizes[i].width >= width) &&
		    (gc2035_frmsizes[i].height >= height))
			break;
	}

	/* If not found, select biggest */
	if (i >= GC2035_SIZE_LAST)
		i = GC2035_SIZE_LAST - 1;

	return i;
}

static struct gc2035 *to_gc2035(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct gc2035, subdev);
}

/**
 * gc2035_reg_read - Read a value from a register in an gc2035 sensor device
 * @client: i2c driver client structure
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an gc2035 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int gc2035_read_smbus(struct i2c_client *client, unsigned char reg,
		unsigned char *value)
{
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);
	int ret;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		*value = (unsigned char)ret;
		ret = 0;
	}
	return ret;
}

static int gc2035_write_smbus(struct i2c_client *client, unsigned char reg,
		unsigned char value)
{
//	dev_err(&client->dev, "%s: in this func!!\n",
//		__func__);
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if (0 != ret)
		pr_err("%s:Error write reg=0x%x, value=0x%x, ret=%d\n",
		__func__, reg, value, ret);

	return ret;
}


/**
 * Write a value to a register in gc2035 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 *
 * Returns zero if successful, or non-zero otherwise.
 */
static int gc2035_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	ret = i2c_smbus_write_byte_data(client, (u8) (reg & 0xff), val);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/**
 * Initialize a list of gc2035 registers.
 * The list of registers is terminated by the pair of values
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 *
 * Returns zero if successful, or non-zero otherwise.
 */
static int gc2035_reg_writes(struct i2c_client *client,
			     const struct gc2035_reg reglist[])
{
	int err = 0, index;
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	for (index = 0; (reglist[index].reg != 0xffff); index++) {
		if ((reglist[index].reg == 0xfffe))
			msleep(reglist[index].val);
		else
			err = gc2035_write_smbus(client,
			(u8)(reglist[index].reg & 0xff), reglist[index].val);
	}

	return 0;
}

static int gc2035_config_preview(struct v4l2_subdev *sd)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	/* enable AEC */
	gc2035_reg_write(client, 0xfe, 0x00);
	gc2035_reg_write(client, 0xb6, 0x03);
	ret = gc2035_reg_writes(client, gc2035_mode[gc2035->i_size]);

	return ret;
}

static int gc2035_config_capture(struct v4l2_subdev *sd)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);
	uint8_t val = 0;
	uint16_t p_clk_div, p_shutter;
	uint16_t c_clk_div, c_shutter;
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	/* disable AEC/AGC */
	gc2035_reg_write(client, 0xfe, 0x00);
	gc2035_reg_write(client, 0xb6, 0x00);
	/* read preview div/shutter val */
	gc2035_read_smbus(client, 0xfa, &val);
	p_clk_div = ((val & 0xf0)>>4) + 1;
	gc2035_read_smbus(client, 0x03, &val);
	p_shutter = val;
	gc2035_read_smbus(client, 0x04, &val);
	p_shutter = (p_shutter << 8) | val;

	/* download capture settings */
	ret = gc2035_reg_writes(client, gc2035_mode[gc2035->i_size]);

	/* read capture clock div */
	/* set to page 0 */
	gc2035_reg_write(client, 0xfe, 0x00);
	gc2035_read_smbus(client, 0xfa, &val);
	c_clk_div = ((val & 0xf0)>>4) + 1;
	/* calc capture shutter */
	c_shutter = p_clk_div * p_shutter / c_clk_div;
	gc2035_reg_write(client, 0x03, (c_shutter & 0xFF00)>>8);
	gc2035_reg_write(client, 0x04, (c_shutter & 0x00FF));

	return ret;
}


static int gc2035_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);
	int ret = 0;

	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);
	dev_err(&client->dev, "%s: enable streaming: %d\n", __func__, enable);
	dev_err(&client->dev, "%s: streaming: %d\n", __func__, gc2035->streaming);
	dev_err(&client->dev, "%s: runmode: %d\n", __func__, gc2035->runmode);

	mutex_lock(&gc2035->lock);
	if (gc2035->streaming == !enable) {
		if (enable) {
			if (CAM_RUNNING_MODE_CAPTURE != gc2035->runmode)
				gc2035_config_preview(sd);
			else
				gc2035_config_capture(sd);
			/* Start Streaming */
			ret = gc2035_reg_writes(client, gc2035_streamon);
			msleep(50);
		} else {
			/* Stop Streaming, Power Down*/
			ret = gc2035_reg_writes(client, gc2035_streamoff);
		}
		if (!ret)
			gc2035->streaming = enable;
	}
	mutex_unlock(&gc2035->lock);

	return ret;
}

static int gc2035_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	if (format->pad != 0)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		// FIXME
		return -ENOTTY;
	}

	mf->width = gc2035_frmsizes[gc2035->i_size].width;
	mf->height = gc2035_frmsizes[gc2035->i_size].height;
	mf->code = gc2035_fmts[gc2035->i_fmt].code;
	mf->colorspace = gc2035_fmts[gc2035->i_fmt].colorspace;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int gc2035_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	int i_fmt;
	int i_size;
	dev_err(&to_gc2035(v4l2_get_subdevdata(sd))->i2c_client->dev, "%s: in this func!!\n",
		__func__);

	i_fmt = gc2035_find_datafmt(mf->code);

	mf->code = gc2035_fmts[i_fmt].code;
	mf->colorspace = gc2035_fmts[i_fmt].colorspace;
	mf->field = V4L2_FIELD_NONE;

	i_size = gc2035_find_framesize(mf->width, mf->height);

	mf->width = gc2035_frmsizes[i_size].width;
	mf->height = gc2035_frmsizes[i_size].height;

	return 0;
}

static int gc2035_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);
	int ret = 0;

	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);
	if (format->pad)
		return -EINVAL;

	ret = gc2035_try_fmt(sd, fmt);
	if (ret < 0)
		return ret;

	gc2035->i_size = gc2035_find_framesize(fmt->width, fmt->height);
	gc2035->i_fmt = gc2035_find_datafmt(fmt->code);

	/*To avoide reentry init sensor, remove from here	*/
	/*ret =  gc2035_reg_writes(client,configscript_common1);*/
//	if (gc2035->init_needed)
//		ret = gc2035_reg_writes(client, configscript_common1);
//
//	if (ret)
//		return ret;

	switch ((u32) gc2035_fmts[gc2035->i_fmt].code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		break;
	case MEDIA_BUS_FMT_JPEG_1X8:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int gc2035_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops gc2035_ctrl_ops = {
	.s_ctrl = gc2035_s_ctrl,
};

static const struct v4l2_ctrl_config gc2035_controls[] = {
	{
		.ops	= &gc2035_ctrl_ops,
		.id	= V4L2_CID_CAM_PREVIEW_ONOFF,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "PreviewOnOff",
		.min	= 0,
		.max	= 1,
		.step	= 1,
		.def	= 0,
		.flags	= V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.ops	= &gc2035_ctrl_ops,
		.id	= V4L2_CID_CAM_CAPTURE,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "Capture",
		.min	= 0,
		.max	= 1,
		.step	= 1,
		.def	= 0,
		.flags	= V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.ops	= &gc2035_ctrl_ops,
		.id	= V4L2_CID_CAM_MOUNTING,
		.type	= V4L2_CTRL_TYPE_INTEGER,
		.name	= "Sensor Mounting",
		.min	= 0,
		.max	= 0x10 | 0x20 | 0x40,
		.step	= 1,
		.def	= 0,
		.flags	= V4L2_CTRL_FLAG_VOLATILE,
	},
};

static int gc2035_set_colorfx(struct gc2035 *gc2035, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc2035->subdev);
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	switch (val) {
	case V4L2_COLORFX_NONE:
		return gc2035_reg_writes(client,
					 gc2035_effect_normal_tbl);
	case V4L2_COLORFX_BW:
		return gc2035_reg_writes(client,
					 gc2035_effect_bw_tbl);
	case V4L2_COLORFX_SEPIA:
		return gc2035_reg_writes(client,
					 gc2035_effect_sepia_tbl);
	case V4L2_COLORFX_NEGATIVE:
		return gc2035_reg_writes(client,
					 gc2035_effect_negative_tbl);
	}
	return -EINVAL;
}

static int gc2035_set_white_balance(struct gc2035 *gc2035, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gc2035->subdev);
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	switch (val) {
	case V4L2_WHITE_BALANCE_FLUORESCENT:
		gc2035_reg_write(client, 0xfe, 0x00);
		gc2035_reg_write(client, 0x82, 0xfc); /*turn off awb*/

		gc2035_reg_write(client, 0xb3, 0x72);
		gc2035_reg_write(client, 0xb4, 0x40);
		gc2035_reg_write(client, 0xb5, 0x5b);
		break;
	case V4L2_WHITE_BALANCE_DAYLIGHT:
		gc2035_reg_write(client, 0xfe, 0x00);
		gc2035_reg_write(client, 0x82, 0xfc); /*turn off awb*/

		gc2035_reg_write(client, 0xb3, 0x70);
		gc2035_reg_write(client, 0xb4, 0x40);
		gc2035_reg_write(client, 0xb5, 0x50);
		break;
	case V4L2_WHITE_BALANCE_CLOUDY:
		gc2035_reg_write(client, 0xfe, 0x00);
		gc2035_reg_write(client, 0x82, 0xfc); /*turn off awb*/

		gc2035_reg_write(client, 0xb3, 0x58);
		gc2035_reg_write(client, 0xb4, 0x40);
		gc2035_reg_write(client, 0xb5, 0x50);
		break;
	case V4L2_WHITE_BALANCE_INCANDESCENT:
		gc2035_reg_write(client, 0xfe, 0x00);
		gc2035_reg_write(client, 0x82, 0xfc); /*turn off awb*/

		gc2035_reg_write(client, 0xb3, 0xa0);
		gc2035_reg_write(client, 0xb4, 0x45);
		gc2035_reg_write(client, 0xb5, 0x40);
		break;
	case V4L2_WHITE_BALANCE_AUTO:
	default:    /*AWB*/
		gc2035_reg_write(client, 0xb3, 0x61);
		gc2035_reg_write(client, 0xb4, 0x40);
		gc2035_reg_write(client, 0xb5, 0x61);

		gc2035_reg_write(client, 0xfe, 0x00);
		gc2035_reg_write(client, 0x82, 0xfe); /*turn on awb*/
		break;
	}
	return 0;
}

static int gc2035_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2035 *gc2035 = container_of(ctrl->handler,
					     struct gc2035, hdl);
	dev_err(&gc2035->i2c_client->dev, "%s: in this func!!\n",
		__func__);

	switch (ctrl->id) {
	case V4L2_CID_COLORFX:
		return gc2035_set_colorfx(gc2035, ctrl->val);
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return gc2035_set_white_balance(gc2035, ctrl->val);
	case V4L2_CID_CAM_PREVIEW_ONOFF:
	{
		if (ctrl->val)
			gc2035->runmode = CAM_RUNNING_MODE_PREVIEW;
		else
			gc2035->runmode = CAM_RUNNING_MODE_NOTREADY;
		break;
	}

	case V4L2_CID_CAM_CAPTURE:
		if (ctrl->val)
			gc2035->runmode = CAM_RUNNING_MODE_CAPTURE;
		else
			gc2035->runmode = CAM_RUNNING_MODE_NOTREADY;
		break;
	}
	return 0;
}

static int gc2035_init_controls(struct gc2035 *gc2035)
{
	int ret;
	int i;
	dev_err(&gc2035->i2c_client->dev, "%s: in this func!!\n",
		__func__);

	/* Initializing hdl with 12 controls (3 for future purposes) */
	v4l2_ctrl_handler_init(&gc2035->hdl, ARRAY_SIZE(gc2035_controls) + 10);

	gc2035->hdl.lock = &gc2035->lock;

	/* register standard menu controls */
	/*
	 * Max value of some controls communicates the supported enums for that
	 * control to the HAL.
	 */
	v4l2_ctrl_new_std_menu(&gc2035->hdl, &gc2035_ctrl_ops,
		V4L2_CID_COLORFX, (1 << V4L2_COLORFX_NONE) |
		(1 << V4L2_COLORFX_NEGATIVE) | (1 << V4L2_COLORFX_SEPIA) |
		(1 << V4L2_COLORFX_BW), 0, V4L2_COLORFX_NONE);

	v4l2_ctrl_new_std_menu(&gc2035->hdl, &gc2035_ctrl_ops,
		V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		V4L2_WHITE_BALANCE_CLOUDY, 0,
		V4L2_WHITE_BALANCE_AUTO);

	/* register custom controls */
	for (i = 0; i < ARRAY_SIZE(gc2035_controls); ++i)
		v4l2_ctrl_new_custom(&gc2035->hdl, &gc2035_controls[i], NULL);

	if (gc2035->hdl.error) {
		ret = gc2035->hdl.error;
		goto free_ctrls;
	}

	gc2035->subdev.ctrl_handler = &gc2035->hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&gc2035->hdl);
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc2035_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->size > 2)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 2;
	if (gc2035_reg_read(client, reg->reg, &reg->val))
		return -EIO return 0;
}

static int gc2035_s_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->size > 2)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (gc2035_reg_write(client, reg->reg, reg->val))
		return -EIO;

	return 0;
}
#endif

static void gc2035_gpio_powerdown(struct gc2035 *gc2035, bool enable)
{
	gpiod_set_value_cansleep(gc2035->pwdn_gpio, enable ? 1 : 0);
}

static void gc2035_gpio_reset(struct gc2035 *gc2035, bool enable)
{
	gpiod_set_value_cansleep(gc2035->reset_gpio, enable ? 1 : 0);
}

static int gc2035_set_power_on(struct gc2035 *gc2035)
{
	struct i2c_client *client = gc2035->i2c_client;
	int ret;
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	ret = regulator_bulk_enable(GC2035_NUM_SUPPLIES,
				    gc2035->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(gc2035->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

	gc2035_gpio_powerdown(gc2035, false);

	gc2035_gpio_reset(gc2035, false);

	return 0;
}

static void gc2035_set_power_off(struct gc2035 *gc2035)
{
	dev_err(&gc2035->i2c_client->dev, "%s: in this func!!\n",
		__func__);
	gc2035_gpio_powerdown(gc2035, true);
	gc2035_gpio_reset(gc2035, true);
	clk_disable_unprepare(gc2035->xclk);
	regulator_bulk_disable(GC2035_NUM_SUPPLIES, gc2035->supplies);
}

static int gc2035_set_power(struct gc2035 *gc2035, int on)
{
	int ret = 0;
	struct i2c_client *client = gc2035->i2c_client;

	if (on) {
		ret = gc2035_set_power_on(gc2035);
		if (ret)
			return ret;
		// FIXME
		gc2035_reg_writes(client, sensor_default_regs);
	} else {
		// FIXME
		gc2035_set_power_off(gc2035);
	}

	return 0;
}

static int gc2035_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);
	int ret = 0;
	printk("Caller is %pS\n", __builtin_return_address(0));
	printk("Caller is %pF\n", __builtin_return_address(0));
	dev_err(&gc2035->i2c_client->dev, "%s: in this func!!\n",
		__func__);

	mutex_lock(&gc2035->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (gc2035->power_count == !on) {
		ret = gc2035_set_power(gc2035, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	gc2035->power_count += on ? 1 : -1;
	WARN_ON(gc2035->power_count < 0);
out:
	mutex_unlock(&gc2035->lock);

	// TODO Restore controls??
	gc2035->init_needed = 1; // FIXME
	dev_err(&gc2035->i2c_client->dev, "%s: ret: %d\n",
		__func__, ret);
	return ret;
}

static int gc2035_get_regulators(struct gc2035 *gc2035)
{
	int i;

	for (i = 0; i < GC2035_NUM_SUPPLIES; i++)
		gc2035->supplies[i].supply = gc2035_supply_name[i];

	return devm_regulator_bulk_get(&gc2035->i2c_client->dev,
				       GC2035_NUM_SUPPLIES,
				       gc2035->supplies);
}

static int gc2035_check_chip_id(struct gc2035 *gc2035)
{
	struct i2c_client *client = gc2035->i2c_client;
	int ret;
	u8 chip_id_hi, chip_id_lo;
	u16 chip_id;
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);

	ret = gc2035_set_power_on(gc2035);
	if (ret)
		return ret;

	ret = gc2035_read_smbus(client, GC2035_REG_CHIP_ID_HIGH, &chip_id_hi);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}
	ret = gc2035_read_smbus(client, GC2035_REG_CHIP_ID_LOW, &chip_id_lo);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}

	chip_id = ((u16)chip_id_hi << 8) | (u16)chip_id_lo;

	if (chip_id != 0x2145) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x2145, got 0x%x\n",
				__func__, chip_id);
		ret = -ENXIO;
	}

power_off:
	gc2035_set_power_off(gc2035);
	return ret;
}

static int gc2035_enum_code(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_mbus_code_enum *code)
{
	dev_err(&to_gc2035(v4l2_get_subdevdata(sd))->i2c_client->dev, "%s: in this func!!\n",
		__func__);
	if (code->pad || code->index >= ARRAY_SIZE(gc2035_fmts))
		return -EINVAL;

	code->code = gc2035_fmts[code->index].code;
	return 0;
}

static int gc2035_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	dev_err(&to_gc2035(v4l2_get_subdevdata(sd))->i2c_client->dev, "%s: in this func!!\n",
		__func__);
	if (fse->pad || fse->index >= GC2035_SIZE_LAST)
		return -EINVAL;

	fse->code = V4L2_PIX_FMT_UYVY;

	fse->min_width = fse->max_width = gc2035_frmsizes[fse->index].width;
	fse->min_height = fse->max_height = gc2035_frmsizes[fse->index].height;

	return 0;
}

/* we only support fixed frame rate */
static int gc2035_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	dev_err(&to_gc2035(v4l2_get_subdevdata(sd))->i2c_client->dev, "%s: in this func!!\n",
		__func__);
	int size;

	if (fie->pad)
		return -EINVAL;
	if (fie->index >= 1) // FIXME Use ARRAY_SIZE
		return -EINVAL;

	size = gc2035_find_framesize(fie->width, fie->height);

	switch (size) {
	case GC2035_SIZE_UXGA:
		fie->interval.numerator = 1;
		fie->interval.denominator = 4;
		break;

	case GC2035_SIZE_VGA:
	default:
		fie->interval.numerator = 1;
		fie->interval.denominator = 8;
		break;
	}
/*	printk(KERN_ERR"%s: width=%d height=%d fi=%d/%d\n", __func__,
			interval->width,
			interval->height, interval->discrete.numerator,
			interval->discrete.denominator);
			*/
	return 0;
}

static int gc2035_g_frame_interval(struct v4l2_subdev *sd,
			 struct v4l2_subdev_frame_interval *ival)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct gc2035 *gc2035 = to_gc2035(client);

	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);
	memset(ival->reserved, 0, sizeof(ival->reserved));

	switch (gc2035->i_size) {
	case GC2035_SIZE_UXGA:
		ival->interval.numerator = 1;
		ival->interval.denominator = 4;
		break;
	case GC2035_SIZE_VGA:
	default:
		ival->interval.numerator = 1;
		ival->interval.denominator = 8;
		break;
	}

	return 0;
}
static int gc2035_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *ival)
{
	dev_err(&to_gc2035(v4l2_get_subdevdata(sd))->i2c_client->dev, "%s: in this func!!\n",
		__func__);
	/*
	 * FIXME: This just enforces the hardcoded framerates until this is
	 *flexible enough.
	 */
	return gc2035_g_frame_interval(sd, ival);
}

static int gc2035_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	/* Quantity of initial bad frames to skip. Revisit. */
	/*Waitting for AWB stability,  avoid green color issue*/
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_err(&client->dev, "%s: in this func!!\n",
		__func__);
	struct gc2035 *gc2035 = to_gc2035(client);
	if (gc2035->init_needed) {
		/* frames need to be dropped for camera power on */
		*frames = 6;
		gc2035->init_needed = 0;
	} else {
		/* frames need to be dropped for camera mode switch */
		*frames = 2;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops gc2035_subdev_core_ops = {
	.s_power = gc2035_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc2035_g_register,
	.s_register = gc2035_s_register,
#endif
};

static const struct v4l2_subdev_video_ops gc2035_subdev_video_ops = {
	.s_stream = gc2035_s_stream,
	.g_frame_interval = gc2035_g_frame_interval,
	.s_frame_interval = gc2035_s_frame_interval,
};

static const struct v4l2_subdev_sensor_ops gc2035_subdev_sensor_ops = {
	.g_skip_frames = gc2035_g_skip_frames,
};

static const struct v4l2_subdev_pad_ops gc2035_subdev_pad_ops = {
	.set_fmt = gc2035_set_fmt,
	.get_fmt = gc2035_get_fmt,
	.enum_mbus_code = gc2035_enum_code,
	.enum_frame_size = gc2035_enum_frame_size,
	.enum_frame_interval = gc2035_enum_frame_interval,
};

static const struct v4l2_subdev_ops gc2035_subdev_ops = {
	.core = &gc2035_subdev_core_ops,
	.video = &gc2035_subdev_video_ops,
	.sensor = &gc2035_subdev_sensor_ops,
	.pad = &gc2035_subdev_pad_ops,
};

static int gc2035_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct gc2035 *gc2035;
	int ret;

	gc2035 = devm_kzalloc(&client->dev, sizeof(*gc2035), GFP_KERNEL);
	if (!gc2035)
		return -ENOMEM;

	gc2035->i2c_client = client;

	gc2035->i_size = GC2035_SIZE_VGA;
	gc2035->i_fmt = 0;	/* First format in the list */

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &gc2035->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	/* get system clock (xclk) */
	gc2035->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(gc2035->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(gc2035->xclk);
	}

	/* request optional power down pin */
	gc2035->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown",
						    GPIOD_OUT_LOW);
	/* request optional reset pin */
	gc2035->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	v4l2_i2c_subdev_init(&gc2035->subdev, client, &gc2035_subdev_ops);
	gc2035->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;

	gc2035->pad.flags = MEDIA_PAD_FL_SOURCE;
	gc2035->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&gc2035->subdev.entity, 1, &gc2035->pad);
	if (ret)
		return ret;

	ret = gc2035_get_regulators(gc2035);
	if (ret)
		return ret;

	mutex_init(&gc2035->lock);

	ret = gc2035_check_chip_id(gc2035);
	if (ret)
		goto entity_cleanup;

	ret = gc2035_init_controls(gc2035);
	if (ret) {
		printk(KERN_ERR "init_controls fail!\n");
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev(&gc2035->subdev);
	if (ret) {
		printk(KERN_ERR "register_subdev fail!\n");
		goto free_ctrls;
	}

	gc2035->runmode = CAM_RUNNING_MODE_NOTREADY;

	printk(KERN_ERR "gc2035 init complete!\n");

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&gc2035->hdl);
entity_cleanup:
	mutex_destroy(&gc2035->lock);
	media_entity_cleanup(&gc2035->subdev.entity);
	printk(KERN_ERR "gc2035 init fail! ret: %d \n", ret);
	return ret;
}

static int gc2035_remove(struct i2c_client *client)
{
	struct gc2035 *gc2035 = to_gc2035(client);

	if (!gc2035) {
		printk(KERN_ERR "gc2035 is null here!! %s\n", __func__);
	} else {
		printk(KERN_ERR "gc2035 is not null...\n");
	}

	v4l2_async_unregister_subdev(&gc2035->subdev);
	v4l2_ctrl_handler_free(&gc2035->hdl);
	mutex_destroy(&gc2035->lock);
	media_entity_cleanup(&gc2035->subdev.entity);
	//clk_disable_unprepare(gc2035->xclk); // TODO
	return 0;
}

static const struct i2c_device_id gc2035_id[] = {
	{ "gc2035", 0 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(i2c, gc2035_id);

static const struct of_device_id gc2035_of_match[] = {
	{ .compatible = "galaxycore,gc2035" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc2035_of_match);

static struct i2c_driver gc2035_i2c_driver = {
	.driver = {
		.name = "gc2035",
		.of_match_table = gc2035_of_match,
	},
	.id_table = gc2035_id,
	.probe = gc2035_probe,
	.remove = gc2035_remove,
};

module_i2c_driver(gc2035_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC2035 Camera driver");
MODULE_AUTHOR("Jun He <junhe@broadcom.com>");
MODULE_LICENSE("GPL v2");
