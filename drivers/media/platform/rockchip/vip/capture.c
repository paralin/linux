// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip VIP Camera Interface Driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 * Copyright (C) 2020 Maxime Chevallier <maxime.chevallier@bootlin.com>
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "dev.h"
#include "regs.h"

#define VIP_REQ_BUFS_MIN	3
#define VIP_MIN_WIDTH		64
#define VIP_MIN_HEIGHT		64
#define VIP_MAX_WIDTH		8192
#define VIP_MAX_HEIGHT		8192

#define RK_VIP_PLANE_Y			0
#define RK_VIP_PLANE_CBCR		1

#define VIP_FETCH_Y_LAST_LINE(VAL) ((VAL) & 0x1fff)
/* Check if swap y and c in bt1120 mode */
#define VIP_FETCH_IS_Y_FIRST(VAL) ((VAL) & 0xf)

static const struct vip_output_fmt out_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.fmt_val = VIP_FORMAT_YUV_OUTPUT_422 |
			   VIP_FORMAT_UV_STORAGE_ORDER_UVUV,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.fmt_val = VIP_FORMAT_YUV_OUTPUT_422 |
			   VIP_FORMAT_UV_STORAGE_ORDER_VUVU,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.fmt_val = VIP_FORMAT_YUV_OUTPUT_420 |
			   VIP_FORMAT_UV_STORAGE_ORDER_UVUV,
		.mbus = MEDIA_BUS_FMT_UYVY8_2X8,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.fmt_val = VIP_FORMAT_YUV_OUTPUT_420 |
			   VIP_FORMAT_UV_STORAGE_ORDER_VUVU,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB24,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR666,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB10,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG10,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG10,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR10,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG12,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR12,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR16,
	}, {
		.fourcc = V4L2_PIX_FMT_Y16,
	}
};

static const struct vip_input_fmt in_fmts[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_YUYV,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_YUYV,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_YVYU,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_YVYU,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_UYVY,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_UYVY,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_VYUY,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= VIP_FORMAT_YUV_INPUT_422 |
				  VIP_FORMAT_YUV_INPUT_ORDER_VYUY,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_YUV422,
		.fmt_type	= VIP_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_8,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_8,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_8,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_8,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_10,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_10,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_10,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_10,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_12,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_12,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_12,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_12,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RGB888,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_8,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_10,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
		.dvp_fmt_val	= VIP_FORMAT_INPUT_MODE_RAW |
				  VIP_FORMAT_RAW_DATA_WIDTH_12,
		.csi_fmt_val	= VIP_CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= VIP_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}
};

static const struct
vip_input_fmt *get_input_fmt(struct v4l2_subdev *sd)
{
	struct v4l2_subdev_format fmt;
	int ret;
	u32 i;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = 0;
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (ret < 0) {
		v4l2_warn(sd->v4l2_dev,
			  "sensor fmt invalid, set to default size\n");
		goto set_default;
	}

	for (i = 0; i < ARRAY_SIZE(in_fmts); i++)
		if (fmt.format.code == in_fmts[i].mbus_code &&
		    fmt.format.field == in_fmts[i].field)
			return &in_fmts[i];

	v4l2_err(sd->v4l2_dev, "remote sensor mbus code not supported\n");

set_default:
	return NULL;
}

static const struct
vip_output_fmt *find_output_fmt(struct rk_vip_stream *stream, u32 pixelfmt)
{
	const struct vip_output_fmt *fmt;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(out_fmts); i++) {
		fmt = &out_fmts[i];
		if (fmt->fourcc == pixelfmt)
			return fmt;
	}

	return NULL;
}

static void rk_vip_setup_buffers(struct rk_vip_stream *stream)
{
	struct rk_vip_device *dev = stream->vipdev;
	void __iomem *base = dev->base_addr;

	write_vip_reg(base, VIP_FRM0_ADDR_Y, stream->buffs[0]->buff_addr[RK_VIP_PLANE_Y]);
	write_vip_reg(base, VIP_FRM0_ADDR_UV, stream->buffs[0]->buff_addr[RK_VIP_PLANE_CBCR]);

	write_vip_reg(base, VIP_FRM1_ADDR_Y, stream->buffs[1]->buff_addr[RK_VIP_PLANE_Y]);
	write_vip_reg(base, VIP_FRM1_ADDR_UV, stream->buffs[1]->buff_addr[RK_VIP_PLANE_CBCR]);
}

static struct rk_vip_buffer *rk_vip_buffer_next(struct rk_vip_stream *stream)
{
	struct rk_vip_buffer *buff;

	if (list_empty(&stream->buf_head))
		return NULL;

	buff = list_first_entry(&stream->buf_head, struct rk_vip_buffer, queue);
	list_del(&buff->queue);

	return buff;
}

static void rk_vip_init_buffers(struct rk_vip_stream *stream)
{
	spin_lock(&stream->vbq_lock);

	stream->buffs[0] = rk_vip_buffer_next(stream);
	stream->buffs[1] = rk_vip_buffer_next(stream);

	spin_unlock(&stream->vbq_lock);

	rk_vip_setup_buffers(stream);
}

static void rk_vip_assign_new_buffer_pingpong(struct rk_vip_stream *stream)
{
	struct rk_vip_scratch_buffer *scratch_buf = &stream->scratch_buf;
	struct rk_vip_device *dev = stream->vipdev;
	struct rk_vip_buffer *buffer = NULL;
	void __iomem *base = dev->base_addr;
	u32 frm_addr_y, frm_addr_uv;

	/* Set up an empty buffer for the next frame */
	spin_lock(&stream->vbq_lock);

	buffer = rk_vip_buffer_next(stream);
	stream->buffs[stream->frame_phase] = buffer;

	spin_unlock(&stream->vbq_lock);

	frm_addr_y = stream->frame_phase ? VIP_FRM1_ADDR_Y : VIP_FRM0_ADDR_Y;
	frm_addr_uv = stream->frame_phase ? VIP_FRM1_ADDR_UV : VIP_FRM0_ADDR_UV;

	if (buffer) {
		write_vip_reg(base, frm_addr_y,
			      buffer->buff_addr[RK_VIP_PLANE_Y]);
		write_vip_reg(base, frm_addr_uv,
			      buffer->buff_addr[RK_VIP_PLANE_CBCR]);
	} else {
		write_vip_reg(base, frm_addr_y, scratch_buf->dma_addr);
		write_vip_reg(base, frm_addr_uv, scratch_buf->dma_addr);
	}
}

static void rk_vip_stream_stop(struct rk_vip_stream *stream)
{
	struct rk_vip_device *vip_dev = stream->vipdev;
	void __iomem *base = vip_dev->base_addr;
	u32 val;

	val = read_vip_reg(base, VIP_CTRL);
	write_vip_reg(base, VIP_CTRL, val & (~VIP_CTRL_ENABLE_CAPTURE));
	write_vip_reg(base, VIP_INTEN, 0x0);
	write_vip_reg(base, VIP_INTSTAT, 0x3ff);
	write_vip_reg(base, VIP_FRAME_STATUS, 0x0);

	stream->state = RK_VIP_STATE_READY;
}

static int rk_vip_queue_setup(struct vb2_queue *queue,
			      unsigned int *num_buffers,
			      unsigned int *num_planes,
			      unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct rk_vip_stream *stream = queue->drv_priv;
	const struct v4l2_plane_pix_format *plane_fmt;
	const struct v4l2_pix_format_mplane *pixm;

	pixm = &stream->pixm;

	if (*num_planes) {
		if (*num_planes != 1)
			return -EINVAL;

		if (sizes[0] < pixm->plane_fmt[0].sizeimage)
			return -EINVAL;
		return 0;
	}

	*num_planes = 1;

	plane_fmt = &pixm->plane_fmt[0];
	sizes[0] = plane_fmt->sizeimage;

	*num_buffers = VIP_REQ_BUFS_MIN;

	return 0;
}

static void rk_vip_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rk_vip_buffer *vipbuf = to_rk_vip_buffer(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct rk_vip_stream *stream = queue->drv_priv;
	struct v4l2_pix_format_mplane *pixm = &stream->pixm;
	unsigned long lock_flags = 0;
	int i;

	memset(vipbuf->buff_addr, 0, sizeof(vipbuf->buff_addr));

	vipbuf->buff_addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);

	for (i = 0; i < pixm->num_planes - 1; i++)
		vipbuf->buff_addr[i + 1] = vipbuf->buff_addr[i] +
			pixm->plane_fmt[i].bytesperline * pixm->height;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	list_add_tail(&vipbuf->queue, &stream->buf_head);
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
}

static int rk_vip_create_scratch_buf(struct rk_vip_stream *stream)
{
	struct rk_vip_scratch_buffer *scratch_buf = &stream->scratch_buf;
	struct rk_vip_device *dev = stream->vipdev;

	/* get a maximum plane size */
	scratch_buf->size = max(stream->pixm.plane_fmt[0].sizeimage,
				stream->pixm.plane_fmt[1].sizeimage);

	scratch_buf->vaddr = dma_alloc_coherent(dev->dev, scratch_buf->size,
						&scratch_buf->dma_addr,
						GFP_KERNEL);
	if (!scratch_buf->vaddr) {
		v4l2_err(&dev->v4l2_dev,
			 "Failed to allocate the memory for scratch buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static void rk_vip_destroy_scratch_buf(struct rk_vip_stream *stream)
{
	struct rk_vip_scratch_buffer *scratch_buf = &stream->scratch_buf;
	struct rk_vip_device *dev = stream->vipdev;

	dma_free_coherent(dev->dev, scratch_buf->size,
			  scratch_buf->vaddr, scratch_buf->dma_addr);
}

static void rk_vip_stop_streaming(struct vb2_queue *queue)
{
	struct rk_vip_stream *stream = queue->drv_priv;
	struct rk_vip_device *dev = stream->vipdev;
	struct rk_vip_buffer *buf;
	struct v4l2_subdev *sd;
	int ret;

	stream->stopping = true;
	ret = wait_event_timeout(stream->wq_stopped,
				 stream->state != RK_VIP_STATE_STREAMING,
				 msecs_to_jiffies(1000));
	if (!ret) {
		rk_vip_stream_stop(stream);
		stream->stopping = false;
	}
	pm_runtime_put(dev->dev);

	/* stop the sub device*/
	sd = dev->sensor.sd;
	v4l2_subdev_call(sd, video, s_stream, 0);
	v4l2_subdev_call(sd, core, s_power, 0);

	/* release buffers */
	if (stream->buffs[0]) {
		list_add_tail(&stream->buffs[0]->queue, &stream->buf_head);
		stream->buffs[0] = NULL;
	}
	if (stream->buffs[1]) {
		list_add_tail(&stream->buffs[1]->queue, &stream->buf_head);
		stream->buffs[1] = NULL;
	}

	while (!list_empty(&stream->buf_head)) {
		buf = rk_vip_buffer_next(stream);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	rk_vip_destroy_scratch_buf(stream);
}

static u32 rk_vip_determine_input_mode(struct rk_vip_device *dev)
{
	struct rk_vip_sensor_info *sensor_info = &dev->sensor;
	struct rk_vip_stream *stream = &dev->stream;
	v4l2_std_id std;
	u32 mode = VIP_FORMAT_INPUT_MODE_YUV;
	int ret;

	ret = v4l2_subdev_call(sensor_info->sd, video, querystd, &std);
	if (ret == 0) {
		/* retrieve std from sensor if exist */
		switch (std) {
		case 0:
			break;
		case V4L2_STD_NTSC:
			mode = VIP_FORMAT_INPUT_MODE_NTSC;
			break;
		case V4L2_STD_PAL:
			mode = VIP_FORMAT_INPUT_MODE_PAL;
			break;
		case V4L2_STD_ATSC:
			mode = VIP_FORMAT_INPUT_MODE_BT1120;
			break;
		default:
			v4l2_err(&dev->v4l2_dev,
				 "std: %lld is not supported", std);
		}
	} else {
		/* determine input mode by mbus_code (fmt_type) */
		switch (stream->vip_fmt_in->fmt_type) {
		case VIP_FMT_TYPE_YUV:
			mode = VIP_FORMAT_INPUT_MODE_YUV;
			break;
		case VIP_FMT_TYPE_RAW:
			mode = VIP_FORMAT_INPUT_MODE_RAW;
			break;
		}
	}

	return mode;
}

static inline u32 rk_vip_scl_ctl(struct rk_vip_stream *stream)
{
	u32 fmt_type = stream->vip_fmt_in->fmt_type;

	return (fmt_type == VIP_FMT_TYPE_YUV) ?
		VIP_SCL_CTRL_ENABLE_YUV_16BIT_BYPASS :
		VIP_SCL_CTRL_ENABLE_RAW_16BIT_BYPASS;
}

static int rk_vip_stream_start(struct rk_vip_stream *stream)
{
	u32 val, mbus_flags, href_pol, vsync_pol,
	    xfer_mode = 0, yc_swap = 0, skip_top = 0;
	struct rk_vip_device *dev = stream->vipdev;
	struct rk_vip_sensor_info *sensor_info;
	void __iomem *base = dev->base_addr;

	sensor_info = &dev->sensor;
	stream->frame_idx = 0;

	mbus_flags = sensor_info->mbus.flags;
	href_pol = (mbus_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) ?
			VIP_FORMAT_HSY_HIGH_ACTIVE : VIP_FORMAT_HSY_LOW_ACTIVE;
	vsync_pol = (mbus_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) ?
			VIP_FORMAT_VSY_HIGH_ACTIVE : VIP_FORMAT_VSY_LOW_ACTIVE;

	if (rk_vip_determine_input_mode(dev) == VIP_FORMAT_INPUT_MODE_BT1120) {
		if (stream->vip_fmt_in->field == V4L2_FIELD_NONE)
			xfer_mode = VIP_FORMAT_BT1120_TRANSMIT_PROGRESS;
		else
			xfer_mode = VIP_FORMAT_BT1120_TRANSMIT_INTERFACE;

		if (!VIP_FETCH_IS_Y_FIRST(stream->vip_fmt_in->dvp_fmt_val))
			yc_swap = VIP_FORMAT_BT1120_YC_SWAP;
	}

	val = vsync_pol | href_pol | rk_vip_determine_input_mode(dev) |
	      stream->vip_fmt_out->fmt_val | stream->vip_fmt_in->dvp_fmt_val |
	      xfer_mode | yc_swap;
	write_vip_reg(base, VIP_FOR, val);
	val = stream->pixm.width;
	if (stream->vip_fmt_in->fmt_type == VIP_FMT_TYPE_RAW)
		val = stream->pixm.width * 2;
	write_vip_reg(base, VIP_VIR_LINE_WIDTH, val);
	write_vip_reg(base, VIP_SET_SIZE,
		      stream->pixm.width | (stream->pixm.height << 16));

	v4l2_subdev_call(sensor_info->sd, sensor, g_skip_top_lines, &skip_top);

	write_vip_reg(base, VIP_CROP, skip_top << VIP_CROP_Y_SHIFT);
	write_vip_reg(base, VIP_FRAME_STATUS, VIP_FRAME_STAT_CLS);
	write_vip_reg(base, VIP_INTSTAT, VIP_INTSTAT_CLS);
	write_vip_reg(base, VIP_SCL_CTRL, rk_vip_scl_ctl(stream));

	rk_vip_init_buffers(stream);

	write_vip_reg(base, VIP_INTEN, VIP_INTEN_FRAME_END_EN |
				       VIP_INTEN_LINE_ERR_EN |
				       VIP_INTEN_PST_INF_FRAME_END_EN);

	write_vip_reg(base, VIP_CTRL, VIP_CTRL_AXI_BURST_16 |
				      VIP_CTRL_MODE_PINGPONG |
				      VIP_CTRL_ENABLE_CAPTURE);

	stream->state = RK_VIP_STATE_STREAMING;

	return 0;
}

static int rk_vip_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct rk_vip_stream *stream = queue->drv_priv;
	struct rk_vip_device *dev = stream->vipdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct v4l2_subdev *sd;
	int ret;

	if (WARN_ON(stream->state != RK_VIP_STATE_READY)) {
		ret = -EBUSY;
		v4l2_err(v4l2_dev, "stream in busy state\n");
		goto destroy_buf;
	}

	stream->vip_fmt_in = get_input_fmt(dev->sensor.sd);

	ret = rk_vip_create_scratch_buf(stream);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to create scratch_buf, %d\n", ret);
		goto destroy_buf;
	}

	ret = pm_runtime_get_sync(dev->dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to get runtime pm, %d\n", ret);
		goto destroy_scratch_buf;
	}

	/* start sub-devices */
	sd = dev->sensor.sd;
	ret = v4l2_subdev_call(sd, core, s_power, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD)
		goto runtime_put;
	ret = v4l2_subdev_call(sd, video, s_stream, 1);
	if (ret < 0)
		goto subdev_poweroff;

	ret = rk_vip_stream_start(stream);
	if (ret < 0)
		goto stop_stream;

	return 0;

stop_stream:
	rk_vip_stream_stop(stream);
subdev_poweroff:
	v4l2_subdev_call(sd, core, s_power, 0);
runtime_put:
	pm_runtime_put(dev->dev);
destroy_scratch_buf:
	rk_vip_destroy_scratch_buf(stream);
destroy_buf:
	while (!list_empty(&stream->buf_head)) {
		struct rk_vip_buffer *buf;

		buf = rk_vip_buffer_next(stream);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}

	return ret;
}

static struct vb2_ops rk_vip_vb2_ops = {
	.queue_setup = rk_vip_queue_setup,
	.buf_queue = rk_vip_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.stop_streaming = rk_vip_stop_streaming,
	.start_streaming = rk_vip_start_streaming,
};

static int rk_vip_init_vb2_queue(struct vb2_queue *q,
				 struct rk_vip_stream *stream)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE |
		  V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = stream;
	q->ops = &rk_vip_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct rk_vip_buffer);
	q->min_buffers_needed = VIP_REQ_BUFS_MIN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &stream->vlock;
	q->dev = stream->vipdev->dev;

	return vb2_queue_init(q);
}

static void rk_vip_set_fmt(struct rk_vip_stream *stream,
			   struct v4l2_pix_format_mplane *pixm,
			   bool try)
{
	struct rk_vip_device *dev = stream->vipdev;
	struct v4l2_subdev_format sd_fmt;
	const struct vip_output_fmt *fmt;
	struct v4l2_rect input_rect;
	u32 width, height;

	fmt = find_output_fmt(stream, pixm->pixelformat);
	if (!fmt)
		fmt = &out_fmts[0];

	if (!try) {
		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		sd_fmt.pad = 0;
		sd_fmt.format.width = pixm->width;
		sd_fmt.format.height = pixm->height;
		v4l2_subdev_call(dev->sensor.sd, pad, set_fmt, NULL, &sd_fmt);

		pixm->width = sd_fmt.format.width;
		pixm->height = sd_fmt.format.height;
	}

	input_rect.width = VIP_MAX_WIDTH;
	input_rect.height = VIP_MAX_HEIGHT;

	width = clamp_t(u32, pixm->width, VIP_MIN_WIDTH, input_rect.width);
	height = clamp_t(u32, pixm->height, VIP_MIN_HEIGHT, input_rect.height);

	v4l2_fill_pixfmt_mp(pixm, fmt->fourcc, width, height);

	pixm->quantization = V4L2_QUANTIZATION_DEFAULT;
	pixm->colorspace = V4L2_COLORSPACE_SRGB;

	pixm->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pixm->colorspace);
	pixm->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pixm->colorspace);

	pixm->field = V4L2_FIELD_NONE;

	if (!try) {
		stream->vip_fmt_out = fmt;
		stream->pixm = *pixm;
	}
}

void rk_vip_stream_init(struct rk_vip_device *dev)
{
	struct rk_vip_stream *stream = &dev->stream;
	struct v4l2_pix_format_mplane pixm;

	memset(stream, 0, sizeof(*stream));
	memset(&pixm, 0, sizeof(pixm));
	stream->vipdev = dev;

	INIT_LIST_HEAD(&stream->buf_head);
	spin_lock_init(&stream->vbq_lock);
	stream->state = RK_VIP_STATE_READY;
	init_waitqueue_head(&stream->wq_stopped);

	/* Set default format */
	pixm.pixelformat = V4L2_PIX_FMT_NV12;
	pixm.width = RK_VIP_DEFAULT_WIDTH;
	pixm.height = RK_VIP_DEFAULT_HEIGHT;
	rk_vip_set_fmt(stream, &pixm, false);

	stream->crop.left = 0;
	stream->crop.top = 0;
	stream->crop.width = 10;
	stream->crop.height = 10;
}

static const struct v4l2_file_operations rk_vip_fops = {
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int rk_vip_enum_input(struct file *file, void *priv,
			     struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(input->name, "Camera", sizeof(input->name));

	return 0;
}

static int rk_vip_try_fmt_vid_cap_mplane(struct file *file, void *fh,
					 struct v4l2_format *f)
{
	struct rk_vip_stream *stream = video_drvdata(file);

	rk_vip_set_fmt(stream, &f->fmt.pix_mp, true);

	return 0;
}

static int rk_vip_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	const struct vip_output_fmt *fmt = NULL;

	if (f->index >= ARRAY_SIZE(out_fmts))
		return -EINVAL;

	fmt = &out_fmts[f->index];
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int rk_vip_s_fmt_vid_cap_mplane(struct file *file,
				       void *priv, struct v4l2_format *f)
{
	struct rk_vip_stream *stream = video_drvdata(file);

	if (stream->state == RK_VIP_STATE_STREAMING)
		return -EBUSY;

	rk_vip_set_fmt(stream, &f->fmt.pix_mp, false);

	return 0;
}

static int rk_vip_g_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct rk_vip_stream *stream = video_drvdata(file);

	f->fmt.pix_mp = stream->pixm;

	return 0;
}

static int rk_vip_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct rk_vip_stream *stream = video_drvdata(file);
	struct device *dev = stream->vipdev->dev;

	strlcpy(cap->driver, dev->driver->name, sizeof(cap->driver));
	strlcpy(cap->card, dev->driver->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", dev_name(dev));

	return 0;
}

static int rk_vip_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *fsize)
{
	struct rk_vip_stream *stream = video_drvdata(file);
	struct rk_vip_device *dev = stream->vipdev;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	const struct vip_output_fmt *fmt;
	int ret;

	if (!dev->sensor.sd)
		return -EINVAL;

	fmt = find_output_fmt(stream, fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	fse.code = fmt->mbus;

	ret = v4l2_subdev_call(dev->sensor.sd, pad, enum_frame_size,
			       NULL, &fse);
	if (ret)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

static int rk_vip_enum_frameintervals(struct file *file, void *fh,
				      struct v4l2_frmivalenum *fival)
{
	struct rk_vip_stream *stream = video_drvdata(file);
	struct rk_vip_device *dev = stream->vipdev;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = fival->index,
		.width = fival->width,
		.height = fival->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	const struct vip_output_fmt *fmt;
	int ret;

	if (!dev->sensor.sd)
		return -EINVAL;

	fmt = find_output_fmt(stream, fival->pixel_format);
	if (!fmt)
		return -EINVAL;

	fie.code = fmt->mbus;

	ret = v4l2_subdev_call(dev->sensor.sd, pad, enum_frame_interval,
			       NULL, &fie);
	if (ret)
		return ret;

	fival->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return 0;
}

static int rk_vip_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int rk_vip_s_input(struct file *file, void *fh, unsigned int i)
{
	if (i)
		return -EINVAL;

	return 0;
}

static int rk_vip_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *p)
{
	struct rk_vip_stream *stream = video_drvdata(file);
	struct rk_vip_device *dev = stream->vipdev;

	return v4l2_g_parm_cap(video_devdata(file), dev->sensor.sd, p);
}

static int rk_vip_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *p)
{
	struct rk_vip_stream *stream = video_drvdata(file);
	struct rk_vip_device *dev = stream->vipdev;

	return v4l2_s_parm_cap(video_devdata(file), dev->sensor.sd, p);
}

static const struct v4l2_ioctl_ops rk_vip_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_enum_fmt_vid_cap = rk_vip_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane = rk_vip_try_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rk_vip_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rk_vip_g_fmt_vid_cap_mplane,
	.vidioc_querycap = rk_vip_querycap,
	.vidioc_enum_framesizes = rk_vip_enum_framesizes,
	.vidioc_enum_frameintervals = rk_vip_enum_frameintervals,

	.vidioc_enum_input = rk_vip_enum_input,
	.vidioc_g_input = rk_vip_g_input,
	.vidioc_s_input = rk_vip_s_input,

	.vidioc_g_parm = rk_vip_g_parm,
	.vidioc_s_parm = rk_vip_s_parm,
};

void rk_vip_unregister_stream_vdev(struct rk_vip_device *dev)
{
	struct rk_vip_stream *stream = &dev->stream;

	media_entity_cleanup(&stream->vdev.entity);
	video_unregister_device(&stream->vdev);
}

int rk_vip_register_stream_vdev(struct rk_vip_device *dev)
{
	struct rk_vip_stream *stream = &dev->stream;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct video_device *vdev = &stream->vdev;
	int ret;

	strlcpy(vdev->name, VIP_VIDEODEVICE_NAME, sizeof(vdev->name));
	mutex_init(&stream->vlock);

	vdev->ioctl_ops = &rk_vip_v4l2_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->fops = &rk_vip_fops;
	vdev->minor = -1;
	vdev->v4l2_dev = v4l2_dev;
	vdev->lock = &stream->vlock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			    V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, stream);
	vdev->vfl_dir = VFL_DIR_RX;
	stream->pad.flags = MEDIA_PAD_FL_SINK;

	rk_vip_init_vb2_queue(&stream->buf_queue, stream);

	vdev->queue = &stream->buf_queue;
	strscpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev,
			 "video_register_device failed with error %d\n", ret);
		return ret;
	}

	ret = media_entity_pads_init(&vdev->entity, 1, &stream->pad);
	if (ret < 0)
		goto unreg;

	return 0;
unreg:
	video_unregister_device(vdev);
	return ret;
}

static void rk_vip_vb_done(struct rk_vip_stream *stream,
			   struct vb2_v4l2_buffer *vb_done)
{
	vb2_set_plane_payload(&vb_done->vb2_buf, 0,
			      stream->pixm.plane_fmt[0].sizeimage);
	vb_done->vb2_buf.timestamp = ktime_get_ns();
	vb_done->sequence = stream->frame_idx;
	vb2_buffer_done(&vb_done->vb2_buf, VB2_BUF_STATE_DONE);
}

static void rk_vip_reset_stream(struct rk_vip_device *vip_dev)
{
	void __iomem *base = vip_dev->base_addr;
	u32 ctl = read_vip_reg(base, VIP_CTRL);

	write_vip_reg(base, VIP_CTRL, ctl & (~VIP_CTRL_ENABLE_CAPTURE));
	write_vip_reg(base, VIP_CTRL, ctl | VIP_CTRL_ENABLE_CAPTURE);
}

irqreturn_t rk_vip_irq_pingpong(int irq, void *ctx)
{
	struct device *dev = ctx;
	struct rk_vip_device *vip_dev = dev_get_drvdata(dev);
	struct rk_vip_stream *stream = &vip_dev->stream;
	void __iomem *base = vip_dev->base_addr;
	unsigned int intstat;

	u32 lastline, lastpix, ctl, vip_frmst;

	intstat = read_vip_reg(base, VIP_INTSTAT);
	vip_frmst = read_vip_reg(base, VIP_FRAME_STATUS);
	lastline = VIP_FETCH_Y_LAST_LINE(read_vip_reg(base, VIP_LAST_LINE));
	lastpix = read_vip_reg(base, VIP_LAST_PIX);
	ctl = read_vip_reg(base, VIP_CTRL);

	/* There are two irqs enabled:
	 *  - PST_INF_FRAME_END: vip FIFO is ready,
	 *    this is prior to FRAME_END
	 *  - FRAME_END: vip has saved frame to memory,
	 *    a frame ready
	 */

	if ((intstat & VIP_INTSTAT_PST_INF_FRAME_END)) {
		write_vip_reg(base, VIP_INTSTAT,
			      VIP_INTSTAT_PST_INF_FRAME_END_CLR);

		if (stream->stopping)
			/* To stop VIP ASAP, before FRAME_END irq */
			write_vip_reg(base, VIP_CTRL,
				      ctl & (~VIP_CTRL_ENABLE_CAPTURE));
	}

	if ((intstat & VIP_INTSTAT_PRE_INF_FRAME_END))
		write_vip_reg(base, VIP_INTSTAT, VIP_INTSTAT_PRE_INF_FRAME_END);

	if (intstat & (VIP_INTSTAT_LINE_ERR | VIP_INTSTAT_PIX_ERR)) {
		write_vip_reg(base, VIP_INTSTAT, VIP_INTSTAT_LINE_ERR |
						 VIP_INTSTAT_PIX_ERR);
		rk_vip_reset_stream(vip_dev);
	}

	if ((intstat & VIP_INTSTAT_FRAME_END)) {
		struct vb2_v4l2_buffer *vb_done = NULL;

		write_vip_reg(base, VIP_INTSTAT, VIP_INTSTAT_FRAME_END_CLR |
						 VIP_INTSTAT_LINE_END_CLR);

		if (stream->stopping) {
			rk_vip_stream_stop(stream);
			stream->stopping = false;
			wake_up(&stream->wq_stopped);
			return IRQ_HANDLED;
		}

		if (lastline != stream->pixm.height) {
			v4l2_err(&vip_dev->v4l2_dev,
				 "Bad frame, irq:0x%x frmst:0x%x size:%dx%d\n",
				 intstat, vip_frmst, lastline, lastpix);

			rk_vip_reset_stream(vip_dev);
		}

		if (vip_frmst & VIP_INTSTAT_F0_READY)
			stream->frame_phase = 0;
		else if (vip_frmst & VIP_INTSTAT_F1_READY)
			stream->frame_phase = 1;
		else
			return IRQ_HANDLED;

		if (stream->buffs[stream->frame_phase])
			vb_done = &stream->buffs[stream->frame_phase]->vb;

		rk_vip_assign_new_buffer_pingpong(stream);

		if (vb_done)
			rk_vip_vb_done(stream, vb_done);

		stream->frame_idx++;
	}

	return IRQ_HANDLED;
}
