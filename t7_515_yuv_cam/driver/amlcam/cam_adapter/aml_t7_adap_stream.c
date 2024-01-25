/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2020 Amlogic or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include <linux/fs.h>

#include "aml_t7_adapter.h"

#define AML_VIDEO_NAME "t7-video-%u-%u"

#define  adap_isr_printk(str,...)  printk("[adap-irq] %5d - " str, __LINE__, ##__VA_ARGS__)

static const struct aml_format adap_cap_support_format[] = {
	{0, 0, 0, V4L2_PIX_FMT_YUYV, 1, 16},
	{0, 0, 0, V4L2_PIX_FMT_YVYU, 1, 16},
	{0, 0, 0, V4L2_PIX_FMT_UYVY, 1, 16},
	{0, 0, 0, V4L2_PIX_FMT_VYUY, 1, 16},
	{0, 0, 0, V4L2_PIX_FMT_BGR24, 1, 24},
	{0, 0, 0, V4L2_PIX_FMT_RGB24, 1, 24},
	{0, 0, 0, V4L2_PIX_FMT_NV21, 1, 12},
	{0, 0, 0, V4L2_PIX_FMT_NV12, 1, 12}
};

#if defined(DUMP_GE2D_IN) || defined(DUMP_GE2D_OUT)
int write_data_to_buf(char *path, char *buf, int size)
{
	int ret = 0;
	struct file *fp = NULL;
	loff_t pos = 0;
	unsigned int r_size = 0;

	if (buf == NULL || size == 0) {
		pr_info("%s:Error input param\n", __func__);
		return -1;
	}

	fp = filp_open(path, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_info("read error.\n");
		return -1;
	}

	r_size = kernel_write(fp, buf, size, &pos);

	filp_close(fp, NULL);

	return ret;
}

MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);

#endif

static int v4l2_2_ge2d_fmt(int v4l2_format)
{
	int format = GE2D_FORMAT_S16_YUV422;

	switch (v4l2_format) {
	case V4L2_PIX_FMT_RGB565X:
		format = GE2D_FORMAT_S16_RGB_565;
		break;
	case V4L2_PIX_FMT_YUV444:
		format = GE2D_FORMAT_S24_YUV444;
		break;
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_YUYV:
		format = GE2D_FORMAT_S16_YUV422;
		break;
	case V4L2_PIX_FMT_BGR24:
		format = GE2D_FORMAT_S24_RGB;
		break;
	case V4L2_PIX_FMT_RGB24:
		format = GE2D_FORMAT_S24_BGR;
		break;
	case V4L2_PIX_FMT_NV12:
		format = GE2D_FORMAT_M24_NV12;
		break;
	case V4L2_PIX_FMT_NV21:
		format = GE2D_FORMAT_M24_NV21;
		break;
	default:
		pr_err("unsupport fmt 0x%x, def to GE2D_FORMAT_S16_YUV422", v4l2_format);
		break;
	}
	return format;
}

static void adap_do_ge2d(struct ge2d_context_s *_context,
	struct aml_buffer * src_buffer, struct aml_buffer * dst_buffer,
	int src_w, int src_h, int src_ge2d_fmt,
	int dst_w, int dst_h, int dst_ge2d_fmt)
{
	struct config_para_ex_s ge2d_config;
#ifdef DUMP_GE2D_IN
	if (src_buffer->vaddr[AML_PLANE_A])
	{
		static int in_num = 0;
		in_num++;
		char in_path[128];
		snprintf(in_path, 128, "/sdcard/DCIM/ge2d-in-%d.raw", in_num);
		write_data_to_buf(in_path, src_buffer->vaddr[AML_PLANE_A], src_buffer->bsize);
	}
#endif

	int endian = GE2D_LITTLE_ENDIAN;
	if (src_ge2d_fmt == dst_ge2d_fmt) // just for copy. do not assign endian.
		endian = GE2D_BIG_ENDIAN;

	memset(&ge2d_config, 0, sizeof(struct config_para_ex_s));
	ge2d_config.alu_const_color = 0;/*0x000000ff;*/
	ge2d_config.bitmask_en = 0;
	ge2d_config.src1_gb_alpha = 0;/*0xff;*/
	ge2d_config.dst_xy_swap = 0;
	ge2d_config.src_planes[0].addr = src_buffer->addr[AML_PLANE_A];
	ge2d_config.src_planes[0].w = src_w;
	ge2d_config.src_planes[0].h = src_h;

	ge2d_config.src_para.format = src_ge2d_fmt | endian;
	ge2d_config.src_key.key_enable = 0;
	ge2d_config.src_key.key_mask = 0;
	ge2d_config.src_key.key_mode = 0;
	ge2d_config.src_para.canvas_index = 0;
	ge2d_config.src_para.mem_type = CANVAS_ALLOC;
	ge2d_config.src_para.fill_color_en = 0;
	ge2d_config.src_para.fill_mode = 0;
	ge2d_config.src_para.x_rev = 0;
	ge2d_config.src_para.y_rev = 0;
	ge2d_config.src_para.color = 0xffffffff;
	ge2d_config.src_para.top = 0;
	ge2d_config.src_para.left = 0;
	ge2d_config.src_para.width = src_w;
	ge2d_config.src_para.height = src_h;
	ge2d_config.src2_para.mem_type = CANVAS_TYPE_INVALID;

	ge2d_config.dst_planes[0].addr = (unsigned long)(dst_buffer->addr[AML_PLANE_A]);
	ge2d_config.dst_planes[0].w = dst_w;
	ge2d_config.dst_planes[0].h = dst_h;

	if (dst_ge2d_fmt == GE2D_FORMAT_M24_NV21  || dst_ge2d_fmt == GE2D_FORMAT_M24_NV12) {
		ge2d_config.dst_planes[1].addr = (unsigned long)((dst_buffer->addr[AML_PLANE_A] & 0xffffffff) + dst_w * dst_h);
		ge2d_config.dst_planes[1].w = dst_w;
		ge2d_config.dst_planes[1].h = dst_h / 2;
	}

	ge2d_config.dst_para.mem_type = CANVAS_ALLOC;
	ge2d_config.dst_para.format = dst_ge2d_fmt | endian;
	ge2d_config.dst_para.fill_color_en = 0;
	ge2d_config.dst_para.fill_mode = 0;
	ge2d_config.dst_para.x_rev = 0;
	ge2d_config.dst_para.y_rev = 0;
	ge2d_config.dst_xy_swap = 0;
	ge2d_config.dst_para.color = 0;
	ge2d_config.dst_para.top = 0;
	ge2d_config.dst_para.left = 0;
	ge2d_config.dst_para.width = dst_w;
	ge2d_config.dst_para.height = dst_h;

	if (ge2d_context_config_ex(_context, &ge2d_config) == 0) {
		stretchblt_noalpha(_context, 0, 0, src_w,
			src_h,
			0, 0, dst_w,
			dst_h);
	}

#ifdef DUMP_GE2D_OUT
	if (dst_buffer->vaddr[AML_PLANE_A])
	{
		static int out_num = 0;
		out_num++;
		char out_path[128];
		snprintf(out_path, 128, "/sdcard/DCIM/ge2d-out-%d.raw", out_num);
		write_data_to_buf(out_path, dst_buffer->vaddr[AML_PLANE_A], dst_buffer->bsize);
	}
#endif
}

// /dev/videoX
// X is determined with adater idx and CAM_VIDEO_IDX_BEGIN_NUM (aml_t7_video.c)
// X = CAM_VIDEO_IDX_BEGIN_NUM + video_start_idx_of_adapter[adapter_idx] + id
static const int video_start_idx_of_adapter[] = {0,5};

static s64 endtime_ms = 0, begtime_ms = 0;

static int adap_cap_irq_handler(void *video, void * in_buff, u32 frm_cnt, u32 width, u32 height)
{
	struct aml_video *vd = video;
	struct aml_buffer *b_current_filled = vd->b_current;
	struct aml_buffer *b_next_to_fill = NULL;
	struct adapter_dev_t *adap_dev = vd->priv;
	struct aml_buffer * in_ge2d_buffer = in_buff;
	unsigned long flags;

	spin_lock_irqsave(&vd->buff_list_lock, flags);

	if (vd->status != AML_ON) {
		spin_unlock_irqrestore(&vd->buff_list_lock, flags);
		return 0;
	}

	b_next_to_fill = list_first_entry_or_null(&vd->head, struct aml_buffer, list);

	if (!b_next_to_fill)
		adap_isr_printk("no free buf. drop frame, video id %d", vd->id);
	else
	{
		if (!b_current_filled)
			adap_isr_printk("free buf. no filled buf.");
		else
		{
			int dst_ge2d_fmt = v4l2_2_ge2d_fmt(vd->afmt.fourcc);
			spin_unlock_irqrestore(&vd->buff_list_lock, flags);
			adap_do_ge2d(adap_dev->context_ge2d, in_ge2d_buffer, b_current_filled,
				width, height, GE2D_FORMAT_S16_YUV422,
				vd->afmt.width, vd->afmt.height, dst_ge2d_fmt);
			spin_lock_irqsave(&vd->buff_list_lock, flags);
			b_current_filled->vb.sequence = frm_cnt;
			b_current_filled->vb.vb2_buf.timestamp = ktime_get_ns();
			b_current_filled->vb.field = V4L2_FIELD_NONE;
			vb2_buffer_done(&b_current_filled->vb.vb2_buf, VB2_BUF_STATE_DONE);
		}
		vd->b_current = b_next_to_fill;
		if (!list_empty(&vd->head))
			list_del(&vd->b_current->list);
	}
	spin_unlock_irqrestore(&vd->buff_list_lock, flags);
	return 0;
}

static int adap_cap_set_format(void *video)
{
	u32 i = 0;
	int rtn = 0;
	struct aml_video *vd = video;
	struct aml_format *fmt = &vd->afmt;
	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	dev_info(vd->dev, "video adap stream set fmt\n");

	fmt->width = vd->f_current.fmt.pix.width;
	fmt->height = vd->f_current.fmt.pix.height;
	fmt->fourcc = vd->f_current.fmt.pix.pixelformat;
	for (i = 0; i < vd->fmt_cnt; i++) {
		if (fmt->fourcc == vd->format[i].fourcc) {
			fmt->bpp = vd->format[i].bpp;
			break;
		}
	}

	return rtn;
}

static int adap_cap_cfg_buffer(void *video, void *buff)
{
	int rtn = 0;
	unsigned long flags;
	struct aml_video *vd = video;
	struct aml_buffer *buffer = buff;
	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	spin_lock_irqsave(&vd->buff_list_lock, flags);

	if (vd->b_current) {
		list_add_tail(&buffer->list, &vd->head);
		spin_unlock_irqrestore(&vd->buff_list_lock, flags);
		return rtn;
	}
	// set vd->b_current.for the next time adap_cap_cfg_buffer is called.
	// vd->b_current is true. will list_add_tail.
	vd->b_current = buff;

	spin_unlock_irqrestore(&vd->buff_list_lock, flags);

	return rtn;
}

static void adap_cap_stream_on(void *video)
{
	struct aml_video *vd = video;
	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	vd->status = AML_ON;
	dev_err(vd->dev, "video adap stream on\n");
}

static void adap_cap_stream_off(void *video)
{
	struct aml_video *vd = video;
	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	vd->status = AML_OFF;
}

static void adap_vb2_discard_done(struct vb2_queue *q)
{
	struct vb2_buffer *vb;
	unsigned long flags;

	spin_lock_irqsave(&q->done_lock, flags);
	list_for_each_entry(vb, &q->done_list, done_entry)
		vb->state = VB2_BUF_STATE_ERROR;
	spin_unlock_irqrestore(&q->done_lock, flags);
}

static void adap_cap_flush_buffer(void *video)
{
	unsigned long flags;
	struct aml_video *vd = video;
	struct aml_buffer *buff;

	spin_lock_irqsave(&vd->buff_list_lock, flags);

	list_for_each_entry(buff, &vd->head, list) {
		if (buff->vb.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&buff->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&vd->head);

	if (vd->b_current && vd->b_current->vb.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(&vd->b_current->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	adap_vb2_discard_done(&vd->vb2_q);
	vd->b_current = NULL;

	spin_unlock_irqrestore(&vd->buff_list_lock, flags);
	mdelay(33);
}

static const struct aml_cap_ops adap_cap_ops = {
	.cap_irq_handler = adap_cap_irq_handler,
	.cap_set_format = adap_cap_set_format,
	.cap_cfg_buffer = adap_cap_cfg_buffer,
	.cap_stream_on = adap_cap_stream_on,
	.cap_stream_off = adap_cap_stream_off,
	.cap_flush_buffer = adap_cap_flush_buffer,
};

int aml_adap_video_register(struct adapter_dev_t *adap_dev)
{
	int id;
	int rtn = 0;
	struct aml_video *video;

	for (id = 0; id < AML_ADAP_STREAM_MAX; id++) {
		video = &adap_dev->video[id];
		video->id = id;
		// /dev/videoX
		// X = CAM_VIDEO_IDX_BEGIN_NUM + video_start_idx_of_adapter[adapter_idx] + id
		video->belong_cam_idx = video_start_idx_of_adapter[adap_dev->index] + video->id;
		snprintf(video->name, sizeof(video->name), AML_VIDEO_NAME, adap_dev->index, video->id);
		video->dev = adap_dev->dev;
		video->v4l2_dev = adap_dev->v4l2_dev;
		video->bus_info = adap_dev->bus_info;
		video->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		video->ops = &adap_cap_ops;
		video->format = adap_cap_support_format;
		video->fmt_cnt = ARRAY_SIZE(adap_cap_support_format);
		video->pipe = &adap_dev->pipe;
		video->status = AML_OFF;
		video->priv = adap_dev;
		video->frm_cnt = 0;

		video->pad.flags = MEDIA_PAD_FL_SINK;

		INIT_LIST_HEAD(&video->head);
		spin_lock_init(&video->buff_list_lock);

		rtn = aml_video_register(video);
		if (rtn) {
			dev_err(video->dev, "Failed to register stream-%d: %d\n", id, rtn);
			break;
		}
	}

	return rtn;
}

void aml_adap_video_unregister(struct adapter_dev_t *adap_dev)
{
	int i;
	struct aml_video *video;

	for (i = 0; i < AML_ADAP_STREAM_MAX; i++) {
		video = &adap_dev->video[i];
		aml_video_unregister(video);
	}
}
