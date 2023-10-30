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

#include "aml_t7_adapter.h"

#define AML_VIDEO_NAME "t7-video-%u-%u"

#define  adap_isr_printk(str,...)  printk("[adap-irq] %5d - " str, __LINE__, ##__VA_ARGS__)

static const struct aml_format adap_cap_support_format[] = {
	{0, 0, MEDIA_BUS_FMT_VYUY8_2X8, V4L2_PIX_FMT_VYUY, 1, 16},
	{0, 0, MEDIA_BUS_FMT_UYVY8_2X8, V4L2_PIX_FMT_UYVY, 1, 16},
	{0, 0, MEDIA_BUS_FMT_YVYU8_2X8, V4L2_PIX_FMT_YVYU, 1, 16},
	{0, 0, MEDIA_BUS_FMT_YUYV8_2X8, V4L2_PIX_FMT_YUYV, 1, 16}
};

// /dev/videoX
// X is determined with adater idx and CAM_VIDEO_IDX_BEGIN_NUM (aml_t7_video.c)
// X = CAM_VIDEO_IDX_BEGIN_NUM + video_start_idx_of_adapter[adapter_idx] + id
static const int video_start_idx_of_adapter[] = {0,0};

static s64 endtime_ms = 0, begtime_ms = 0;

static int adap_cap_irq_handler(void *video, int status)
{
	unsigned long flags;
	struct aml_video *vd = video;

	struct aml_buffer *b_current_filled = vd->b_current;
	struct aml_buffer *b_next_to_fill = NULL;

	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	spin_lock_irqsave(&vd->buff_list_lock, flags);

	if (vd->status != AML_ON) {
		spin_unlock_irqrestore(&vd->buff_list_lock, flags);
		adap_isr_printk("err not stream on\n");
		return 0;
	}

	// step 1: get from free list. next buf to fill..
	b_next_to_fill = list_first_entry_or_null(&vd->head, struct aml_buffer, list);

	// step 2:report buf done &  set new hw addr to fill.
	if (b_next_to_fill) {
		if (b_current_filled) {
			vd->frm_cnt++;
			if ( (vd->frm_cnt % 100) == 0 ) {
				endtime_ms = ktime_to_ms( ktime_get() );
				if (endtime_ms > begtime_ms) {
					adap_isr_printk("adap fps - 100 frames %ld ms\n", (endtime_ms - begtime_ms) );
				}
				begtime_ms = endtime_ms;
			}
			b_current_filled->vb.sequence = vd->frm_cnt;
			b_current_filled->vb.vb2_buf.timestamp = ktime_get_ns();
			b_current_filled->vb.field = V4L2_FIELD_NONE;
			vb2_buffer_done(&b_current_filled->vb.vb2_buf, VB2_BUF_STATE_DONE);
		} else {
			adap_isr_printk("free buf. no filled buf.\n");
		}
		vd->b_current = b_next_to_fill;
		ops->hw_stream_cfg_buf(vd, vd->b_current);
		list_del(&vd->b_current->list);

	} else {
		adap_isr_printk("no free buf. drop frame, video id %d\n", vd->id);
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

	dev_err(vd->dev, "video adap stream set fmt\n");

	if (ops && ops->hw_stream_set_fmt) {
		rtn = ops->hw_stream_set_fmt(video, fmt);
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
	if (ops && ops->hw_stream_cfg_buf) {
		rtn = ops->hw_stream_cfg_buf(video, buff);
		vd->b_current = buff;
	}

	spin_unlock_irqrestore(&vd->buff_list_lock, flags);

	return rtn;
}

static void adap_cap_stream_on(void *video)
{
	struct aml_video *vd = video;
	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	if (ops && ops->hw_stream_on)
		ops->hw_stream_on(video);

	vd->status = AML_ON;
}

static void adap_cap_stream_off(void *video)
{
	struct aml_video *vd = video;
	struct adapter_dev_t *adap_dev = vd->priv;
	const struct adapter_dev_ops *ops = adap_dev->ops;

	if (ops && ops->hw_stream_off)
		ops->hw_stream_off(video);

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
