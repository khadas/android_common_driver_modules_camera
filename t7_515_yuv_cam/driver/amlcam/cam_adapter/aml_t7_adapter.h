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

#ifndef __AML_T7_ADAPTER_H__
#define __AML_T7_ADAPTER_H__

#include <linux/amlogic/media/ge2d/ge2d.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>

#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/media-entity.h>
#include <linux/kfifo.h>
#include <linux/delay.h>

#include "aml_t7_video.h"

#define ADAP_DDR_BUFF_CNT 4
#define ADAP_DOL_BUFF_CNT 2
#define ADAP_ALIGN(data, val) ((data + val - 1) & (~(val - 1)))

enum {
	MODE_MIPI_RAW_SDR_DDR = 0,
	MODE_MIPI_YUV_SDR_DDR,
	MODE_MIPI_RGB_SDR_DDR,
	MODE_MIPI_YUV_FRAME_VC_DDR
};

enum {
	AML_ADAP_PAD_SINK = 0,
	AML_ADAP_PAD_SRC,
	AML_ADAP_PAD_SRC_1,
	AML_ADAP_PAD_SRC_2,
	AML_ADAP_PAD_SRC_3,
	AML_ADAP_PAD_MAX,
};

enum {
	AML_ADAP_STREAM_FIRST_VC0_0,
	AML_ADAP_STREAM_FIRST_VC0_1,
	AML_ADAP_STREAM_FIRST_VC0_MAX,
	AML_ADAP_STREAM_FIRST_VC1_0 = AML_ADAP_STREAM_FIRST_VC0_MAX,
	AML_ADAP_STREAM_FIRST_VC1_1,
	AML_ADAP_STREAM_MAX,
};

enum {
	AML_ADAP_MEM_PATH = 0,
	AML_ADAP_ISP_PATH
};

#define ADAP_YUV422_8BIT  0x1e
#define ADAP_YUV422_10BIT 0x1f
#define ADAP_RGB444       0x20
#define ADAP_RGB555       0x21
#define ADAP_RGB565       0x22
#define ADAP_RGB666       0x23
#define ADAP_RGB888       0x24
#define ADAP_RAW6         0x28
#define ADAP_RAW7         0x29
#define ADAP_RAW8         0x2a
#define ADAP_RAW10        0x2b
#define ADAP_RAW12        0x2c
#define ADAP_RAW14        0x2d

enum {
	ADAP_DOL_NONE = 0,
	ADAP_DOL_VC,
	ADAP_DOL_LINEINFO,
	ADAP_DOL_MAX
};

struct adap_regval {
	u32 reg;
	u32 val;
};

struct adap_exp_offset {
	int long_offset;
	int short_offset;
	int offset_x;
	int offset_y;
};

struct fe_param_t{
   int fe_sel;//add for sel 7 mipi_isp_top
   int fe_cfg_ddr_max_bytes_other;
   int fe_dec_ctrl0;
   int fe_dec_ctrl1;
   int fe_dec_ctrl2;
   int fe_dec_ctrl3;
   int fe_dec_ctrl4;

   int fe_work_mode;
   int fe_mem_x_start;
   int fe_mem_x_end;
   int fe_mem_y_start;
   int fe_mem_y_end;
   int fe_isp_x_start;
   int fe_isp_x_end;
   int fe_isp_y_start;
   int fe_isp_y_end;
   int fe_mem_ping_addr;
   int fe_mem_pong_addr;
   int fe_mem_other_addr;
   int fe_mem_line_stride;
   int fe_mem_line_minbyte;
   int fe_isp_line_stride;
   int fe_isp_line_minbyte;

   int fe_int_mask;
};

struct adapter_dev_param {
	int path;
	int mode;
	u32 isp_path_width;
	u32 isp_path_height;
	u32 mem_path_width;
	u32 mem_path_height;
	int format;
	int dol_type;
	struct fe_param_t fe_param;
	struct adap_exp_offset offset;
	struct aml_buffer mem_path_buf[3];
	struct aml_buffer isp_path_buf[3];
	struct aml_buffer *cur_mem_buf;
	struct aml_buffer *cur_isp_buf;
	struct list_head mem_path_list;
	struct list_head isp_path_list;
	struct spinlock ddr_lock;
};

struct adapter_dev_ops {
	int (*hw_init)(void *a_dev);
	void (*hw_reset)(void *a_dev);
	int (*hw_start)(void *a_dev);
	void (*hw_stop)(void *a_dev);
	u32 (*hw_get_interrupt_status)(void *a_dev);

	int (*hw_stream_set_fmt)(struct aml_video *video, struct aml_format *fmt);
	int (*hw_stream_cfg_buf)(void *a_dev, struct aml_buffer *buff, int path);
	void (*hw_stream_on)(void *a_dev);
	void (*hw_stream_off)(void *a_dev);
};

struct adapter_workqueue_t {
	struct work_struct work_obj;
	struct kfifo adap_fifo_out;
};

struct adapter_dev_t {
	u32 index;
	u32 version;
	char *bus_info;
	struct device *dev;
	struct platform_device *pdev;

	int fe_0_irq;
	int fe_1_irq;
	int fe_2_irq;
	int fe_3_irq;

	void __iomem *adap;
	void __iomem *isp_top;
	struct clk *adap_clk;

	struct v4l2_subdev sd;
	struct media_pad pads[AML_ADAP_PAD_MAX];
	struct v4l2_mbus_framefmt pfmt[AML_ADAP_PAD_MAX];
	unsigned int fmt_cnt;
	unsigned int mem_frm_cnt;
	unsigned int isp_frm_cnt;
	const struct aml_format *formats;
	struct v4l2_device *v4l2_dev;
	struct media_pipeline pipe;

	struct adapter_dev_param param;
	const struct adapter_dev_ops *ops;

	struct aml_video video[AML_ADAP_STREAM_MAX];
	int is_streaming;
	struct ge2d_context_s * context_ge2d;
	struct adapter_workqueue_t* adapter_wq;
};
struct adapter_task_t {
	struct adapter_dev_t* adapter_dev;
	struct aml_buffer* src_buffer;
	u32 frm_cnt;
	int path_select;
};

void adap_subdev_suspend(struct adapter_dev_t *adap_dev);
int adap_subdev_resume(struct adapter_dev_t *adap_dev);

int aml_adap_subdev_init(void *c_dev);
void aml_adap_subdev_deinit(void *c_dev);
int aml_adap_video_register(struct adapter_dev_t *adap_dev);
void aml_adap_video_unregister(struct adapter_dev_t *adap_dev);
int aml_adap_subdev_register(struct adapter_dev_t *adap_dev);
void aml_adap_subdev_unregister(struct adapter_dev_t *adap_dev);

extern const struct adapter_dev_ops adap_dev_hw_ops;

#endif /* __AML_P1_ADAPTER_H__ */
