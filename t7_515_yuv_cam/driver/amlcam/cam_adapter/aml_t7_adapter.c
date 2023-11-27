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
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt)  "aml-adap:%s:%d: " fmt, __func__, __LINE__

#include <linux/version.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/delay.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#include <linux/dma-map-ops.h>
#else
#include <linux/dma-contiguous.h>
#endif
#include <linux/cma.h>

#include "aml_t7_cam.h"
#include "hw/aml_t7_adapter_hw.h"

#define AML_ADAPTER_NAME "t7-adapter-%u"

#ifdef DEBUG_TEST_MIPI_RESET
volatile int debug_test_mipi_reset = 1;
#endif

#define ISP_SIZE_ALIGN(data, aln)   (((data) + (aln) -1) & (~((aln) - 1)))

#define  adap_isr_printk(str,...)  printk("[adap-irq] %5d - " str, __LINE__, ##__VA_ARGS__)

#define MAX_ADAP_FIFO_LEN 8

static const struct aml_format adap_support_formats[] = {
	{0, 0, MEDIA_BUS_FMT_YUYV8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_UYVY8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_YVYU8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_VYUY8_2X8, 0, 1, 16},
	{0, 0, MEDIA_BUS_FMT_SBGGR8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SGBRG8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SGRBG8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SRGGB8_1X8, 0, 1, 8},
	{0, 0, MEDIA_BUS_FMT_SBGGR10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SGBRG10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SGRBG10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SRGGB10_1X10, 0, 1, 10},
	{0, 0, MEDIA_BUS_FMT_SBGGR12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SGBRG12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SGRBG12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SRGGB12_1X12, 0, 1, 12},
	{0, 0, MEDIA_BUS_FMT_SBGGR14_1X14, 0, 1, 14},
	{0, 0, MEDIA_BUS_FMT_SGBRG14_1X14, 0, 1, 14},
	{0, 0, MEDIA_BUS_FMT_SGRBG14_1X14, 0, 1, 14},
	{0, 0, MEDIA_BUS_FMT_SRGGB14_1X14, 0, 1, 14},
};

volatile uint32_t adap_debug_irq_in_count = 0;
volatile uint32_t adap_debug_irq_out_count = 0;

static void __iomem *adap_ioremap_resource(void *a_dev, char *name)
{
	void __iomem *reg;
	struct resource *res;
	resource_size_t size;
	struct adapter_dev_t *adap_dev;
	struct device *dev;
	struct platform_device *pdev;

	if (!a_dev || !name) {
		pr_err("Error input param\n");
		return NULL;
	}

	adap_dev = a_dev;
	dev = adap_dev->dev;
	pdev = adap_dev->pdev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(dev, "Error %s res\n", name);
		return NULL;
	}

	size = resource_size(res);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	reg = devm_ioremap(dev, res->start, size);
#else
	reg = devm_ioremap_nocache(dev, res->start, size);
#endif
	if (!reg) {
		dev_err(dev, "Failed to ioremap %s %pR\n", name, res);
		reg = IOMEM_ERR_PTR(-ENOMEM);
	}

	return reg;
}

static void adap_iounmap_resource(void *a_dev)
{
	struct device *dev;
	struct adapter_dev_t *adap_dev;

	adap_dev = a_dev;
	dev = adap_dev->dev;

	if (adap_dev->adap) {
		devm_iounmap(dev, adap_dev->adap);
		adap_dev->adap = NULL;
	}
}


static int adap_of_parse_dev(struct adapter_dev_t *adap_dev)
{
	int rtn = 0;

	adap_dev->adap = adap_ioremap_resource(adap_dev, "adapter");
	if (!adap_dev->adap) {
		dev_err(adap_dev->dev, "Failed to get adapter reg\n");
		rtn = -EINVAL;
		goto error_rtn;
	}
	adap_dev->isp_top = adap_dev->adap + ADAP_TOP_TO_BASE_OFFSET;

	adap_dev->fe_0_irq = of_irq_get_byname(adap_dev->dev->of_node, "adap_irq_fe0");
	adap_dev->fe_2_irq = of_irq_get_byname(adap_dev->dev->of_node, "adap_irq_fe2");
	if (!adap_dev->fe_0_irq || !adap_dev->fe_2_irq ) {
		dev_err(adap_dev->dev, "Error to parse irq\n");
		rtn = -EINVAL;
		goto error_rtn;
	}

	of_reserved_mem_device_init(adap_dev->dev);

	adap_dev->adap_clk = devm_clk_get(adap_dev->dev, "mipi_isp_clk");
	if (IS_ERR(adap_dev->adap_clk)) {
		devm_iounmap(adap_dev->dev, adap_dev->adap);
		dev_err(adap_dev->dev, "Error to get adap_clk\n");
		return PTR_ERR(adap_dev->adap_clk);
	}

error_rtn:
	return rtn;
}

static void adap_subdev_workqueue( struct work_struct *work )
{
	struct adapter_workqueue_t* p_workqueue = container_of(work, struct adapter_workqueue_t, work_obj);
	struct adapter_dev_t *adap_dev = NULL;
	struct adapter_task_t task;
	struct aml_video *video = NULL;
	struct aml_buffer *b_raw_buffer = NULL;
	int ret;
	int id;
	u32 frm_cnt;
	u32 src_w;
	u32 src_h;
	int path_select;
	unsigned long flags;

	if (kfifo_len(&p_workqueue->adap_fifo_out) > 0) {
		ret = kfifo_out(&p_workqueue->adap_fifo_out, &task, sizeof(struct adapter_task_t));
		if (ret == sizeof(struct adapter_task_t)) {
			b_raw_buffer = task.src_buffer;
			frm_cnt = task.frm_cnt;
			adap_dev = task.adapter_dev;
			path_select = task.path_select;

			if (path_select == AML_ADAP_MEM_PATH) {
				src_w = adap_dev->param.mem_path_width;
				src_h = adap_dev->param.mem_path_height;
				for (id = AML_ADAP_STREAM_FIRST_VC0_0; id < AML_ADAP_STREAM_FIRST_VC0_MAX; id++)
				{
					video = &adap_dev->video[id];
					if (video->ops->cap_irq_handler) {
						video->ops->cap_irq_handler(video, b_raw_buffer, frm_cnt, src_w, src_h);
					} else {
						adap_isr_printk("error no no video cap irq handler \n");
					}
				}
				spin_lock_irqsave(&adap_dev->param.ddr_lock, flags);
				list_add_tail(&b_raw_buffer->list, &adap_dev->param.mem_path_list);
				spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);
			} else {
				src_w = adap_dev->param.isp_path_width;
				src_h = adap_dev->param.isp_path_height;
				for (id = AML_ADAP_STREAM_FIRST_VC1_0; id < AML_ADAP_STREAM_MAX; id++)
				{
					video = &adap_dev->video[id];
					if (video->ops->cap_irq_handler) {
						video->ops->cap_irq_handler(video, b_raw_buffer, frm_cnt, src_w, src_h);
					} else {
						adap_isr_printk("error no no video cap irq handler \n");
					}
				}
				spin_lock_irqsave(&adap_dev->param.ddr_lock, flags);
				list_add_tail(&b_raw_buffer->list, &adap_dev->param.isp_path_list);
				spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);
			}
		} else {
		    pr_err("adap kfifo out fail");
		}
	}
}


static s64 endtime_ms_mem = 0, begtime_ms_mem = 0;
static s64 endtime_ms_isp = 0, begtime_ms_isp = 0;

static int adap_irq_handler_locked(struct adapter_dev_t *adap_dev, int path)
{
	int i;
	struct aml_video *video;
	struct aml_buffer *b_current_filled = NULL;
	struct aml_buffer *b_next_to_fill = NULL;
	struct adapter_dev_param *param = &adap_dev->param;

	if (path == AML_ADAP_MEM_PATH) {
		b_current_filled = adap_dev->param.cur_mem_buf;
		b_next_to_fill = list_first_entry_or_null(&param->mem_path_list, struct aml_buffer, list);
	} else {
		b_current_filled = adap_dev->param.cur_isp_buf;
		b_next_to_fill = list_first_entry_or_null(&param->isp_path_list, struct aml_buffer, list);
	}

	if (!b_next_to_fill) {
		return 0;
	}

	if (!b_current_filled) {
		adap_isr_printk("free buf. no filled buf.\n");
	} else {
		struct adapter_task_t __task;
		if (path == AML_ADAP_MEM_PATH) {
			adap_dev->mem_frm_cnt++;
			if ( (adap_dev->mem_frm_cnt % 100) == 0 ) {
				endtime_ms_mem = ktime_to_ms( ktime_get() );
				if (endtime_ms_mem > begtime_ms_mem) {
					adap_isr_printk("adap fps mem - 100 frames %ld ms\n", (endtime_ms_mem - begtime_ms_mem) );
				}
				begtime_ms_mem = endtime_ms_mem;
			}
			__task.frm_cnt = adap_dev->mem_frm_cnt;
		} else {
			adap_dev->isp_frm_cnt++;
			if ( (adap_dev->isp_frm_cnt % 100) == 0 ) {
				endtime_ms_isp = ktime_to_ms( ktime_get() );
				if (endtime_ms_isp > begtime_ms_isp) {
					adap_isr_printk("adap fps isp - 100 frames %ld ms\n", (endtime_ms_isp - begtime_ms_isp) );
				}
				begtime_ms_isp = endtime_ms_isp;
			}
			__task.frm_cnt = adap_dev->isp_frm_cnt;
		}
		__task.src_buffer = b_current_filled;
		__task.adapter_dev = adap_dev;
		__task.path_select = path;
		if (!kfifo_is_full(&adap_dev->adapter_wq->adap_fifo_out)) {
			kfifo_in(&adap_dev->adapter_wq->adap_fifo_out, &__task, sizeof(struct adapter_task_t));
			schedule_work(&adap_dev->adapter_wq->work_obj);
		} else {
			adap_isr_printk("fifo is full");
		}
	}
	if (path == AML_ADAP_MEM_PATH) {
		param->cur_mem_buf = b_next_to_fill;
		adap_dev->ops->hw_stream_cfg_buf(adap_dev, param->cur_mem_buf, AML_ADAP_MEM_PATH);
		if (!list_empty(&param->mem_path_list)) {
			list_del(&param->cur_mem_buf->list);
		}
	} else {
		param->cur_isp_buf = b_next_to_fill;
		adap_dev->ops->hw_stream_cfg_buf(adap_dev, param->cur_isp_buf, AML_ADAP_ISP_PATH);
		if (!list_empty(&param->isp_path_list)) {
			list_del(&param->cur_isp_buf->list);
		}
	}
	return 0;
}
static irqreturn_t adap_interrupt_handler(int irq, void *dev)
{
	struct aml_video *video;
	struct adapter_dev_t *adap_dev = dev;
	unsigned long flags;

	spin_lock_irqsave(&adap_dev->param.ddr_lock, flags);

	u32 status = 0;
	adap_debug_irq_in_count++;

#ifdef DEBUG_TEST_MIPI_RESET
	if (debug_test_mipi_reset) {
		adap_isr_printk("debug_test_mipi_reset 1\n");
		spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);
		return IRQ_HANDLED;
	}
#endif

	status = adap_dev->ops->hw_get_interrupt_status(adap_dev);

	if (0 == adap_dev->is_streaming) {
		adap_isr_printk("do handle irq before stream on\n");
		adap_debug_irq_out_count++;
		spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);
		return IRQ_HANDLED;
	}

	if (0x0 == (status & 0x240000)) {
		//adap_isr_printk("error no wr_done sticky\n");
		adap_debug_irq_out_count++;
		spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);
		return IRQ_HANDLED;
	}
	do {
		if (status & (0x1 << 18))
			adap_irq_handler_locked(adap_dev, AML_ADAP_MEM_PATH);
		if (status & (0x1 << 21))
			adap_irq_handler_locked(adap_dev, AML_ADAP_ISP_PATH);
		status = adap_dev->ops->hw_get_interrupt_status(adap_dev);
	} while (0x0 != (status & 0x240000)) ;

	adap_debug_irq_out_count++;
	spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);

	return IRQ_HANDLED;
}

static int adap_subdev_hw_stream_on(struct adapter_dev_t *adap_dev)
{
	if (adap_dev->ops->hw_stream_on)
		adap_dev->ops->hw_stream_on(adap_dev);
	return 0;
}

static int adap_subdev_hw_stream_off(struct adapter_dev_t *adap_dev)
{
	if (adap_dev->ops->hw_stream_off)
		adap_dev->ops->hw_stream_off(adap_dev);
	return 0;
}

static int adap_subdev_hw_start(struct adapter_dev_t *adap_dev)
{
	if (adap_dev->ops->hw_start)
		adap_dev->ops->hw_start(adap_dev);

#ifdef DEBUG_TEST_MIPI_RESET
	debug_test_mipi_reset = 1;
#endif
	return 0;
}

static void adap_subdev_hw_stop(struct adapter_dev_t *adap_dev)
{
	if (adap_dev->ops->hw_stop)
		adap_dev->ops->hw_stop(adap_dev);
}

static int adap_subdev_cma_alloc(struct platform_device *pdev, dma_addr_t *paddr, unsigned long size)
{
	struct page *cma_pages = NULL;

	size = ISP_SIZE_ALIGN(size, 1 << 12);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct device *dev = &(pdev->dev);
	struct cma *cma_area;
	if (dev && dev->cma_area)
		cma_area = dev->cma_area;
	else
		cma_area = dma_contiguous_default_area;
	cma_pages = cma_alloc(cma_area, size >> PAGE_SHIFT, 0, 0);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
	cma_pages = dma_alloc_from_contiguous(
		&(pdev->dev), size >> PAGE_SHIFT, 0, false);
#else
	cma_pages = dma_alloc_from_contiguous(
		&(pdev->dev), size >> PAGE_SHIFT, 0);
#endif
	if (cma_pages) {
		*paddr = page_to_phys(cma_pages);
	} else {
		pr_debug("Failed alloc cma pages.\n");
		return -1;
	}
	return 0;
}

static void adap_subdev_cma_free(struct platform_device *pdev, void *page, unsigned long size)
{
	struct page *cma_pages = NULL;
	bool rc = false;

	size = ISP_SIZE_ALIGN(size, 1 << 12);

	cma_pages = page;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct cma *cma_area;
	struct device *dev = &(pdev->dev);
	if (dev && dev->cma_area)
		cma_area = dev->cma_area;
	else
		cma_area = dma_contiguous_default_area;
	rc = cma_release(cma_area, cma_pages, size >> PAGE_SHIFT);
#else
	rc = dma_release_from_contiguous(&(pdev->dev), cma_pages, size >> PAGE_SHIFT);
#endif
	if (rc == false) {
		pr_debug("Failed to release cma buffer\n");
		return;
	}
}

static int adap_subdev_alloc_raw_buffs(struct adapter_dev_t *a_dev)
{
	int i = 0;
	int rtn = 0;
	unsigned long flags;
	dma_addr_t paddr = 0x0000;
	unsigned int bsize = 0;
	unsigned int fsize = 0;
	unsigned int fcnt = 0;
	struct adapter_dev_param *param = &a_dev->param;

	//mem path
	fsize = param->mem_path_width * param->mem_path_height * 16 / 8;

	fsize = ISP_SIZE_ALIGN(fsize, 1 << 12);

	fcnt = sizeof(param->mem_path_buf) /sizeof(param->mem_path_buf[0]);

	bsize = fcnt * fsize + fsize;

	rtn = adap_subdev_cma_alloc(a_dev->pdev, &paddr, bsize);
	if (rtn != 0) {
		pr_err("Failed to alloc mem raw buff\n");
		return -1;
	}

	spin_lock_irqsave(&param->ddr_lock, flags);

	for (i = 0; i < fcnt; i++) {
		param->mem_path_buf[i].bsize = fsize;
		param->mem_path_buf[i].addr[AML_PLANE_A] = paddr + i * fsize;

		list_add_tail(&param->mem_path_buf[i].list, &param->mem_path_list);
	}

	spin_unlock_irqrestore(&param->ddr_lock, flags);

	//isp path

	fsize = param->isp_path_width * param->isp_path_height * 16 / 8;

	fsize = ISP_SIZE_ALIGN(fsize, 1 << 12);

	fcnt = sizeof(param->isp_path_buf) /sizeof(param->isp_path_buf[0]);

	bsize = fcnt * fsize + fsize;

	rtn = adap_subdev_cma_alloc(a_dev->pdev, &paddr, bsize);
	if (rtn != 0) {
		pr_err("Failed to alloc isp raw buff\n");
		return -1;
	}

	spin_lock_irqsave(&param->ddr_lock, flags);

	for (i = 0; i < fcnt; i++) {
		param->isp_path_buf[i].bsize = fsize;
		param->isp_path_buf[i].addr[AML_PLANE_A] = paddr + i * fsize;

		list_add_tail(&param->isp_path_buf[i].list, &param->isp_path_list);
	}

	spin_unlock_irqrestore(&param->ddr_lock, flags);


	return rtn;
}

static void adap_subdev_free_raw_buffs(struct adapter_dev_t *a_dev)
{
	int i = 0;
	unsigned int fcnt = 0;
	struct adapter_dev_param *param = &a_dev->param;
	dma_addr_t paddr = 0x0000;
	void *page = NULL;
	unsigned long flags;

	spin_lock_irqsave(&param->ddr_lock, flags);
	INIT_LIST_HEAD(&param->mem_path_list);
	spin_unlock_irqrestore(&param->ddr_lock, flags);

	paddr = a_dev->param.mem_path_buf[0].addr[AML_PLANE_A];
	page = phys_to_page(paddr);

	fcnt = sizeof(param->mem_path_buf) /sizeof(param->mem_path_buf[0]);

	if (paddr)
		adap_subdev_cma_free(a_dev->pdev, page, a_dev->param.mem_path_buf[0].bsize * (fcnt + 1));

	for (i = 0; i < fcnt; i++) {
		param->mem_path_buf[i].addr[AML_PLANE_A] = 0;
	}

	spin_lock_irqsave(&param->ddr_lock, flags);
	INIT_LIST_HEAD(&param->isp_path_list);
	spin_unlock_irqrestore(&param->ddr_lock, flags);

	paddr = a_dev->param.isp_path_buf[0].addr[AML_PLANE_A];
	page = phys_to_page(paddr);

	fcnt = sizeof(param->isp_path_buf) /sizeof(param->isp_path_buf[0]);

	if (paddr)
		adap_subdev_cma_free(a_dev->pdev, page, a_dev->param.isp_path_buf[0].bsize * (fcnt + 1));

	for (i = 0; i < fcnt; i++) {
		param->isp_path_buf[i].addr[AML_PLANE_A] = 0;
	}

}


static int adap_subdev_cfg_buf_once(struct adapter_dev_t *a_dev)
{
	unsigned long flags;
	struct aml_video video;
	struct adapter_dev_param *param = &a_dev->param;

	spin_lock_irqsave(&param->ddr_lock, flags);

	param->cur_mem_buf = list_first_entry_or_null(&param->mem_path_list, struct aml_buffer, list);

	list_del(&param->cur_mem_buf->list);

	a_dev->ops->hw_stream_cfg_buf(a_dev, param->cur_mem_buf, AML_ADAP_MEM_PATH);

	param->cur_isp_buf = list_first_entry_or_null(&param->isp_path_list, struct aml_buffer, list);

	list_del(&param->cur_isp_buf->list);

	a_dev->ops->hw_stream_cfg_buf(a_dev, param->cur_isp_buf, AML_ADAP_ISP_PATH);

	spin_unlock_irqrestore(&param->ddr_lock, flags);

	return 0;

}
static int adap_subdev_set_stream(struct v4l2_subdev *sd, int enable)
{
	int rtn = 0;
	unsigned long flags;
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	if (enable) {
		adap_dev->is_streaming = 1;
		adap_dev->context_ge2d = create_ge2d_work_queue();
		adap_subdev_alloc_raw_buffs(adap_dev);
		adap_subdev_cfg_buf_once(adap_dev);
		adap_subdev_hw_stream_on(adap_dev);
		adap_subdev_hw_start(adap_dev);
		mdelay(66);
	} else {
		adap_subdev_hw_stop(adap_dev);
		adap_subdev_hw_stream_off(adap_dev);
		mdelay(66);
		adap_subdev_free_raw_buffs(adap_dev);
		spin_lock_irqsave(&adap_dev->param.ddr_lock, flags);
		kfifo_reset(&adap_dev->adapter_wq->adap_fifo_out);
		spin_unlock_irqrestore(&adap_dev->param.ddr_lock, flags);
		destroy_ge2d_work_queue(adap_dev->context_ge2d);
		adap_dev->is_streaming = 0;
	}
	dev_err(adap_dev->dev, "adap_subdev_set_stream done");
	return rtn;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int adap_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int adap_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)

#endif
{
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	if (code->pad == AML_ADAP_PAD_SINK) {
		if (code->index >= adap_dev->fmt_cnt)
			return -EINVAL;

		code->code = adap_dev->formats[code->index].code;
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = adap_dev->formats[0].code;
	}

	return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static struct v4l2_mbus_framefmt *
adap_subdev_get_padfmt(struct adapter_dev_t *adap_dev,
				struct v4l2_subdev_state *cfg,
				struct v4l2_subdev_format *fmt)
#else
static struct v4l2_mbus_framefmt *
adap_subdev_get_padfmt(struct adapter_dev_t *adap_dev,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)

#endif
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&adap_dev->sd, cfg, fmt->pad);

	return &adap_dev->pfmt[fmt->pad];
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int adap_subdev_get_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_format *fmt)
#else
static int adap_subdev_get_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
#endif
{
	struct v4l2_mbus_framefmt *format;
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	format = adap_subdev_get_padfmt(adap_dev, cfg, fmt);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int adap_subdev_convert_fmt(struct adapter_dev_t *adap_dev,
				struct v4l2_mbus_framefmt *format)
{
	int i = 0;
	int fmt = -1;
	const struct aml_format *am_fmt;

	for (i = 0; i < adap_dev->fmt_cnt; i++) {
		if (adap_dev->formats[i].code == format->code) {
			am_fmt = &adap_dev->formats[i];
			break;
		}
	}

	if (i == adap_dev->fmt_cnt)
		return fmt;

	switch (am_fmt->bpp) {
	case 8:
		fmt = ADAP_RAW8;
	break;
	case 10:
		fmt = ADAP_RAW10;
	break;
	case 12:
		fmt = ADAP_RAW12;
	break;
	case 14:
		fmt = ADAP_RAW14;
	break;
	case 16:
		fmt = ADAP_YUV422_8BIT;
	break;
	default:
		dev_err(adap_dev->dev, "Error support format\n");
	break;
	}

	return fmt;
}

static int adap_subdev_hw_init(struct adapter_dev_t *adap_dev)
{
	int rtn = 0;
	int adap_fmt = 0;
	struct adapter_dev_param *param = &adap_dev->param;

	if (param->format == ADAP_YUV422_8BIT ) {
		param->mode = MODE_MIPI_YUV_FRAME_VC_DDR;
		param->dol_type = ADAP_DOL_VC;
		param->offset.offset_x = 0;
		param->offset.offset_y = 0;
		param->offset.long_offset = 0x0;
		param->offset.short_offset = 0x0;
	} else {
		param->mode = MODE_MIPI_RAW_SDR_DDR;
		param->dol_type = ADAP_DOL_NONE;
		param->offset.offset_x = 12;
		param->offset.offset_y = 0;
		param->offset.long_offset = 0x8;
		param->offset.short_offset = 0x8;
	}

	if (adap_dev->ops->hw_reset)
		adap_dev->ops->hw_reset(adap_dev);

	if (adap_dev->ops->hw_init)
		adap_dev->ops->hw_init(adap_dev);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int adap_subdev_set_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *cfg,
				 struct v4l2_subdev_format *fmt)
#else
static int adap_subdev_set_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
#endif
{
	int rtn = 0;
	struct v4l2_mbus_framefmt *format;
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);
	struct adapter_dev_param *param = &adap_dev->param;

	format = adap_subdev_get_padfmt(adap_dev, cfg, fmt);
	if (format == NULL)
		return -EINVAL;

	*format = fmt->format;
	format->field = V4L2_FIELD_NONE;

	if (fmt->pad == AML_ADAP_PAD_SRC || fmt->pad == AML_ADAP_PAD_SRC_1) {
		// mem path
		param->mem_path_width =  (param->mem_path_width > format->width) ? param->mem_path_width : format->width;
		param->mem_path_height = (param->mem_path_height > format->height) ? param->mem_path_height : format->height;
		dev_info(adap_dev->dev, "mem path w %d h %d\n", param->mem_path_width, param->mem_path_height);
	} else if (fmt->pad == AML_ADAP_PAD_SRC_2 || fmt->pad == AML_ADAP_PAD_SRC_3) {
		// isp path
		param->isp_path_width = (param->isp_path_width > format->width) ? param->isp_path_width : format->width;
		param->isp_path_height = (param->isp_path_height > format->height) ? param->isp_path_height : format->height;
		dev_info(adap_dev->dev, "isp path w %d h %d\n", param->isp_path_width, param->isp_path_height);
	} else if (fmt->pad == AML_ADAP_PAD_SINK) {
		int adap_fmt = adap_subdev_convert_fmt(adap_dev, format);
		if (adap_fmt < 0) {
			dev_info(adap_dev->dev, "Error to convert fmt\n");
			return -1;
		}

		if (param->mem_path_width == 0 || param->mem_path_height == 0) {
			param->mem_path_width = format->width;
			param->mem_path_height = format->height;
			dev_info(adap_dev->dev, "mem path w %d h %d\n", param->mem_path_width, param->mem_path_height);
		}

		if (param->isp_path_width == 0 || param->isp_path_height == 0) {
			param->isp_path_width = format->width;
			param->isp_path_height = format->height;
			dev_info(adap_dev->dev, "isp path w %d h %d\n", param->isp_path_width, param->isp_path_height);
		}

		param->format = adap_fmt;
		rtn = adap_subdev_hw_init(adap_dev);
	}

	return rtn;
}

static int adap_subdev_log_status(struct v4l2_subdev *sd)
{
	struct adapter_dev_t *adap_dev = v4l2_get_subdevdata(sd);

	dev_info(adap_dev->dev, "log status done\n");

	return 0;
}

const struct v4l2_subdev_core_ops adap_subdev_core_ops = {
	.log_status = adap_subdev_log_status,
};

static const struct v4l2_subdev_video_ops adap_subdev_video_ops = {
	.s_stream = adap_subdev_set_stream,
};

static const struct v4l2_subdev_pad_ops adap_subdev_pad_ops = {
	.enum_mbus_code = adap_subdev_enum_mbus_code,
	.get_fmt = adap_subdev_get_format,
	.set_fmt = adap_subdev_set_format,
};

static const struct v4l2_subdev_ops adap_subdev_v4l2_ops = {
	.core = &adap_subdev_core_ops,
	.video = &adap_subdev_video_ops,
	.pad = &adap_subdev_pad_ops,
};

static int adap_v4l2_subdev_link_validate(struct media_link *link)
{
	int rtn = 0;
	struct media_entity *src = link->source->entity;
	struct media_entity *sink = link->sink->entity;

	rtn = v4l2_subdev_link_validate(link);
	if (rtn)
		pr_err("Error: src->sink: %s-->%s, rtn %d\n",
			src->name, sink->name, rtn);

	return rtn;
}

static const struct media_entity_operations adap_subdev_media_ops = {
	.link_validate = adap_v4l2_subdev_link_validate,
};

int aml_adap_subdev_register(struct adapter_dev_t *adap_dev)
{
	int rtn = -1;
	struct device *dev = adap_dev->dev;
	struct v4l2_subdev *sd = &adap_dev->sd;
	struct media_pad *pads = adap_dev->pads;
	struct v4l2_device *v4l2_dev = adap_dev->v4l2_dev;

	adap_dev->formats = adap_support_formats;
	adap_dev->fmt_cnt = ARRAY_SIZE(adap_support_formats);
	adap_dev->mem_frm_cnt = 0;
	adap_dev->isp_frm_cnt = 0;

	v4l2_subdev_init(sd, &adap_subdev_v4l2_ops);
	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), AML_ADAPTER_NAME, adap_dev->index);
	adap_dev->context_ge2d = NULL;

	struct adapter_workqueue_t* __adapter_wq;
	__adapter_wq = devm_kzalloc(dev, sizeof(*__adapter_wq), GFP_KERNEL);
	if (!kfifo_initialized(&__adapter_wq->adap_fifo_out)) {
		rtn = kfifo_alloc(&__adapter_wq->adap_fifo_out, MAX_ADAP_FIFO_LEN * sizeof(struct adapter_task_t), GFP_KERNEL);
		if (rtn)
			dev_err(dev, "alloc adap_workqueue fifo failed.\n");
	}
	INIT_WORK( &__adapter_wq->work_obj, adap_subdev_workqueue);
	adap_dev->adapter_wq = __adapter_wq;

	spin_lock_init(&adap_dev->param.ddr_lock);
	INIT_LIST_HEAD(&adap_dev->param.mem_path_list);
	INIT_LIST_HEAD(&adap_dev->param.isp_path_list);

	v4l2_set_subdevdata(sd, adap_dev);

	pads[AML_ADAP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[AML_ADAP_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	pads[AML_ADAP_PAD_SRC_1].flags = MEDIA_PAD_FL_SOURCE;
	pads[AML_ADAP_PAD_SRC_2].flags = MEDIA_PAD_FL_SOURCE;
	pads[AML_ADAP_PAD_SRC_3].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	sd->entity.ops = &adap_subdev_media_ops;

	rtn = media_entity_pads_init(&sd->entity, AML_ADAP_PAD_MAX, pads);
	if (rtn) {
		dev_err(dev, "Error init entity pads: %d\n", rtn);
		return rtn;
	}

	rtn = v4l2_device_register_subdev(v4l2_dev, sd);
	if (rtn < 0) {
		dev_err(dev, "Error to register subdev: %d\n", rtn);
		media_entity_cleanup(&sd->entity);
		return rtn;
	}

	dev_info(adap_dev->dev,
		"Success to register adapter-%u subdev\n", adap_dev->index);

	return rtn;
}

void aml_adap_subdev_unregister(struct adapter_dev_t *adap_dev)
{
	struct v4l2_subdev *sd = &adap_dev->sd;
	struct device *dev = adap_dev->dev;
	struct adapter_workqueue_t* __adapter_wq = adap_dev->adapter_wq;

	if (__adapter_wq) {
		kfifo_free(&__adapter_wq->adap_fifo_out);
		devm_kfree(dev, __adapter_wq);
	}
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
}

int aml_adap_subdev_init(void *c_dev)
{
	int rtn = -1;
	struct platform_device *pdev;
	struct adapter_dev_t *adap_dev;
	struct device_node *node;
	struct cam_device *cam_dev = c_dev;

	adap_dev = &cam_dev->adap_dev;

	node = of_parse_phandle(cam_dev->dev->of_node, "adapter", 0);
	if (!node) {
		pr_err("Failed to parse adapter handle\n");
		return rtn;
	}

	adap_dev->pdev = of_find_device_by_node(node);
	if (!adap_dev->pdev) {
		of_node_put(node);
		pr_err("Failed to find adapter platform device");
		return rtn;
	}
	of_node_put(node);

	pdev = adap_dev->pdev;
	adap_dev->dev = &pdev->dev;
	adap_dev->v4l2_dev = &cam_dev->v4l2_dev;
	adap_dev->index = cam_dev->index;
	adap_dev->bus_info = cam_dev->bus_info;
	adap_dev->is_streaming = 0;
	platform_set_drvdata(pdev, adap_dev);

	adap_dev->ops = &adap_dev_hw_ops;

	rtn = adap_of_parse_dev(adap_dev);
	if (rtn) {
		dev_err(adap_dev->dev, "Failed to parse dev\n");
		return rtn;
	}

	if (adap_dev->index == 0) {
		rtn = devm_request_irq(adap_dev->dev, adap_dev->fe_0_irq,
			adap_interrupt_handler,
			IRQF_SHARED, "fe0", adap_dev);
	} else if (adap_dev->index == 1) {
		rtn = devm_request_irq(adap_dev->dev, adap_dev->fe_2_irq,
			adap_interrupt_handler,
			IRQF_SHARED, "fe2", adap_dev);
	}
	if (rtn) {
		adap_iounmap_resource(adap_dev);
		dev_err(adap_dev->dev, "failed reg irq" );
		return rtn;
	} else {
		dev_info(adap_dev->dev, "success reg irq" );
	}

	clk_set_rate(adap_dev->adap_clk, 666666667);
	rtn = clk_prepare_enable(adap_dev->adap_clk);
	if (rtn) {
		adap_iounmap_resource(adap_dev);
		return rtn;
	}

	dev_info(adap_dev->dev, "Success adapter-%u subdev init\n", adap_dev->index);

	return rtn;
}

void aml_adap_subdev_deinit(void *c_dev)
{
	struct adapter_dev_t *adap_dev;
	struct cam_device *cam_dev;

	cam_dev = c_dev;
	adap_dev = &cam_dev->adap_dev;

	devm_free_irq(adap_dev->dev, adap_dev->fe_0_irq, adap_dev);
	devm_free_irq(adap_dev->dev, adap_dev->fe_2_irq, adap_dev);

	clk_disable_unprepare(adap_dev->adap_clk);
	if (adap_dev->adap_clk != NULL) {
		devm_clk_put(adap_dev->dev, adap_dev->adap_clk);
		adap_dev->adap_clk = NULL;
	}

	adap_iounmap_resource(adap_dev);

	dev_info(adap_dev->dev, "Success adapter-%u subdev deinit\n", adap_dev->index);
}

