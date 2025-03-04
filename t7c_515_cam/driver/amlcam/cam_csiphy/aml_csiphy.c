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
#include <linux/version.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "aml_cam.h"

#define AML_CSIPHY_NAME "isp-csiphy"

static struct csiphy_dev_t *g_csiphy_dev[4];

struct csiphy_dev_t *csiphy_get_dev(int idx)
{
	return g_csiphy_dev[idx];
}

static const struct aml_format csiphy_support_formats[] = {
	{0, 0, 0, 0, MEDIA_BUS_FMT_SBGGR8_1X8, 0, 1, 8},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGBRG8_1X8, 0, 1, 8},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGRBG8_1X8, 0, 1, 8},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SRGGB8_1X8, 0, 1, 8},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SBGGR10_1X10, 0, 1, 10},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGBRG10_1X10, 0, 1, 10},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGRBG10_1X10, 0, 1, 10},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SRGGB10_1X10, 0, 1, 10},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SBGGR12_1X12, 0, 1, 12},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGBRG12_1X12, 0, 1, 12},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGRBG12_1X12, 0, 1, 12},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SRGGB12_1X12, 0, 1, 12},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SBGGR14_1X14, 0, 1, 14},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGBRG14_1X14, 0, 1, 14},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SGRBG14_1X14, 0, 1, 14},
	{0, 0, 0, 0, MEDIA_BUS_FMT_SRGGB14_1X14, 0, 1, 14},
};

static int csiphy_of_parse_endpoint_node(struct device_node *node,
					struct csiphy_async_subdev *c_asd)
{
	int rtn = -1;
	struct v4l2_fwnode_endpoint vep = { { 0 } };

	rtn = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &vep);
	if (rtn) {
		aml_cam_log_err("Failed to parse csiphy endpoint\n");
		return rtn;
	}

	c_asd->phy_id = vep.base.id;
	c_asd->data_lanes = vep.bus.mipi_csi2.num_data_lanes;

	return 0;
}

static int csiphy_of_parse_ports(struct csiphy_dev_t *csiphy_dev)
{
	unsigned int rtn = 0;
	struct device_node *node = NULL;
	struct device_node *remote = NULL;
	struct csiphy_async_subdev *c_asd = NULL;
	struct v4l2_async_subdev *asd = NULL;
	struct v4l2_async_notifier *notifier = csiphy_dev->notifier;
	struct device *dev = csiphy_dev->dev;
#ifdef SENSOR_SEARCH
	struct device *sensor_dev = NULL;
#endif
	v4l2_async_notifier_init(notifier);

	for_each_endpoint_of_node(dev->of_node, node) {
		if (!of_device_is_available(node))
			continue;

		remote = of_graph_get_remote_port_parent(node);
		if (!remote) {
			aml_cam_log_err("Cannot get remote parent\n");
			of_node_put(node);
			return -EINVAL;
		}
#ifdef SENSOR_SEARCH
		sensor_dev = of_fwnode_handle(remote)->dev;
		if ( sensor_dev->driver && strstr(node->name, sensor_dev->driver->name) ) {
			aml_cam_log_err("subdev driver name %s \n", sensor_dev->driver->name);
		} else
			continue;
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
		asd = __v4l2_async_notifier_add_fwnode_subdev(notifier,
				of_fwnode_handle(remote),
				sizeof(struct csiphy_async_subdev));
#else
		asd = v4l2_async_notifier_add_fwnode_subdev(notifier,
				of_fwnode_handle(remote),
				sizeof(*c_asd));
#endif
		if (IS_ERR(asd)) {
			aml_cam_log_err("Failed to add subdev\n");
			of_node_put(node);
			return -EINVAL;
		}

		c_asd = container_of(asd, struct csiphy_async_subdev, asd);

		rtn = csiphy_of_parse_endpoint_node(node, c_asd);
		if (rtn < 0) {
			of_node_put(node);
			return rtn;
		}
#ifdef SENSOR_SEARCH
		of_node_put(node);
		return rtn;//only one sensor supported
#endif
	}

	of_node_put(node);

	return rtn;
}

static void csiphy_notifier_cleanup(void *c_dev)
{
	struct csiphy_dev_t *csiphy_dev;
	struct v4l2_async_notifier *notifier;

	csiphy_dev = c_dev;
	notifier = csiphy_dev->notifier;

	v4l2_async_notifier_cleanup(notifier);
}

static void __iomem *csiphy_ioremap_resource(void *c_dev, char *name)
{
	void __iomem *reg;
	struct resource *res;
	resource_size_t size;
	struct csiphy_dev_t *csiphy_dev;
	struct device *dev;
	struct platform_device *pdev;

	if (!c_dev || !name) {
		aml_cam_log_err("Error input param\n");
		return NULL;
	}

	csiphy_dev = c_dev;
	dev = csiphy_dev->dev;
	pdev = csiphy_dev->pdev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		aml_cam_log_err("Error %s res\n", name);
		return NULL;
	}

	size = resource_size(res);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
	reg = devm_ioremap(dev, res->start, size);
#else
	reg = devm_ioremap_nocache(dev, res->start, size);
#endif
	if (!reg) {
		aml_cam_log_err("Failed to ioremap %s %pR\n", name, res);
		reg = IOMEM_ERR_PTR(-ENOMEM);
	}

	return reg;
}

static void csiphy_iounmap_resource(void *c_dev)
{
	struct device *dev;
	struct csiphy_dev_t *csiphy_dev;

	csiphy_dev = c_dev;
	dev = csiphy_dev->dev;

	if (csiphy_dev->csi_dphy) {
		devm_iounmap(dev, csiphy_dev->csi_dphy);
		csiphy_dev->csi_dphy = NULL;
	}

	if (csiphy_dev->csi_aphy) {
		devm_iounmap(dev, csiphy_dev->csi_aphy);
		csiphy_dev->csi_aphy = NULL;
	}
}

static int csiphy_of_parse_version(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = 0;
	u32 *version;
	struct device *dev;

	dev = csiphy_dev->dev;
	version = &csiphy_dev->version;

	switch (*version) {
	case 0:
		csiphy_dev->ops = &csiphy_dev_hw_ops;
	break;
	default:
		rtn = -EINVAL;
		aml_cam_log_err("Error invalid version num: %u\n", *version);
	break;
	}

	return rtn;
}

static int csiphy_of_parse_dev(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = -1;

	csiphy_dev->csi_dphy = csiphy_ioremap_resource(csiphy_dev, "csi_phy");
	if (!csiphy_dev->csi_dphy) {
		rtn = -EINVAL;
		goto error_rtn;
	}

	csiphy_dev->csi_aphy = csiphy_ioremap_resource(csiphy_dev, "csi_aphy");
	if (!csiphy_dev->csi_aphy) {
		rtn = -EINVAL;
		goto error_rtn;
	}

	//csiphy_dev->csiphy_clk = devm_clk_get(csiphy_dev->dev, "cts_mipi_csi_phy_clk");
	csiphy_dev->csiphy_clk = devm_clk_get(csiphy_dev->dev, "mipi_phy_clk");
	if (IS_ERR(csiphy_dev->csiphy_clk)) {
		aml_cam_log_err("Error to get csiphy_clk\n");
		return PTR_ERR(csiphy_dev->csiphy_clk);
	}

	csiphy_dev->csiphy_clk1 = devm_clk_get(csiphy_dev->dev, "mipi_phy_clk1");
	if (IS_ERR(csiphy_dev->csiphy_clk1)) {
		aml_cam_log_err("Error to get csiphy_clk1\n");
		return PTR_ERR(csiphy_dev->csiphy_clk1);
	}

	rtn = 0;

error_rtn:
	return rtn;
}

static struct media_entity *csiphy_subdev_get_sensor_entity(struct media_entity *entity)
{
	struct media_pad *pad;
	struct media_pad *r_pad;

	pad = &entity->pads[AML_CSIPHY_PAD_SINK];
	if (!(pad->flags & MEDIA_PAD_FL_SINK))
		return NULL;

	r_pad = media_entity_remote_pad(pad);
	if (!r_pad || !is_media_entity_v4l2_subdev(r_pad->entity))
		return NULL;

	if (r_pad->entity->function == MEDIA_ENT_F_CAM_SENSOR)
		return r_pad->entity;
	else
		return NULL;
}

static int csiphy_subdev_querymenu(struct v4l2_ctrl_handler *hdl, struct v4l2_querymenu *qm)
{
	struct v4l2_ctrl *ctrl;
	u32 i = qm->index;

	ctrl = v4l2_ctrl_find(hdl, qm->id);
	if (!ctrl)
		return -EINVAL;

	qm->reserved = 0;
	/* Sanity checks */
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_MENU:
		if (!ctrl->qmenu)
			return -EINVAL;
		break;
	case V4L2_CTRL_TYPE_INTEGER_MENU:
		if (!ctrl->qmenu_int)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (i < ctrl->minimum || i > ctrl->maximum)
		return -EINVAL;

	/* Use mask to see if this menu item should be skipped */
	if (ctrl->menu_skip_mask & (1ULL << i))
		return -EINVAL;
	/* Empty menu items should also be skipped */
	if (ctrl->type == V4L2_CTRL_TYPE_MENU) {
		if (!ctrl->qmenu[i] || ctrl->qmenu[i][0] == '\0')
			return -EINVAL;
		strscpy(qm->name, ctrl->qmenu[i], sizeof(qm->name));
	} else {
		qm->value = ctrl->qmenu_int[i];
	}
	return 0;
}

static int csiphy_subdev_get_link_freq(struct media_entity *entity, s64 *link_freq)
{
	int rtn = 0;
	struct v4l2_querymenu qm;
	struct media_entity *sensor;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl *ctrl;

	sensor = csiphy_subdev_get_sensor_entity(entity);
	if (!sensor) {
		aml_cam_log_err("Failed to get sensor entity\n");
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(sensor);

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_LINK_FREQ);
	if (!ctrl) {
		aml_cam_log_err("Failed to get link freq ctrl\n");
		return -EINVAL;
	}

	qm.id = V4L2_CID_LINK_FREQ;
	qm.index = ctrl->val;

	rtn = csiphy_subdev_querymenu(subdev->ctrl_handler, &qm);
	if (rtn) {
		aml_cam_log_err("Failed to querymenu idx %d\n", qm.index);
		return rtn;
	}

	*link_freq = qm.value;

	return 0;
}

static int csiphy_subdev_get_clock_mode(struct media_entity *entity)
{
	int clock_mode = 0;
	struct media_entity *sensor;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl *ctrl;

	sensor = csiphy_subdev_get_sensor_entity(entity);
	if (!sensor) {
		aml_cam_log_err("Failed to get sensor entity\n");
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(sensor);

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_AML_CLOCK_MODE);
	if (!ctrl) {
		aml_cam_log_err("Failed to get clock mode, using fault value\n");
	} else {
		clock_mode = ctrl->val;
	}
	aml_cam_log_dbg("clock mode: %d \n", clock_mode);
	return clock_mode;

}

static int csiphy_subdev_get_lanes(struct media_entity *entity, int *data_lanes)
{
	struct media_entity *sensor;
	struct v4l2_subdev *subdev;
	struct v4l2_ctrl *ctrl;

	sensor = csiphy_subdev_get_sensor_entity(entity);
	if (!sensor) {
		aml_cam_log_err("Failed to get sensor entity\n");
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(sensor);

	ctrl = v4l2_ctrl_find(subdev->ctrl_handler, V4L2_CID_AML_CSI_LANES);
	if (!ctrl) {
		aml_cam_log_err("Failed to get csi lanes ctrl\n");
		return -EINVAL;
	}

	*data_lanes = ctrl->val;

	return 0;
}

static int csiphy_subdev_stream_on(void *priv)
{
	int rtn = -1, nlanes = 0;
	s64 link_freq = 0;
	struct csiphy_dev_t *csiphy_dev = priv;
	rtn = csiphy_subdev_get_link_freq(&csiphy_dev->sd.entity, &link_freq);
	if (rtn)
		return rtn;

	rtn = csiphy_subdev_get_lanes(&csiphy_dev->sd.entity, &nlanes);
	if (rtn)
		return rtn;

	csiphy_dev->lanecnt = nlanes;
	csiphy_dev->lanebps = link_freq;
	csiphy_dev->clock_mode = csiphy_subdev_get_clock_mode(&csiphy_dev->sd.entity);

	return csiphy_dev->ops->hw_start(csiphy_dev, csiphy_dev->index, nlanes, link_freq);
}

static void csiphy_subdev_stream_off(void *priv)
{
	struct csiphy_dev_t *csiphy_dev = priv;

	csiphy_dev->ops->hw_stop(csiphy_dev, csiphy_dev->index);
}

static void csiphy_subdev_log_status(void *priv)
{
	struct csiphy_dev_t *csiphy_dev = priv;

	aml_cam_log_info("Log status done\n");
}

static const struct aml_sub_ops csiphy_subdev_ops = {
	.stream_on = csiphy_subdev_stream_on,
	.stream_off = csiphy_subdev_stream_off,
	.log_status = csiphy_subdev_log_status,
};

static int csiphy_subdev_power_on(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = 0;
	aml_cam_log_info("%s in\n", __func__);

	dev_pm_domain_attach(csiphy_dev->dev, true);
	pm_runtime_enable(csiphy_dev->dev);
	pm_runtime_get_sync(csiphy_dev->dev);

	if (!__clk_is_enabled(csiphy_dev->csiphy_clk)) {
		clk_set_rate(csiphy_dev->csiphy_clk, 200000000);
		rtn = clk_prepare_enable(csiphy_dev->csiphy_clk);
		if (rtn)
			aml_cam_log_err("Error to enable csiphy_clk\n");
	}

	if (!__clk_is_enabled(csiphy_dev->csiphy_clk1)) {

		clk_set_rate(csiphy_dev->csiphy_clk1, 200000000);
		rtn = clk_prepare_enable(csiphy_dev->csiphy_clk1);
		if (rtn)
			aml_cam_log_err("Error to enable csiphy_clk1n");
	}
	return rtn;
}

static void csiphy_subdev_power_off(struct csiphy_dev_t *csiphy_dev)
{
	aml_cam_log_info("%s in\n", __func__);

	clk_disable_unprepare(csiphy_dev->csiphy_clk);
	clk_disable_unprepare(csiphy_dev->csiphy_clk1);

	pm_runtime_put_sync(csiphy_dev->dev);
	pm_runtime_disable(csiphy_dev->dev);
	dev_pm_domain_detach(csiphy_dev->dev, true);
}

void csiphy_subdev_suspend(struct csiphy_dev_t *csiphy_dev)
{
	aml_cam_log_info("%s in\n", __func__);

	if (__clk_is_enabled(csiphy_dev->csiphy_clk))
		clk_disable_unprepare(csiphy_dev->csiphy_clk);

	if (__clk_is_enabled(csiphy_dev->csiphy_clk1))
		clk_disable_unprepare(csiphy_dev->csiphy_clk1);

	pm_runtime_put_sync(csiphy_dev->dev);
	pm_runtime_disable(csiphy_dev->dev);
	dev_pm_domain_detach(csiphy_dev->dev, true);

	aml_cam_log_info("%s out \n", __func__);
}

int csiphy_subdev_resume(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = 0;
	dev_pm_domain_attach(csiphy_dev->dev, true);

	pm_runtime_enable(csiphy_dev->dev);
	pm_runtime_get_sync(csiphy_dev->dev);

	if (!__clk_is_enabled(csiphy_dev->csiphy_clk)) {
		rtn = clk_prepare_enable(csiphy_dev->csiphy_clk);
		if (rtn)
			aml_cam_log_err("Error to enable csiphy_clk\n");
	}
	if (!__clk_is_enabled(csiphy_dev->csiphy_clk1)) {
		rtn = clk_prepare_enable(csiphy_dev->csiphy_clk1);
		if (rtn)
			aml_cam_log_err("Error to enable csiphy_clk1\n");
	}
	return rtn;
}

static int csiphy_proc_show(struct seq_file *proc_entry, void *arg ) {

	struct csiphy_dev_t *c_dev = proc_entry->private;

	u32 *mipi_info = c_dev->ops->hw_csiphy_info(c_dev);

	seq_printf(
		proc_entry, "\n*******************CSIPHY MODULE PARAM*******************\n");

	seq_printf(proc_entry, " ------- PubAttr Info ------- \n");
	seq_printf(proc_entry, "LaneCnt" "\t" "LaneBps" "\t" "\n");
	seq_printf(proc_entry, "%d \t %ld \t \n\n",
					c_dev->lanecnt,
					c_dev->lanebps );

	seq_printf(proc_entry, " ------- Csiphy Info ------- \n");
	seq_printf(proc_entry,"clk" "\t" "Lane1" "\t" "Lane2" "\t" "Lane3" "\t" "Lane4" "\n");
	seq_printf(proc_entry, "0x%02x \t 0x%02x \t 0x%02x \t 0x%02x \t 0x%02x \n\n",
					*mipi_info,
					*(mipi_info + 1),
					*(mipi_info + 2),
					*(mipi_info + 3),
					*(mipi_info + 4) );

	seq_printf(proc_entry, " ------- Dphy Cfg ------- \n");
	seq_printf(proc_entry,"MIPI_PHY_MUX_CTRL0" "\t" "MIPI_PHY_MUX_CTRL1" "\t" "\n");
	seq_printf(proc_entry, "0x%02x \t\t\t 0x%02x \n\n",
				*(mipi_info + 5),
				*(mipi_info + 6) );

	seq_printf(proc_entry, " ------- Aphy Cfg ------- \n");
	seq_printf(proc_entry,"MIPI_CSI_2M_PHY2_CNTL1" "\t" "MIPI_CSI_2M_PHY2_CNTL2" "\t" "MIPI_CSI_2M_PHY2_CNTL3" "\t"
		"\n" "MIPI_CSI_2M_PHY2_CNTL4" "\t" "MIPI_CSI_2M_PHY2_CNTL5" "\t" "MIPI_CSI_2M_PHY2_CNTL6" "\t" "\n");
	seq_printf(proc_entry, "0x%02x \t\t 0x%02x \t\t 0x%02x \t\n 0x%02x \t\t 0x%02x \t\t 0x%02x \t\t \n\n",
				*(mipi_info + 7),
				*(mipi_info + 8),
				*(mipi_info + 9),
				*(mipi_info + 10),
				*(mipi_info + 11),
				*(mipi_info + 12) );

	seq_printf(proc_entry,"CSI2_host_err1" "\t" "CSI2_host_err2" "\n");
	seq_printf(proc_entry, "0x%02x \t\t 0x%02x \n\n",
					*(mipi_info + 13),
					*(mipi_info + 14) );

	seq_printf(proc_entry, "******************* PROC END *******************\n\n");

	return 0;
}

static int csiphy_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, csiphy_proc_show, PDE_DATA(inode));
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
static const struct proc_ops csiphy_proc_file_ops = {
	/// kernel 5.15
	.proc_open = csiphy_debug_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations csiphy_proc_file_ops = {
	.open = csiphy_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int csiphy_proc_init(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = -1;
	char file_name[50] = {0};
	struct aml_subdev *subdev = &csiphy_dev->subdev;

	sprintf(file_name, "%s%d", "csiphy", csiphy_dev->index);
	subdev->proc_node_entry = proc_create_data(file_name, 0644, NULL, &csiphy_proc_file_ops, csiphy_dev);
	if (subdev->proc_node_entry == NULL) {
		aml_cam_log_err("csiphy%u create proc failed!\n", csiphy_dev->index);
		return rtn;
	}

	return 0;
}

static void csiphy_proc_exit(struct csiphy_dev_t *csiphy_dev) {

	char file_name[50] = {0};
	sprintf(file_name, "%s%d", "csiphy", csiphy_dev->index);

	remove_proc_entry(file_name, NULL);
}


int aml_csiphy_subdev_register(struct csiphy_dev_t *csiphy_dev)
{
	int rtn = -1;
	struct media_pad *pads = csiphy_dev->pads;
	struct aml_subdev *subdev = &csiphy_dev->subdev;

	csiphy_dev->formats = csiphy_support_formats;
	csiphy_dev->fmt_cnt = ARRAY_SIZE(csiphy_support_formats);

	pads[AML_CSIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[AML_CSIPHY_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	subdev->name = AML_CSIPHY_NAME;
	subdev->dev = csiphy_dev->dev;
	subdev->sd = &csiphy_dev->sd;
	subdev->function = MEDIA_ENT_F_VID_IF_BRIDGE;
	subdev->v4l2_dev = csiphy_dev->v4l2_dev;
	subdev->pads = pads;
	subdev->pad_max = AML_CSIPHY_PAD_MAX;
	subdev->formats = csiphy_dev->formats;
	subdev->fmt_cnt = csiphy_dev->fmt_cnt;
	subdev->pfmt = csiphy_dev->pfmt;
	subdev->ops = &csiphy_subdev_ops;
	subdev->priv = csiphy_dev;

	rtn = csiphy_proc_init(csiphy_dev);
	if (rtn)
		goto error_rtn;

	rtn = aml_subdev_register(subdev);
	if (rtn)
		goto error_rtn;

	aml_cam_log_info("CSIPHY%u: register subdev\n", csiphy_dev->index);

error_rtn:
	return rtn;
}

void aml_csiphy_subdev_unregister(struct csiphy_dev_t *csiphy_dev)
{
	csiphy_proc_exit(csiphy_dev);
	aml_subdev_unregister(&csiphy_dev->subdev);
}

int aml_csiphy_subdev_init(void *c_dev)
{
	int rtn = -1;
	struct platform_device *pdev;
	struct csiphy_dev_t *csiphy_dev;
	struct device_node *node;
	struct cam_device *cam_dev = c_dev;

	csiphy_dev = &cam_dev->csiphy_dev;

	node = of_parse_phandle(cam_dev->dev->of_node, "csiphy", 0);
	if (!node) {
		aml_cam_log_err("Failed to parse csiphy handle\n");
		return rtn;
	}

	csiphy_dev->pdev = of_find_device_by_node(node);
	if (!csiphy_dev->pdev) {
		of_node_put(node);
		aml_cam_log_err("Failed to find csiphy platform device");
		return rtn;
	}
	of_node_put(node);

	pdev = csiphy_dev->pdev;
	csiphy_dev->dev = &pdev->dev;
	csiphy_dev->v4l2_dev = &cam_dev->v4l2_dev;
	csiphy_dev->notifier = &cam_dev->notifier;
	csiphy_dev->index = cam_dev->index;
	platform_set_drvdata(pdev, csiphy_dev);

	rtn = csiphy_of_parse_ports(csiphy_dev);
	if (rtn) {
		aml_cam_log_err("Failed to parse port\n");
		return -ENODEV;
	}

	rtn = csiphy_of_parse_version(csiphy_dev);
	if (rtn) {
		aml_cam_log_err("Failed to parse version\n");
		return rtn;
	}

	rtn = csiphy_of_parse_dev(csiphy_dev);
	if (rtn) {
		aml_cam_log_err("Failed to parse dev\n");
		return rtn;
	}

	rtn = csiphy_subdev_power_on(csiphy_dev);
	if (rtn) {
		aml_cam_log_err("Failed to power on\n");
		return rtn;
	}

	aml_cam_log_info("CSIPHY%u: subdev init\n", csiphy_dev->index);

	g_csiphy_dev[cam_dev->index] = csiphy_dev;

	return rtn;
}


void aml_csiphy_subdev_deinit(void *c_dev)
{
	struct csiphy_dev_t *csiphy_dev;
	struct cam_device *cam_dev = c_dev;

	csiphy_dev = &cam_dev->csiphy_dev;

	csiphy_subdev_power_off(csiphy_dev);

	csiphy_iounmap_resource(csiphy_dev);

	csiphy_notifier_cleanup(csiphy_dev);

	devm_clk_put(csiphy_dev->dev, csiphy_dev->csiphy_clk);

	aml_cam_log_info("CSIPHY%u: subdev deinit\n", csiphy_dev->index);
}
