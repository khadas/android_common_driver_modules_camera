/*
 * isp.c
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[amlcam]:%s:%d: " fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mc.h>
#include <linux/notifier.h>
#include "aml_t7_cam.h"

#ifdef DEBUG_TEST_MIPI_RESET
extern int debug_test_mipi_reset;
#endif


static void cam_debug_dq_check_timeout(struct timer_list *t);

static int cam_subdevs_register(struct cam_device *cam_dev)
{
	int rtn;

	rtn = aml_csiphy_subdev_register(&cam_dev->csiphy_dev);
	if (rtn)
		return rtn;

	rtn = aml_adap_subdev_register(&cam_dev->adap_dev);
	if (rtn)
		return rtn;

	return 0;
}

static int cam_videos_register(struct cam_device *cam_dev)
{
	int rtn = 0;

	switch (cam_dev->index) {
	case AML_CAM_0:
	case AML_CAM_1:
		rtn = aml_adap_video_register(&cam_dev->adap_dev);
	break;
	default:
		rtn = -EINVAL;
		dev_err(cam_dev->dev, "Error camera index: %u\n", cam_dev->index);
	break;
	}

	return rtn;
}

static void cam_subdevs_unregister(struct cam_device *cam_dev)
{
	aml_csiphy_subdev_unregister(&cam_dev->csiphy_dev);
	aml_adap_subdev_unregister(&cam_dev->adap_dev);
}

static void cam_videos_unregister(struct cam_device *cam_dev)
{
	switch (cam_dev->index) {
	case AML_CAM_0:
	case AML_CAM_1:
		aml_adap_video_unregister(&cam_dev->adap_dev);
	break;
	default:
		dev_err(cam_dev->dev, "Error camera index: %u\n", cam_dev->index);
	break;
	}
}

static int cam_create_csiphy_adap_links(struct cam_device *cam_dev)
{
	int rtn = 0;
	u32 flags = 0;
	struct media_entity *csiphy = &cam_dev->csiphy_dev.sd.entity;
	struct media_entity *adap = &cam_dev->adap_dev.sd.entity;

	flags = MEDIA_LNK_FL_ENABLED;
	rtn = media_create_pad_link(csiphy, AML_CSIPHY_PAD_SRC,
				adap, AML_ADAP_PAD_SINK, flags);
	if (rtn) {
		dev_err(cam_dev->dev, "Failed to link %s->%s entity\n",
			csiphy->name, adap->name);
		return rtn;
	}

	return rtn;
}

static int cam_create_adap_video_links(struct cam_device *cam_dev)
{
	int rtn = 0;
	u32 flags = 0, i = 0;
	struct media_entity *video;
	struct media_entity *adap = &cam_dev->adap_dev.sd.entity;

	flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED;

	for ( i = 0; i < AML_ADAP_STREAM_MAX; ++i) {
		video = &cam_dev->adap_dev.video[i].vdev.entity;

		rtn = media_create_pad_link(adap, AML_ADAP_PAD_SRC + i, video, 0, flags);
		if (rtn) {
			dev_err(cam_dev->dev, "Failed to link %s->%s entity\n",
					video->name, adap->name);
			return rtn;
		}
	}
	return rtn;
}

static int cam_create_links(struct cam_device *cam_dev)
{
	int rtn = 0;
	u32 flags = 0, i = 0;
	struct media_entity *video;
	struct media_entity *csiphy = &cam_dev->csiphy_dev.sd.entity;
	struct media_entity *adap = &cam_dev->adap_dev.sd.entity;

	rtn = cam_create_csiphy_adap_links(cam_dev);
	if (rtn)
		return rtn;

	switch (cam_dev->index) {
	case AML_CAM_0:
	case AML_CAM_1:
		rtn = cam_create_adap_video_links(cam_dev);
	break;
	default:
		rtn = -EINVAL;
		dev_err(cam_dev->dev, "Error camera index: %u\n", cam_dev->index);
	break;
	}

	return rtn;
}

static int cam_async_notifier_bound(struct v4l2_async_notifier *async,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct csiphy_async_subdev *c_asd =
			container_of(asd, struct csiphy_async_subdev, asd);
	pr_info("bound called with sd 0x%x, asd 0x%x, sd->dev 0x%x, name %s", subdev, asd, subdev->dev, subdev->name);

	subdev->host_priv = c_asd;

	return 0;
}

void cam_async_notifier_unbind(struct v4l2_async_notifier *async,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct cam_device *cam_dev = container_of(async, struct cam_device, notifier);

	media_device_unregister(&cam_dev->media_dev);
	cam_videos_unregister(cam_dev);
}

static int cam_async_notifier_complete(struct v4l2_async_notifier *async)
{
	int rtn = -1;
	unsigned int i;
	struct v4l2_subdev *sd;
	struct media_entity *sensor;
	struct media_entity *csiphy;
	struct csiphy_dev_t *csiphy_dev;
	struct cam_device *cam_dev = container_of(async, struct cam_device, notifier);
	struct v4l2_device *v4l2_dev = &cam_dev->v4l2_dev;

	list_for_each_entry(sd, &v4l2_dev->subdevs, list) {
		if (!sd->host_priv)
			continue;

		sensor = &sd->entity;
		csiphy_dev = &cam_dev->csiphy_dev;
		csiphy = &csiphy_dev->sd.entity;

		for (i = 0; i < sensor->num_pads; i++) {
			if (sensor->pads[i].flags & MEDIA_PAD_FL_SOURCE)
					break;
		}
		if (i == sensor->num_pads) {
			rtn = -EINVAL;
			dev_err(cam_dev->dev, "No source pad in sensor\n");
			goto error_return;
		}

		rtn = media_create_pad_link(sensor, i, csiphy,
				AML_CSIPHY_PAD_SINK, MEDIA_LNK_FL_ENABLED);
		if (rtn < 0) {
			dev_err(cam_dev->dev,
					"Failed to link %s->%s entities: %d\n",
					sensor->name, csiphy->name, rtn);
				goto error_return;
		}
	}

	rtn = cam_videos_register(cam_dev);
	if (rtn < 0) {
		dev_err(cam_dev->dev, "Failed to register video node: %d\n", rtn);
		goto error_return;
	}

	rtn = cam_create_links(cam_dev);
	if (rtn) {
		dev_err(cam_dev->dev, "Failed to create links: %d\n", rtn);
		goto error_video;
	}

	rtn = v4l2_device_register_subdev_nodes(&cam_dev->v4l2_dev);
	if (rtn < 0) {
		dev_err(cam_dev->dev, "Failed to register sd node: %d\n", rtn);
		goto error_video;
	}

	rtn = media_device_register(&cam_dev->media_dev);
	if (rtn) {
		dev_err(cam_dev->dev, "Failed to register media: %d\n", rtn);
		goto error_video;
	}

	dev_info(cam_dev->dev, "Success async notifier complete\n");

	timer_setup(&(cam_dev->dq_check_timer ), cam_debug_dq_check_timeout, 0);

	dev_info(cam_dev->dev, "dq check timer setup\n");

	return rtn;

error_video:
	cam_videos_unregister(cam_dev);
error_return:
	return rtn;
}

static const struct v4l2_async_notifier_operations cam_async_notifier_ops = {
	.bound = cam_async_notifier_bound,
	.unbind = cam_async_notifier_unbind,
	.complete = cam_async_notifier_complete,
};

static int cam_async_notifier_register(struct cam_device *cam_dev)
{
	int rtn = -1;
	struct v4l2_device *v4l2_dev = &cam_dev->v4l2_dev;
	struct v4l2_async_notifier *notifier = &cam_dev->notifier;

	if (list_empty(&notifier->asd_list)) {
		dev_err(cam_dev->dev, "Error input param\n");
		return -EINVAL;
	}

	notifier->ops = &cam_async_notifier_ops;

	rtn = v4l2_async_notifier_register(v4l2_dev, notifier);
	if (rtn)
		dev_err(cam_dev->dev, "Failed to notifier register: %d\n", rtn);

	return rtn;
}

static int cam_init_subdevices(struct cam_device *cam_dev)
{
	int rtn = -1;

	rtn = aml_csiphy_subdev_init(cam_dev);
	if (rtn) {
		dev_err(cam_dev->dev, "Failed to init csiphy subdev: %d\n", rtn);
		return rtn;
	}

	rtn = aml_adap_subdev_init(cam_dev);
	if (rtn) {
		dev_err(cam_dev->dev, "Failed to init adap subdev: %d\n", rtn);
		return rtn;
	}

	return rtn;
}

static void cam_deinit_subdevices(struct cam_device *cam_dev)
{
	aml_adap_subdev_deinit(cam_dev);
	aml_csiphy_subdev_deinit(cam_dev);
}

static int cam_link_notify(struct media_link *link, u32 flags,
			   unsigned int notification)
{
	pr_debug("amlcam: %s --> %s, flag %u\n",
		link->source->entity->name,
		link->sink->entity->name,
		flags);

	return 0;
}

const struct media_device_ops mdev_ops = {
	.link_notify = cam_link_notify,
};

static void cam_media_dev_init(struct cam_device *cam_dev)
{
	snprintf(cam_dev->media_dev.driver_name,
		sizeof(cam_dev->media_dev.driver_name),
		AML_CAM_DRIVER_NAME, cam_dev->index);
	snprintf(cam_dev->media_dev.model,
		sizeof(cam_dev->media_dev.model),
		AML_CAM_DRIVER_NAME, cam_dev->index);
	snprintf(cam_dev->media_dev.bus_info,
		sizeof(cam_dev->media_dev.bus_info),
		AML_CAM_BUS_INFO, cam_dev->index);

	cam_dev->bus_info = cam_dev->media_dev.bus_info;
	cam_dev->media_dev.dev = cam_dev->dev;
	cam_dev->media_dev.ops = &mdev_ops;

	media_device_init(&cam_dev->media_dev);
}

void cam_debug_mipi_dump(struct cam_device *cam_dev)
{
	extern void adap_hw_dump_reg(struct adapter_dev_t *ctx);
	extern void csiphy_hw_dump_reg(struct csiphy_dev_t * ctx);

	extern  uint32_t adap_debug_irq_in_count;
	extern  uint32_t adap_debug_irq_out_count;

	pr_err("adap irq in %d,  out %d", adap_debug_irq_in_count, adap_debug_irq_out_count);
	pr_err("adap irq in %d,  out %d", adap_debug_irq_in_count, adap_debug_irq_out_count);

	//csiphy_hw_dump_reg(&(cam_dev->csiphy_dev) );
	//adap_hw_dump_reg(&(cam_dev->adap_dev));
}

int cam_debug_mipi_off(struct cam_device *cam_dev)
{
	// stream off
	cam_dev->adap_dev.ops->hw_stream_off(&(cam_dev->adap_dev));
	cam_dev->adap_dev.ops->hw_stop(&(cam_dev->adap_dev) );

	// deinit
	cam_dev->csiphy_dev.ops->hw_stop(&(cam_dev->csiphy_dev) );
	return 0;
}

int cam_debug_mipi_on(struct cam_device *cam_dev)
{

	// re-init adap & csiphy;
	cam_dev->adap_dev.ops->hw_reset( &(cam_dev->adap_dev) );
	cam_dev->adap_dev.ops->hw_init( &(cam_dev->adap_dev) );

	// stream on
	cam_dev->adap_dev.ops->hw_start(&(cam_dev->adap_dev) );
	cam_dev->adap_dev.ops->hw_stream_on(&(cam_dev->adap_dev));

	cam_dev->csiphy_dev.ops->hw_start(&(cam_dev->csiphy_dev), cam_dev->csiphy_dev.mipi_data_lanes, cam_dev->csiphy_dev.mipi_link_freq);
	cam_dev->adap_dev.is_streaming = 1;

#ifdef DEBUG_TEST_MIPI_RESET
	debug_test_mipi_reset = 0;
#endif

	return 0;
}

static void cam_debug_dq_check_timeout(struct timer_list *t)
{
	struct cam_device *cam_dev = container_of(t, struct cam_device, dq_check_timer);

	pr_err("in, dump & reset mipi");

	cam_debug_mipi_dump( cam_dev);

	cam_debug_mipi_off(cam_dev);
	cam_debug_mipi_on(cam_dev);

	pr_err("leave ");

}

static int cam_v4l2_dev_register(struct cam_device *cam_dev)
{
	struct v4l2_device *v4l2_dev;

	v4l2_dev = &cam_dev->v4l2_dev;
	v4l2_dev->mdev = &cam_dev->media_dev;

	snprintf(v4l2_dev->name, sizeof(v4l2_dev->name), AML_CAM_DRIVER_NAME, cam_dev->index);

	return v4l2_device_register(cam_dev->dev, v4l2_dev);
}

static int cam_probe(struct platform_device *pdev)
{
	int rtn = 0;
	struct cam_device *cam_dev;
	struct device *dev = &pdev->dev;

	cam_dev = devm_kzalloc(dev, sizeof(*cam_dev), GFP_KERNEL);
	if (!cam_dev)
		return -ENOMEM;

	if (of_property_read_u32(dev->of_node, "index", &cam_dev->index)) {
		dev_err(dev, "Failed to read camera index\n");
		return -EINVAL;
	}

	// compatible for t7c camera dts and t7c aml isp driver.
	// t7c aml isp driver using index = 2 for csi_b;
	// this driver use index = 1 for csi_b;
	if (cam_dev->index > 1)
		cam_dev->index = 1;

	dev_set_drvdata(dev, cam_dev);
	cam_dev->dev = dev;

	cam_media_dev_init(cam_dev);

	rtn = cam_init_subdevices(cam_dev);
	if (rtn)
		goto error_return;

	rtn = cam_v4l2_dev_register(cam_dev);
	if (rtn) {
		dev_err(dev, "Failed to v4l2 register: %d\n", rtn);
		goto error_init_subdevs;
	}

	rtn = cam_subdevs_register(cam_dev);
	if (rtn) {
		dev_err(dev, "Failed to register entity\n");
		goto error_v4l2_dev;
	}

	rtn = cam_async_notifier_register(cam_dev);
	if (rtn) {
		dev_err(cam_dev->dev,
			"Failed to register subdev notifier(%d)\n", rtn);
		goto error_subdevs_register;
	}

	return rtn;

error_subdevs_register:
	cam_subdevs_unregister(cam_dev);

error_v4l2_dev:
	v4l2_device_unregister(&cam_dev->v4l2_dev);

error_init_subdevs:
	cam_deinit_subdevices(cam_dev);

error_return:
	return rtn;
}

static int cam_remove(struct platform_device *pdev)
{
	struct cam_device *cam_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	v4l2_async_notifier_unregister(&cam_dev->notifier);

	cam_subdevs_unregister(cam_dev);

	v4l2_device_unregister(&cam_dev->v4l2_dev);

	cam_deinit_subdevices(cam_dev);

	dev_info(cam_dev->dev, "cam-%u remove finished\n", cam_dev->index);
	devm_kfree(dev, cam_dev);
	return 0;
}
