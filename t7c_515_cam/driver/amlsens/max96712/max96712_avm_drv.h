// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2023 Amlogic, Inc. All rights reserved.
 */

#ifndef MAX96712_AVM_DRV_H

#define MAX96712_AVM_DRV_H

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>

#include "sensor_drv.h"

extern int max96712_avm_init(struct i2c_client *client, void *sdrv);
extern int max96712_avm_deinit(struct i2c_client *client);
extern int max96712_avm_sensor_id(struct i2c_client *client);
extern int max96712_avm_power_on(struct device *dev, struct sensor_gpio *gpio);
extern int max96712_avm_power_off(struct device *dev, struct sensor_gpio *gpio);
extern int max96712_avm_power_suspend(struct device *dev);
extern int max96712_avm_power_resume(struct device *dev);

#endif
