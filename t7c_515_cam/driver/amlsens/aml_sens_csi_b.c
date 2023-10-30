// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2023 Amlogic, Inc. All rights reserved.
 */

#include "aml_sens_base.c"

static const struct of_device_id sensor_of_match[] = {
	{.compatible = "amlogic, sensor-csi-b"},
	{/* sentinel */}};
MODULE_DEVICE_TABLE(of, sensor_of_match);

static struct i2c_driver sensor_i2c_driver = {
	.probe_new = sensor_probe,
	.remove = sensor_remove,
	.driver = {
		.name = "amlsens-csi-b",
		.pm = &sensor_pm_ops,
		.of_match_table = of_match_ptr(sensor_of_match),
	},
};

module_i2c_driver(sensor_i2c_driver);

MODULE_DESCRIPTION("Amlogic Image Sensor Driver");
MODULE_AUTHOR("Amlogic Inc.");
MODULE_LICENSE("GPL v2");
