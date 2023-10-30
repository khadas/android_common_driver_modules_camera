/*
 * isp.c
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "aml_t7_cam.c"

static const struct of_device_id cam_of_table[] = {
	{.compatible = "amlogic, yuvcamera"},
	{.compatible = "amlogic, yuvcamera-droid"},
	{ },
};

MODULE_DEVICE_TABLE(of, cam_of_table);

static struct platform_driver cam_driver = {
	.probe = cam_probe,
	.remove = cam_remove,
	.driver = {
		.name = "aml_camera_droid",
		.of_match_table = cam_of_table,
	},
};

module_platform_driver(cam_driver);

MODULE_AUTHOR("Ethan Cao");
MODULE_DESCRIPTION("Amlogic Camera Driver");
MODULE_LICENSE("Dual BSD/GPL");

