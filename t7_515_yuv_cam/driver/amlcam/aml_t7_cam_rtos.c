/*
 * isp.c
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */



#include "aml_t7_cam.c"


static int  driver_registered = 0; // 0 not registered. 1 registered.

static const struct of_device_id cam_of_table[] = {
	{.compatible = "amlogic, yuvcamera-rtos"},
	{.compatible = "amlogic, yuvcamera"},
	{ },
};

MODULE_DEVICE_TABLE(of, cam_of_table);

static struct platform_driver cam_driver = {
	.probe = cam_probe,
	.remove = cam_remove,
	.driver = {
		.name = "aml_camera_rtos",
		.of_match_table = cam_of_table,
	},
};

extern int register_freertos_notifier(struct notifier_block *nb);
extern int unregister_freertos_notifier(struct notifier_block *nb);

static int rtos_driver_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	int32_t rc = 0;
	printk("event = %d, amlcam platform add drv\n", event);

	rc = platform_driver_register(&(cam_driver) );
	driver_registered = 1;
	return rc;
}

static struct notifier_block camera_notifier =
{
		.notifier_call = rtos_driver_event,
};

static int __init amlcam_drv_init(void)
{
	int err;
	printk("amlcam driver register notifier\n");

	err = register_freertos_notifier(&camera_notifier);
	if (err)
	{
		printk("amlcam register_freertos_notifier error\n");
		return -1;
	}
	printk("amlcam register_freertos_notifier completed\n");

	return err;

}

static void __exit amlcam_drv_exit(void)
{
	unregister_freertos_notifier(&camera_notifier);
	if (driver_registered) {
		platform_driver_unregister(&(cam_driver) );
	}
}

module_init(amlcam_drv_init);
module_exit(amlcam_drv_exit);

MODULE_AUTHOR("Ethan Cao");
MODULE_DESCRIPTION("Amlogic Camera Driver");
MODULE_LICENSE("Dual BSD/GPL");

