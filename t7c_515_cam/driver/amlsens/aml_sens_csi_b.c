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

#if  IS_ENABLED(CONFIG_AMLOGIC_FREERTOS)

static int  driver_registered = 0; // 0 not registered. 1 registered.

extern int register_freertos_notifier(struct notifier_block *nb);
extern int unregister_freertos_notifier(struct notifier_block *nb);

static int rtos_driver_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	int32_t rc = 0;
	printk("event = %d, amlsens-b i2c add drv\n", event);

	rc = i2c_add_driver(&sensor_i2c_driver);
	driver_registered = 1;
	return rc;
}

static struct notifier_block camera_notifier =
{
		.notifier_call = rtos_driver_event,
};

static int __init amlsens_b_drv_init(void)
{
	int err;
	printk(" amlsens-b driver register notifier\n");

	err = register_freertos_notifier(&camera_notifier);
	if (err)
	{
		printk(" amlsens-b register_freertos_notifier error\n");
		return -1;
	}
	printk(" amlsens-b register_freertos_notifier completed\n");

	return err;

}

static void __exit  amlsens_b_drv_exit(void)
{
	unregister_freertos_notifier(&camera_notifier);
	if (driver_registered) {
		i2c_del_driver(&sensor_i2c_driver);
	}
}

module_init(amlsens_b_drv_init);
module_exit(amlsens_b_drv_exit);

#else

module_i2c_driver(sensor_i2c_driver);

#endif

MODULE_DESCRIPTION("Amlogic Image Sensor Driver");
MODULE_AUTHOR("Amlogic Inc.");
MODULE_LICENSE("GPL v2");
