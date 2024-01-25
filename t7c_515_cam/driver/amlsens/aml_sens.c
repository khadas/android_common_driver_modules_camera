// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2023 Amlogic, Inc. All rights reserved.
 */
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
#include <linux/debugfs.h>

#include "sensor_drv.h"
#include "i2c_api.h"

struct amlsens *g_sensor[8];

static struct amlsens * sensor_get_ptr(struct i2c_client * client)
{
	int i = 0;
	for (i = 0; i < sizeof(g_sensor); i++) {
		if (g_sensor[i]->client == client)
			return g_sensor[i];
	}

	return NULL;
}

static int sensor_set_ptr(struct amlsens * sensor)
{
	int i = 0;
	for (i = 0; i < sizeof(g_sensor); i++) {
		if (g_sensor[i] == NULL) {
			g_sensor[i] = sensor;
			sensor->index = i;
			return 0;
		}
	}

	return -1;
}

static int sensor_id_detect(struct amlsens *sensor)
{
	int i = 0, ret = -1;
	struct sensor_subdev *subdev = NULL;

	for (i = 0; i < ARRAY_SIZE(aml_sensors); i++) {
		subdev = aml_sensors[i];
		if (subdev == NULL)
			return ret;

		subdev->sensor_power_on(sensor->dev, &sensor->gpio);

		ret = subdev->sensor_get_id(sensor->client);
		if (ret == 0) {
			sensor->sd_sdrv = subdev;
			return ret;
		}

		subdev->sensor_power_off(sensor->dev, &sensor->gpio);
	}

	return ret;
}

static int sensor_power_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct amlsens *sensor = sensor_get_ptr(client);

	sensor->sd_sdrv->sensor_power_suspend(dev);

	return 0;
}

static int sensor_power_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct amlsens *sensor = sensor_get_ptr(client);

	sensor->sd_sdrv->sensor_power_resume(dev);

	return 0;
}

static const struct dev_pm_ops sensor_pm_ops = {
	SET_RUNTIME_PM_OPS(sensor_power_suspend, sensor_power_resume, NULL)};

static int sensor_parse_power(struct amlsens *sensor)
{
	int rtn = 0;

	sensor->gpio.rst_gpio = devm_gpiod_get_optional(sensor->dev,
												"reset",
												 GPIOD_OUT_LOW);
	if (IS_ERR(sensor->gpio.rst_gpio))
	{
		rtn = PTR_ERR(sensor->gpio.rst_gpio);
		dev_err(sensor->dev, "Cannot get reset gpio: %d\n", rtn);
		goto err_return;
	}

	sensor->gpio.pwdn_gpio = devm_gpiod_get_optional(sensor->dev,
												"pwdn",
												GPIOD_OUT_LOW);
	if (IS_ERR(sensor->gpio.pwdn_gpio)) {
		rtn = PTR_ERR(sensor->gpio.pwdn_gpio);
		dev_err(sensor->dev, "Cannot get pwdn gpio: %d\n", rtn);
		return 0;
	}

	sensor->gpio.power_gpio = devm_gpiod_get_optional(sensor->dev,
												"pwr",
												GPIOD_OUT_LOW);
	if (IS_ERR(sensor->gpio.power_gpio)) {
		rtn = PTR_ERR(sensor->gpio.power_gpio);
		dev_info(sensor->dev, "Cannot get power gpio: %d\n", rtn);
		return 0;
	}

err_return:

	return rtn;
}

static int sensor_parse_endpoint(struct amlsens *sensor)
{
	int rtn = 0;
	struct fwnode_handle *endpoint = NULL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(sensor->dev), NULL);
	if (!endpoint) {
		dev_err(sensor->dev, "Endpoint node not found\n");
		return -EINVAL;
	}

	rtn = v4l2_fwnode_endpoint_alloc_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (rtn)
	{
		dev_err(sensor->dev, "Parsing endpoint node failed\n");
		rtn = -EINVAL;
		goto err_return;
	}

	/* Only CSI2 is supported for now */
	if (sensor->ep.bus_type != V4L2_MBUS_CSI2_DPHY)
	{
		dev_err(sensor->dev, "Unsupported bus type, should be CSI2\n");
		rtn = -EINVAL;
		goto err_free;
	}

	sensor->nlanes = sensor->ep.bus.mipi_csi2.num_data_lanes;
	if (sensor->nlanes != 2 && sensor->nlanes != 4)
	{
		dev_err(sensor->dev, "Invalid data lanes: %d\n", sensor->nlanes);
		rtn = -EINVAL;
		goto err_free;
	}
	dev_info(sensor->dev, "Using %u data lanes\n", sensor->nlanes);

	if (!sensor->ep.nr_of_link_frequencies)
	{
		dev_err(sensor->dev, "link-frequency property not found in DT\n");
		rtn = -EINVAL;
		goto err_free;
	}

	return rtn;

err_free:
	v4l2_fwnode_endpoint_free(&sensor->ep);
err_return:
	return rtn;
}

static const char *sensor_reg_usage_str = {
	"Usage:\n"
	"echo r addr(H) > /sys/kernel/debug/sensor0/reg;\n"
	"echo w addr(H) value(H) > /sys/kernel/debug/sensor0/reg;\n"
};

static void parse_param(char *buf_orig, char **parm)
{
	char *ps, *token;
	unsigned int n = 0;
	char delim1[3] = " ";
	char delim2[2] = "\n";
	ps = buf_orig;
	strcat(delim1, delim2);
	while (1) {
		token = strsep(&ps, delim1);
		if (token == NULL)
			break;
		if (*token == '\0')
			continue;
		parm[n++] = token;
		pr_debug("%s %d of parm : %s \n", __func__, n, token);
	}
}

static ssize_t sensor_file_read(struct file *f, char *buffer, size_t len, loff_t *offset)
{
	return simple_read_from_buffer(buffer, len, offset, sensor_reg_usage_str, strlen(sensor_reg_usage_str));
}

static ssize_t sensor_file_write(struct file *f, const char *buffer, size_t len, loff_t *offset)
{
	ssize_t ret;
	char data[PAGE_SIZE];
	char *parm[8] = {NULL};
	long val = 0;
	u16 reg_addr;
	u8 reg_val;
	int i2c_ret;
	struct path f_path = f->f_path;
	struct dentry *f_dentry = f_path.dentry;
	struct dentry *p_dentry = f_dentry->d_parent;

	ret = simple_write_to_buffer(data, PAGE_SIZE, offset, buffer, len);
	if (ret < 0)
		return ret;
	pr_debug("%s: data len: %d, write data: %s \n", __func__, ret, data);
	parse_param(data, (char **)&parm);
	if (!parm[0]) {
		ret = -EINVAL;
		goto Err;
	}
	if (!strcmp(parm[0], "w")) {
		if (!parm[1] || (kstrtoul(parm[1], 16, &val) < 0)) {
			ret = -EINVAL;
			goto Err;
		}
		reg_addr = val;
		if (!parm[2] || (kstrtoul(parm[2], 16, &val) < 0)) {
			ret = -EINVAL;
			goto Err;
		}
		reg_val = val;
		pr_debug("%s reg addr 0x%x, reg val 0x%x \n", __func__, reg_addr, reg_val);
		if (g_sensor[0] && strstr(p_dentry->d_iname, "sensor0")) {
			pr_err("%s sensor0 write name %s", __func__, p_dentry->d_iname);
			i2c_ret = i2c_write_a16d8(g_sensor[0]->client, g_sensor[0]->client->addr, reg_addr, reg_val);
			if (i2c_ret) {
				pr_err("%s i2c write fail i2c addr 0x%x \n", __func__, g_sensor[0]->client->addr);
				ret = -EINVAL;
				goto Err;
			} else {
				pr_err("SENSOR WRITE[0x%x]=0x%x i2c addr 0x%x \n", reg_addr, reg_val, g_sensor[0]->client->addr);
			}
		} else if (g_sensor[1] && strstr(p_dentry->d_iname, "sensor1")) {
			pr_err("%s sensor1 write name %s", __func__, p_dentry->d_iname);
			i2c_ret = i2c_write_a16d8(g_sensor[1]->client, g_sensor[1]->client->addr, reg_addr, reg_val);
			if (i2c_ret) {
				pr_err("%s i2c write fail i2c addr 0x%x \n", __func__, g_sensor[1]->client->addr);
				ret = -EINVAL;
				goto Err;
			} else {
				pr_err("SENSOR WRITE[0x%x]=0x%x i2c addr 0x%x \n", reg_addr, reg_val, g_sensor[1]->client->addr);
			}
		} else {
			pr_err("%s need init sensor first \n", __func__);
			goto Err;
		}
	} else if (!strcmp(parm[0], "r")) {
		if (!parm[1] || (kstrtoul(parm[1], 16, &val) < 0)) {
			ret = -EINVAL;
			goto Err;
		}
		reg_addr = val;
		pr_debug("%s reg addr 0x%x \n", __func__, reg_addr);
		if (g_sensor[0] && strstr(p_dentry->d_iname, "sensor0")) {
			pr_err("%s sensor0 read name %s", __func__, p_dentry->d_iname);
			i2c_ret = i2c_read_a16d8(g_sensor[0]->client, g_sensor[0]->client->addr, reg_addr, &reg_val);
			if (i2c_ret) {
				pr_err("%s i2c read fail i2c addr 0x%x \n", __func__, g_sensor[0]->client->addr);
				ret = -EINVAL;
				goto Err;
			} else {
				pr_err("SENSOR READ[0x%x]=0x%x i2c addr 0x%x \n", reg_addr, reg_val, g_sensor[0]->client->addr);
			}
		} else if (g_sensor[1] && strstr(p_dentry->d_iname, "sensor1")) {
			pr_err("%s sensor1 read name %s", __func__, p_dentry->d_iname);
			i2c_ret = i2c_read_a16d8(g_sensor[1]->client, g_sensor[1]->client->addr, reg_addr, &reg_val);
			if (i2c_ret) {
				pr_err("%s i2c read fail i2c addr 0x%x \n", __func__, g_sensor[1]->client->addr);
				ret = -EINVAL;
				goto Err;
			} else {
				pr_err("SENSOR READ[0x%x]=0x%x i2c addr 0x%x \n", reg_addr, reg_val, g_sensor[1]->client->addr);
			}
		} else {
			pr_err("%s need init sensor first \n", __func__);
			goto Err;
		}
	} else {
		pr_err("unsupprt cmd!\n");
	}

Err:
	return ret;
}

static const struct file_operations data_file_fops = {
	.owner = THIS_MODULE,
	.write = sensor_file_write,
	.read = sensor_file_read,
};

static int settle_init_debugfs(struct amlsens *sensor) {
	char sensor_dir[256];
	snprintf(sensor_dir, 256, "sensor%d", sensor->index);
	sensor->debugfs_dir = debugfs_create_dir(sensor_dir, NULL);
	if (sensor->debugfs_dir == NULL) {
		printk("%s: create debugfs dir failed\n", __func__);
		return -1;
	}
	sensor->debugfs_file = debugfs_create_file("reg", S_IRUGO | S_IWUSR, sensor->debugfs_dir, NULL, &data_file_fops);
	if (sensor->debugfs_file == NULL) {
		printk("%s: create debugfs file failed\n", __func__);
		return -1;
	}
	return 0;
}

static int sensor_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct amlsens *sensor;
	int ret = -EINVAL;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	sensor->client = client;

	ret = sensor_parse_endpoint(sensor);
	if (ret) {
		dev_err(sensor->dev, "Error parse endpoint\n");
		goto return_err;
	}

	ret = sensor_parse_power(sensor);
	if (ret) {
		dev_err(sensor->dev, "Error parse power ctrls\n");
		goto free_err;
	}

	ret = sensor_id_detect(sensor);
	if (ret) {
		dev_err(sensor->dev, "None sensor detect\n");
		goto free_err;
	}

	/* Power on the device to match runtime PM state below */
	dev_info(dev, "bef get id. pwdn - 0, reset - 1\n");

	sensor->sd_sdrv->sensor_init(client, (void *)sensor);

	v4l2_fwnode_endpoint_free(&sensor->ep);

	sensor_set_ptr(sensor);
	settle_init_debugfs(sensor);

	return 0;

free_err:
	v4l2_fwnode_endpoint_free(&sensor->ep);
return_err:
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	struct amlsens *sensor = sensor_get_ptr(client);

	if (sensor) {
		debugfs_remove_recursive(sensor->debugfs_dir);
		sensor->sd_sdrv->sensor_deinit(client);
		sensor = NULL;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_AMLOGIC_FREERTOS)

enum {
	SENSOR_A = 0,
	SENSOR_B,
	SENSOR_MAX_NUM
};

static const struct of_device_id sensor_a_of_match[] = {
	{.compatible = "amlogic, sensor-csi-a"},
	{/* sentinel */}};
MODULE_DEVICE_TABLE(of, sensor_a_of_match);

static struct i2c_driver sensor_a_driver = {
	.probe_new = sensor_probe,
	.remove = sensor_remove,
	.driver = {
		.name = "amlsens-csi-a",
		.pm = &sensor_pm_ops,
		.of_match_table = of_match_ptr(sensor_a_of_match),
	},
};

static const struct of_device_id sensor_b_of_match[] = {
	{.compatible = "amlogic, sensor-csi-b"},
	{/* sentinel */}};
MODULE_DEVICE_TABLE(of, sensor_b_of_match);

static struct i2c_driver sensor_b_driver = {
	.probe_new = sensor_probe,
	.remove = sensor_remove,
	.driver = {
		.name = "amlsens-csi-b",
		.pm = &sensor_pm_ops,
		.of_match_table = of_match_ptr(sensor_b_of_match),
	},
};

static struct i2c_driver *sensor_driver_ptr[SENSOR_MAX_NUM] = {
	&sensor_a_driver,
	&sensor_b_driver
};

static int sensor_driver_registered[SENSOR_MAX_NUM] = {0,0}; // 0 not registered. 1 registered.

int senosor_a_after_rtos = 0;
module_param(senosor_a_after_rtos, int, 0664);

int senosor_b_after_rtos = 0;
module_param(senosor_b_after_rtos, int, 0664);

extern int register_freertos_notifier(struct notifier_block *nb);
extern int unregister_freertos_notifier(struct notifier_block *nb);

static int rtos_driver_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	int32_t rc = 0;
	int i = 0;
	pr_info("event = %d, amlsens i2c add drv\n", event);
	for (i = 0; i < SENSOR_MAX_NUM; ++i) {
		if (sensor_driver_registered[i] == 0) {
			rc = i2c_add_driver(sensor_driver_ptr[i]);
			sensor_driver_registered[i] = 1;
		}
	}
	return rc;
}

static struct notifier_block camera_notifier =
{
	.notifier_call = rtos_driver_event,
};

static int __init amlsens_drv_init(void)
{
	int err;

	pr_info("amlsens driver init\n");
	if (senosor_a_after_rtos == 0) {
		// not after rtos, just add driver now.
		pr_info("add sensor a driver\n");
		i2c_add_driver(sensor_driver_ptr[SENSOR_A]);
		sensor_driver_registered[SENSOR_A] = 1;
	}

	if (senosor_b_after_rtos == 0) {
		// not after rtos, just add driver now.
		pr_info("add sensor b driver\n");
		i2c_add_driver(sensor_driver_ptr[SENSOR_B]);
		sensor_driver_registered[SENSOR_B] = 1;
	}

	err = register_freertos_notifier(&camera_notifier);
	if (err) {
		pr_info("amlsens register_freertos_notifier error\n");
		return -1;
	}
	pr_info("amlsens register_freertos_notifier completed\n");

	return err;
}

static void __exit  amlsens_drv_exit(void)
{
	int i = 0;
	unregister_freertos_notifier(&camera_notifier);

	for (i = 0; i < SENSOR_MAX_NUM; ++i) {
		if (sensor_driver_registered[i] == 1) {
			i2c_del_driver(sensor_driver_ptr[i]);
			sensor_driver_registered[i] = 0;
		}
	}
}

module_init(amlsens_drv_init);
module_exit(amlsens_drv_exit);

#else

static const struct of_device_id sensor_of_match[] = {
	{.compatible = "amlogic, sensor"},
	{/* sentinel */}};
MODULE_DEVICE_TABLE(of, sensor_of_match);

static struct i2c_driver sensor_i2c_driver = {
	.probe_new = sensor_probe,
	.remove = sensor_remove,
	.driver = {
		.name = "amlsens",
		.pm = &sensor_pm_ops,
		.of_match_table = of_match_ptr(sensor_of_match),
	},
};

module_i2c_driver(sensor_i2c_driver);

#endif

MODULE_DESCRIPTION("Amlogic Image Sensor Driver");
MODULE_AUTHOR("Amlogic Inc.");
MODULE_LICENSE("GPL v2");
