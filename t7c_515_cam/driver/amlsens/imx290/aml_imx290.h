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

#define IMX290_STANDBY 0x3000
#define IMX290_REGHOLD 0x3001
#define IMX290_XMSTA 0x3002
#define IMX290_GAIN 0x3014
#define IMX290_EXPOSURE 0x3020
#define IMX290_SUB_ID 0xb201
#define IMX290_ID 0x0290
#define IMX290_SLAVE_ID 0x1A

#define IMX290_HMAX_LOW 0x301c
#define IMX290_HMAX_HIGH 0x301d

#define IMX290_FR_FDG_SEL 0x3009
#define IMX290_PHY_LANE_NUM 0x3407
#define IMX290_CSI_LANE_MODE 0x3443

#define IMX290_60HZ

struct imx290_regval {
	u16 reg;
	u8 val;
};

struct imx290_mode {
	u32 width;
	u32 height;
	u32 hmax;
	u32 link_freq_index;

	const struct imx290_regval *data;
	u32 data_size;
};

struct imx290 {
	int index;
	struct device *dev;
	struct clk *xclk;
	struct regmap *regmap;
	u8 nlanes;
	u8 bpp;
	u32 enWDRMode;

	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct v4l2_fwnode_endpoint ep;
	struct media_pad pad;
	struct v4l2_mbus_framefmt current_format;
	const struct imx290_mode *current_mode;

	struct sensor_gpio *gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *wdr;
	struct v4l2_ctrl *data_lanes;

	int status;
	struct mutex lock;

	int flag_60hz;
};

struct imx290_pixfmt {
	u32 code;
	u32 min_width;
	u32 max_width;
	u32 min_height;
	u32 max_height;
	u8 bpp;
};

static const struct imx290_pixfmt imx290_formats[] = {
	// 30hz
	{MEDIA_BUS_FMT_SRGGB10_1X10, 1280, 1920, 720, 1080, 10},
	{MEDIA_BUS_FMT_SRGGB12_1X12, 1280, 1920, 720, 1080, 12},
	// 60hz sdr
	{MEDIA_BUS_FMT_SGBRG10_1X10, 1280, 1920, 720, 1080, 10},
};

static const struct regmap_config imx290_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static const struct imx290_regval imx290_global_init_settings_60hz[] = {
	{0x3000, 0x01},
	{0x3002, 0x00},
	{0x3005, 0x00},
	{0x3007, 0x00},
	{0x3009, 0x01},
	{0x300a, 0x3c},
	{0x300f, 0x00},
	{0x3010, 0x21},
	{0x3012, 0x64},
	{0x3014, 0x00},
	{0x3016, 0x09},
	{0x3018, 0xDF},
	{0x3019, 0x04},
	{0x301c, 0xEC},
	{0x301d, 0x07},
	{0x3020, 0x02},
	{0x3021, 0x01},
	{0x3022, 0x00},
	{0x3046, 0x00},
	{0x304b, 0x0a},
	{0x3418, 0x49},
	{0x3419, 0x04},
	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x3070, 0x02},
	{0x3071, 0x11},
	{0x309b, 0x10},
	{0x309c, 0x22},
	{0x30a2, 0x02},
	{0x30a6, 0x20},
	{0x30a8, 0x20},
	{0x30aa, 0x20},
	{0x30ac, 0x20},
	{0x30b0, 0x43},
	{0x3106, 0x00},
	{0x3119, 0x9e},
	{0x311c, 0x1e},
	{0x311e, 0x08},
	{0x3128, 0x05},
	{0x3129, 0x1d},
	{0x313d, 0x83},
	{0x3150, 0x03},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x317c, 0x12},
	{0x317e, 0x00},
	{0x31ec, 0x37},
	{0x32b8, 0x50},
	{0x32b9, 0x10},
	{0x32ba, 0x00},
	{0x32bb, 0x04},
	{0x32c8, 0x50},
	{0x32c9, 0x10},
	{0x32ca, 0x00},
	{0x32cb, 0x04},
	{0x332c, 0xd3},
	{0x332d, 0x10},
	{0x332e, 0x0d},
	{0x3358, 0x06},
	{0x3359, 0xe1},
	{0x335a, 0x11},
	{0x3360, 0x1e},
	{0x3361, 0x61},
	{0x3362, 0x10},
	{0x33b0, 0x50},
	{0x33b2, 0x1a},
	{0x33b3, 0x04},
	{0x3405, 0x10},
	{0x3407, 0x03},
	{0x3414, 0x0a},
	{0x3415, 0x00},
	{0x3441, 0x0a},
	{0x3442, 0x0a},
	{0x3443, 0x03},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x57},
	{0x3447, 0x00},
	{0x3448, 0x37},
	{0x3449, 0x00},
	{0x344a, 0x1f},
	{0x344b, 0x00},
	{0x344c, 0x1f},
	{0x344d, 0x00},
	{0x344e, 0x1f},
	{0x344f, 0x00},
	{0x3450, 0x77},
	{0x3451, 0x00},
	{0x3452, 0x1f},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
	{0x3472, 0x9c},
	{0x3473, 0x07},
	{0x3480, 0x49},
	{0x3002, 0x00},

};

static const struct imx290_regval imx290_global_init_settings[] = {
	{0x3000, 0x01}, /* standby */

	{0x3005, 0x01}, // 0:10bit 1:12bit
	{0x3007, 0x00}, // full hd 1080p
	{0x3009, 0x12},
	{0x300a, 0xF0}, // black level
	{0x300B, 0x00},
	{0x300c, 0x00},
	{0x300F, 0x00},
	{0x3010, 0x21},
	{0x3012, 0x64},
	{0x3013, 0x00},
	{0x3014, 0x02}, // Gain
	{0x3016, 0x09},
	{0x3018, 0x85}, /* VMAX[7:0] */
	{0x3019, 0x04}, /* VMAX[15:8] */
	{0x301a, 0x00}, /* VMAX[16] */
	{0x301b, 0x00},
	{0x301c, 0x30}, /* HMAX[7:0] */
	{0x301d, 0x11}, /* HMAX[15:8] */
	{0x3020, 0x81}, // SHS1
	{0x3021, 0x01},
	{0x3022, 0x00}, // SHS1
	{0x3024, 0x00}, // SHS2
	{0x3025, 0x00}, // SHS2
	{0x3026, 0x00}, // SHS2
	{0x3030, 0x00}, // RHS1
	{0x3031, 0x00}, // RHS1
	{0x3032, 0x00}, // RHS1
	{0x3045, 0x01}, // DOL
	{0x3046, 0xe1}, // LANE CHN
	{0x304b, 0x00},
	//{0x3418, 0x 1},//Y_out size, tools should modify from B2 to 9C
	//{0x3419, 0x 1},//Y_out size

	{0x305C, 0x18},
	{0x305D, 0x03},
	{0x305E, 0x20},
	{0x305F, 0x01},

	{0x3070, 0x02}, // must set
	{0x3071, 0x11},
	{0x309B, 0x10},
	{0x309C, 0x22},
	{0x30A2, 0x02},
	{0x30A6, 0x20},
	{0x30A8, 0x20},
	{0x30AA, 0x20},
	{0x30AC, 0x20},
	{0x30B0, 0x43},

	{0x3106, 0x00}, // Need double confirm, H company 11h, 8/3th version
	{0x3119, 0x9e},
	{0x311c, 0x1e},
	{0x311e, 0x08},

	{0x3128, 0x05},

	{0x3129, 0x00},

	{0x313D, 0x83},
	{0x3150, 0x03},
	{0x315E, 0x1A}, // 1A:37.125MHz 1B:74.25MHz
	{0x3164, 0x1A}, // 1A:37.125MHz 1B:74.25MHz
	{0x317C, 0x00},
	{0x317E, 0x00},

	{0x31EC, 0x0E},

	{0x32B8, 0x50},
	{0x32B9, 0x10},
	{0x32BA, 0x00},
	{0x32BB, 0x04},
	{0x32C8, 0x50},
	{0x32C9, 0x10},
	{0x32CA, 0x00},
	{0x32CB, 0x04},

	{0x332C, 0xD3},
	{0x332D, 0x10},
	{0x332E, 0x0D},
	{0x3358, 0x06},
	{0x3359, 0xE1},
	{0x335A, 0x11},
	{0x3360, 0x1E},
	{0x3361, 0x61},
	{0x3362, 0x10},
	{0x33B0, 0x50},
	{0x33B2, 0x1A},
	{0x33B3, 0x04},

	{0x3405, 0x20},
	{0x3407, 0x03},
	{0x3414, 0x0A},
	{0x3415, 0x01},
	{0x3418, 0x49},
	{0x3419, 0x04},
	{0x3441, 0x0C},
	{0x3442, 0x0C},
	{0x3443, 0x03},
	{0x3444, 0x20}, // mclk :37.125M
	{0x3445, 0x25},

	{0x3446, 0x47}, // global timming
	{0x3447, 0x00},
	{0x3448, 0x1F},
	{0x3449, 0x00},
	{0x344A, 0x17},
	{0x344B, 0x00},
	{0x344C, 0x0F},
	{0x344D, 0x00},
	{0x344E, 0x17},
	{0x344F, 0x00},
	{0x3450, 0x47},
	{0x3451, 0x00},
	{0x3452, 0x0F},
	{0x3453, 0x00},
	{0x3454, 0x0F},
	{0x3455, 0x00},

	{0x3472, 0x9C},
	{0x3473, 0x07},
	{0x347B, 0x24}, // add
	{0x3480, 0x49},

	{0x3002, 0x00}, /* master mode start */

	{0x3049, 0x0A}, /* XVSOUTSEL XHSOUTSEL */

};

static struct imx290_regval dol_1080p_30fps_4lane_10bits[] = {
	{0x3000, 0x01}, /* standby */

	{0x3002, 0x00}, /* XTMSTA */

	{0x3005, 0x00},
	{0x3007, 0x00},
	{0x3009, 0x01},
	{0x300a, 0x3c},
	{0x300c, 0x11},
	{0x300f, 0x00},
	{0x3010, 0x21},
	{0x3012, 0x64},
	{0x3014, 0x02},
	{0x3016, 0x09},
	{0x3018, 0xC4}, // VMAX change from 0465 to 04C4
	{0x3019, 0x04}, // VMAX

	{0x301c, 0xEC}, //* HMAX */ change from 0898 to 07EC
	{0x301d, 0x07}, //* HMAX */
	{0x3020, 0x3c}, // SHS1
	{0x3021, 0x01}, // SHS1
	{0x3022, 0x00}, // SHS1
	{0x3024, 0xcb}, // SHS2
	{0x3025, 0x00}, // SHS2
	{0x3026, 0x00}, // SHS2
	{0x3030, 0xc9}, // RHS1
	{0x3031, 0x00}, // RHS1
	{0x3032, 0x00}, // RHS1
	{0x3045, 0x05}, // DOL
	{0x3046, 0x00}, // Datasheet should modify, Tools should modify
	{0x304b, 0x0a},
	{0x3418, 0x5e}, // Y_out size, tools should modify from B2 to 9C
	{0x3419, 0x09}, // Y_out size

	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},

	{0x3070, 0x02},
	{0x3071, 0x11},

	{0x309b, 0x10},
	{0x309c, 0x22},

	{0x30a2, 0x02},
	{0x30a6, 0x20},
	{0x30a8, 0x20},
	{0x30aa, 0x20},
	{0x30ac, 0x20},
	{0x30b0, 0x43},

	{0x3106, 0x11}, // Need double confirm, H company 11h, 8/3th version
	{0x3119, 0x9e},
	{0x311c, 0x1e},
	{0x311e, 0x08},

	{0x3128, 0x05},
	{0x3129, 0x1d},
	{0x313d, 0x83},
	{0x3150, 0x03},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x317c, 0x12},
	{0x317e, 0x00},
	{0x31ec, 0x37},

	{0x32b8, 0x50},
	{0x32b9, 0x10},
	{0x32ba, 0x00},
	{0x32bb, 0x04},

	{0x32c8, 0x50},
	{0x32c9, 0x10},
	{0x32ca, 0x00},
	{0x32cb, 0x04},

	{0x332c, 0xd3},
	{0x332d, 0x10},
	{0x332e, 0x0d},

	{0x3358, 0x06},
	{0x3359, 0xe1},
	{0x335a, 0x11},

	{0x3360, 0x1e},
	{0x3361, 0x61},
	{0x3362, 0x10},

	{0x33b0, 0x50},
	{0x33b2, 0x1a},
	{0x33b3, 0x04},

	{0x3405, 0x10},
	{0x3407, 0x03},
	{0x3414, 0x0a},
	{0x3415, 0x00},

	{0x3441, 0x0a},
	{0x3442, 0x0a},
	{0x3443, 0x03},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x57},
	{0x3447, 0x00},
	{0x3448, 0x37},
	{0x3449, 0x00},
	{0x344a, 0x1f},

	{0x344b, 0x00},
	{0x344c, 0x1f},
	{0x344d, 0x00},

	{0x344e, 0x1f},
	{0x344f, 0x00},
	{0x3450, 0x77},
	{0x3451, 0x00},

	{0x3452, 0x1f},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},

	{0x3472, 0xA0}, // Xout size from 079c to 07A0,8/3th's info
	{0x3473, 0x07},
	{0x347B, 0x23}, // add
	{0x3480, 0x49},

	{0x3002, 0x00}, /* master mode start */
};

static const struct imx290_regval imx290_1080p_settings[] = {
	/* mode settings */
	{0x3007, 0x00},
	{0x303a, 0x0c},
	{0x3414, 0x0a},
	{0x3472, 0x80},
	{0x3473, 0x07},
	{0x3418, 0x38}, // vmax
	{0x3419, 0x04}, // vmax
	{0x3012, 0x64},
	{0x3013, 0x00},
	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x3480, 0x49},
	/* data rate settings */
	//{ 0x3009, 0x01 },// fr fdg sel lane related 60/50 fps
	{0x3405, 0x10},
	{0x3446, 0x57},
	{0x3447, 0x00},
	{0x3448, 0x37},
	{0x3449, 0x00},
	{0x344a, 0x1f},
	{0x344b, 0x00},
	{0x344c, 0x1f},
	{0x344d, 0x00},
	{0x344e, 0x1f},
	{0x344f, 0x00},
	{0x3450, 0x77},
	{0x3451, 0x00},
	{0x3452, 0x1f},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
	//{ 0x301c, 0x98 },// hmax low
	//{ 0x301d, 0x08 },// hmax high
};

static const struct imx290_regval imx290_720p_settings[] = {
	/* mode settings */
	{0x3007, 0x10},
	{0x303a, 0x06},
	{0x3414, 0x04},
	{0x3472, 0x00},
	{0x3473, 0x05},
	{0x3418, 0xd0},
	{0x3419, 0x02},
	{0x3012, 0x64},
	{0x3013, 0x00},
	{0x305c, 0x20},
	{0x305d, 0x00},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x3480, 0x49},
	/* data rate settings */
	{0x3009, 0x01},
	{0x3405, 0x10},
	{0x3446, 0x4f},
	{0x3447, 0x00},
	{0x3448, 0x2f},
	{0x3449, 0x00},
	{0x344a, 0x17},
	{0x344b, 0x00},
	{0x344c, 0x17},
	{0x344d, 0x00},
	{0x344e, 0x17},
	{0x344f, 0x00},
	{0x3450, 0x57},
	{0x3451, 0x00},
	{0x3452, 0x17},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
	{0x301c, 0xe4},
	{0x301d, 0x0c},
};

static const struct imx290_regval imx290_10bit_settings[] = {
	{0x3005, 0x00},
	{0x3046, 0x00},
	{0x3129, 0x1d},
	{0x317c, 0x12},
	{0x31ec, 0x37},
	{0x3441, 0x0a},
	{0x3442, 0x0a},
	{0x300a, 0x3c},
	{0x300b, 0x00},
};

static const struct imx290_regval imx290_12bit_settings[] = {
	{0x3005, 0x01},
	{0x3046, 0x01},
	{0x3129, 0x00},
	{0x317c, 0x00},
	{0x31ec, 0x0e},
	{0x3441, 0x0c},
	{0x3442, 0x0c},
	{0x300a, 0xf0},
	{0x300b, 0x00},
};

extern int imx290_init(struct i2c_client *client, void *sdrv);
extern int imx290_deinit(struct i2c_client *client);
extern int imx290_sensor_id(struct i2c_client *client);
extern int imx290_power_on(struct device *dev, struct sensor_gpio *gpio);
extern int imx290_power_off(struct device *dev, struct sensor_gpio *gpio);
extern int imx290_power_suspend(struct device *dev);
extern int imx290_power_resume(struct device *dev);
