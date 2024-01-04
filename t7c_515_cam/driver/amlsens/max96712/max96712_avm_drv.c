// SPDX-License-Identifier: GPL-2.0
/*
 * maxim max96712 quad GMSL2 deserializer driver
*/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt)  "[max96712]:%s:%d: " fmt, __func__, __LINE__

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/of_gpio.h>

#include "i2c_api.h"
#include "../../amlcam/cam_common/aml_misc.h"


#include "max96712_avm_drv.h"

#define AML_SENSOR_NAME  "max96712-%u"

#define MAX96712_SLAVE_ID 0x29

#define  MAX96712_ID   0x20
#define  MAX96722_ID   0xA1
#define  MAX96705_ID   0b01000001

#define MAX96712_DPLL_FREQ 1500


// 30 fps(first) or 25fps. uncomment following line. otherwise 15fps.
//#define SENSOR_30FPS_FRAMESYNC  1
#define SENSOR_25FPS_FRAMESYNC  1

#define  CONFIG_DEV_IN_DRV 1

#define  DEFAULT_OX01F10_I2C_ADDR    0x36  // 8 bits 0x6c
#define  DEFAULT_MAX96705_I2C_ADDR   0X40  // 8 bits 0x80
#define  DEFAULT_MAX96712_I2C_ADDR   0X29    // 8 bits 0x52

// config max96705, let it transfer packets from OX01F10_I2C_ADDR to DEFAULT_OX01F10_I2C_ADDR
#define  OX01F10_I2C_ADDR    DEFAULT_OX01F10_I2C_ADDR
#define  MAX96705_I2C_ADDR   DEFAULT_MAX96705_I2C_ADDR
#define  MAX96712_I2C_ADDR   DEFAULT_MAX96712_I2C_ADDR

#define  MAX_SENSOR_NUM      4

enum max96712_pattern {
    MAX96712_PATTERN_CHECKERBOARD = 0,
    MAX96712_PATTERN_GRADIENT,
};


enum max96712_link {
    LINK_A = 0,
    LINK_B,
    LINK_C,
    LINK_D
};

enum max96712_video_pipe {
    VIDEO_PIPE_0 = 0,
    VIDEO_PIPE_1,
    VIDEO_PIPE_2,
    VIDEO_PIPE_3
};

enum max96712_mipi_mode {
    MIPI_MODE_2X4 = 0,
    MIPI_MODE_4X2,
    MIPI_MODE_1X4A_2X2,
    MIPI_MODE_1X4B_2X2
};

struct max96712_regval {
    u16 reg;
    u8 val;
};

struct max96712_mode {
    u32 width;
    u32 height;

    const struct max96712_regval *data;
    u32 data_size;
};

struct max96712_priv;

struct max96705_dev {
    struct i2c_client *client;
    struct regmap *regmap;

    int index;
    int i2c_addr;
    int i2c_reg_addr_bytes;
    int i2c_reg_value_bytes;
    struct max96712_priv  * ref_des;
};

struct ox01f10_dev {
    struct i2c_client *client;
    struct regmap *regmap;

    int index;
    int i2c_addr;
    int i2c_reg_addr_bytes;
    int i2c_reg_value_bytes;
    struct max96712_priv  * ref_des;
};

struct max96712_priv {
    int index;

    struct device *dev;
    struct i2c_client *client;
    struct regmap *regmap;

    struct v4l2_subdev sd;
    struct media_pad pads[1];

    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *test_pattern;
    struct v4l2_ctrl *data_lanes;
    u8 nlanes;

    enum max96712_pattern pattern;

    struct v4l2_mbus_framefmt current_format;
    struct max96712_mode     *current_mode;

    struct mutex lock;

    struct max96705_dev  ser_devs[MAX_SENSOR_NUM];
    struct ox01f10_dev   sensor_devs[MAX_SENSOR_NUM];
};

struct max96712_pixfmt {
    u32 code;
    u32 min_width;
    u32 max_width;
    u32 min_height;
    u32 max_height;
    u8 bpp;
};


static struct max96712_pixfmt max96712_formats[] = {
    { MEDIA_BUS_FMT_UYVY8_2X8, 1280, 1920, 800, 3200, 16 }
};

static const s64 max96712_link_freq[] = {
    1500000000
};

static const struct max96712_regval max96712_test_pattern_settings[] = {
};


static const struct max96712_regval max96712_default_settings[] = {
};

/* Mode configs */
static struct max96712_mode max96712_modes[] = {
    {
        .width = 1280,
        .height = 800,

        .data = max96712_default_settings,
        .data_size = ARRAY_SIZE(max96712_default_settings),
    },
    {
        .width = 1280,
        .height = 1600,
        .data = max96712_default_settings,
        .data_size = ARRAY_SIZE(max96712_default_settings),
    },
    {
        .width = 1280,
        .height = 2400,

        .data = max96712_default_settings,
        .data_size = ARRAY_SIZE(max96712_default_settings),
    },
    {
        .width = 1280,
        .height = 3200,
        .data = max96712_default_settings,
        .data_size = ARRAY_SIZE(max96712_default_settings),
    }
};

static struct max96712_priv  g_max96712_dev;

static inline struct max96712_priv *to_max96712_dev(struct v4l2_subdev *sd)
{
    return container_of(sd, struct max96712_priv, sd);
}

static int max96712_read(struct max96712_priv *priv, int reg)
{
    int ret, val;

    ret = regmap_read(priv->regmap, reg, &val);
    if (ret) {
        dev_err(&priv->client->dev, "read 0x%04x failed, ret = %x\n", reg, ret);
        return ret;
    }

    return val;
}

static int max96712_write(struct max96712_priv *priv, unsigned int reg, u8 val)
{
    int ret;
    int max_retry = 5;

    do {
        ret = regmap_write(priv->regmap, reg, val);
        if (ret == 0)
            return ret;
        max_retry--;
        msleep(2);
    } while (max_retry);

    dev_err(&priv->client->dev, "write 0x%04x failed\n", reg);
    return ret;
}

static int max96712_update_bits(struct max96712_priv *priv, unsigned int reg,
                u8 mask, u8 val)
{
    int ret;
    int max_retry = 5;

    do {
        ret = regmap_update_bits(priv->regmap, reg, mask, val);
        if (ret == 0)
            return ret;
        max_retry--;
        msleep(2);
    } while (max_retry);

    dev_err(&priv->client->dev, "update 0x%04x failed\n", reg);
    return ret;
}

static int max96712_write_bulk(struct max96712_priv *priv, unsigned int reg,
                   const void *val, size_t val_count)
{
    int ret;

    ret = regmap_bulk_write(priv->regmap, reg, val, val_count);
    if (ret)
        dev_err(&priv->client->dev, "bulk write 0x%04x failed\n", reg);

    return ret;
}

static int max96712_write_bulk_value(struct max96712_priv *priv,
                     unsigned int reg, unsigned int val,
                     size_t val_count)
{
    unsigned int i;
    u8 values[4];

    for (i = 1; i <= val_count; i++)
        values[i - 1] = (val >> ((val_count - i) * 8)) & 0xff;

    return max96712_write_bulk(priv, reg, &values, val_count);
}

static int max96705_read_reg(struct max96705_dev *max96705, u16 addr, u8 *value)
{
    int msg_count = 0;
    int i, ret;

    u8 buf[4];
    struct i2c_msg msgs[2];

    buf[0] = addr & 0xff;

    msgs[0].addr  = max96705->i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = buf;

    msgs[1].addr  = max96705->i2c_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = buf;

    msg_count = sizeof(msgs) / sizeof(msgs[0]);
    for (i = 0; i < 5; i++) {
        ret = i2c_transfer(max96705->client->adapter, msgs, msg_count);
        if (ret == msg_count)
            break;
        msleep(20);
    }

    if (ret != msg_count)
        pr_err("I2C read with i2c transfer failed for addr: %x, ret %d\n", addr, ret);

    *value = buf[0] & 0xff;
    return ((ret != msg_count) ? -1 : 0);
}

static int max96705_write_reg(struct max96705_dev *max96705, u16 addr, u8 value)
{
    int msg_count = 0;
    int i, ret;

    u8 buf[3];
    struct i2c_msg msgs[1];

    buf[0] = addr & 0xff;
    buf[1] = value & 0xff;

    msgs[0].addr = max96705->i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = buf;

    msg_count = sizeof(msgs) / sizeof(msgs[0]);
    for (i = 0; i < 5; i++) {
        ret = i2c_transfer(max96705->client->adapter, msgs, msg_count);
        if (ret == msg_count) {
            break;
        }
        msleep(20);
    }

    if (ret != msg_count)
        pr_err("I2C write failed for addr: %x, ret %d\n", addr, ret);

    return ((ret != msg_count) ? -1 : 0);
}

static int max96705_get_id(struct max96705_dev *max96705)
{
    u8 val = 0;
    max96705_read_reg(max96705, 0x1e, &val);
    return val;
}

static int max96712_gmsl_2_link_lock_state(struct max96712_priv *priv, int link_idx)
{
    int reg_val = 0x00;
    //GMSL2:  0x001a  0x000a  0x000b  0x000c bit 3  , indicates link A B C D locked states;
    int reg_addr[] = {
        0x001a,0x000a,0x000b,0x000c
    };
    reg_val = max96712_read(priv, reg_addr[link_idx] );
    return reg_val & 0x08;
}

static int max96712_gmsl_1_link_lock_state(struct max96712_priv *priv, int link_idx)
{
    int reg_val = 0x00;

    //gmsl1 mode:  0x0bcb  0x0ccb  0x0dcb  0x0ecb  bit 0, indicates link A B C D locked state
    int reg_addr[] = {
        0x0bcb,0x0ccb,0x0dcb,0x0ecb
    };
    reg_val = max96712_read(priv, reg_addr[link_idx] );
    return reg_val & 0x01;

}


static int max96712_gmsl_2_video_lock_state(struct max96712_priv *priv, int video_pipe)
{
    int reg_val = 0x00;

    //  0x01dc 0x01fc 0x021c 0x023c 0x025c 0x027c 0x029c or 0x028c bit 0
    int reg_addr[] = {
        0x01dc,0x01fc,0x021c,0x023c,0x025c,0x027c,0x029c,0x028c
    };
    reg_val = max96712_read(priv, reg_addr[video_pipe] );
    return reg_val & 0x01;
}

static int max96712_gmsl_1_video_lock_state(struct max96712_priv *priv, int video_pipe)
{
    // video_lock in gmsl1 mode is equivalent to serial link lock;
    return max96712_gmsl_1_link_lock_state( priv, video_pipe);
}

static void max96712_reset(struct max96712_priv *priv)
{
    max96712_update_bits(priv, 0x13, 0x40, 0x40);
    msleep(20);
}

static void max96712_mipi_enable(struct max96712_priv *priv, bool enable)
{
    dev_info(&priv->client->dev, "%s in, enable %d ", __func__, enable);

    if (enable) {
        max96712_update_bits(priv, 0x40b, 0x02, 0x02); // bit 1 csi_out_en = 1;
        max96712_update_bits(priv, 0x8a0, 0x80, 0x80); // bit 7 force_csi_out_en = 1
    } else {
        max96712_update_bits(priv, 0x8a0, 0x80, 0x00); // bit 7 bit 7 force_csi_out_en = 0
        max96712_update_bits(priv, 0x40b, 0x02, 0x00); // bit 1 csi_out_en = 0
    }
}


static void max96712_mipi_dphy_4x2(struct max96712_priv *priv)
{
    dev_info(&priv->client->dev, "%s in ", __func__);

    // Select 4x2 mode
    max96712_write(priv, 0x8a0, 0b00000001);

    // Configure lane mapping for PHY0 and PHY1
    // ob0100 0100

    // controller 0 & 1 setting
    // for 4x2 and 1xb4 & 2x2 mode: controller 1 is mapped to phy1( controller 0 is mapped to phy0);
    // phy1 d1 to lane 1, phy1 d0 to lane 0; phy0 d1 to lane 1; phy0 d0 to lane 0
    max96712_write(priv, 0x8a3, 0x44);

    /* dphy mode - Configure a 2-lane DPHY using PHY0. */
    max96712_write(priv, 0x90a, 0b01000000);
    /* dphy mode - Configure a 2-lane DPHY using PHY1*/
    max96712_write(priv, 0x94a, 0x40);

}


static void max96712_mipi_dphy_2x4(struct max96712_priv *priv)
{
    dev_info(&priv->client->dev, "%s in ", __func__);
    // Select 2x4 mode
    max96712_write(priv, 0x8a0, 0x04);

    // enable output phy
    max96712_write(priv, 0x8a2, 0xf4);

    // Configure lane mapping for PHY0 and PHY1
    // ob1110 0100

    // controller 1 setting
    // for 4x2 and 1xb4 & 2x2 mode: controller 1 is mapped to phy1( controller 0 is mapped to phy0);
    // note: controller 1 is mapped to phy0 and phy1 for 2x4 mode and 1x4a mode; controller 0 is unused.
    // phy1 d1 to lane 3, phy1 d0 to lane 2; phy0 d1 to lane 1; phy0 d0 to lane 0
    max96712_write(priv, 0x8a3, 0xe4);

    // controller 2 setting
    // note:controller 2 is mapped to phy2 and phy3 for 2x4 mode and 1x4b mode; controller 2 is unused.
    // phy3 d1 to lane 3, phy3 d0 to lane 2; phy2 d1 to lane 1; phy2 d0 to lane 0
    max96712_write(priv, 0x8a4, 0xe4);

    max96712_write(priv, 0x90a, 0xc0); // 4 lanes
    max96712_write(priv, 0x94a, 0xc0);
    max96712_write(priv, 0x98a, 0xc0);
    max96712_write(priv, 0x9ca, 0xc0);
}


static void max96712_mipi_dpll(struct max96712_priv *priv, unsigned int dpll_freq)
{
    dev_info(&priv->client->dev, "%s in ", __func__);

    // put dpll in reset before changing mipi lane rate.
    max96712_write(priv, 0x1d00, 0xf4); // controller 1
    max96712_write(priv, 0x1e00, 0xf4); // controller 2

    //Set link frequency for phy 0 1 2 3 to 1.5GHz
    max96712_write(priv, 0x0415, 0xef);
    max96712_write(priv, 0x0418, 0xef);

    // release reset after dpll.
    max96712_write(priv, 0x1d00, 0xf5); // controller 1
    max96712_write(priv, 0x1e00, 0xf5); // controller 2
}

static void max96712_mipi_polarity(struct max96712_priv *priv)
{
}


static void max96712_mipi_configure(struct max96712_priv *priv, enum max96712_mipi_mode mode)
{
    if ( MIPI_MODE_4X2 == mode) {
        max96712_mipi_dphy_4x2(priv);
    } else if (MIPI_MODE_2X4 == mode) {
        max96712_mipi_dphy_2x4( priv);
    }

    max96712_mipi_polarity(priv);

    max96712_mipi_dpll(priv, MAX96712_DPLL_FREQ);

}

static void max96712_pattern_enable(struct max96712_priv *priv, bool enable)
{
    const u32 h_active = 1920;
    const u32 h_fp = 88;
    const u32 h_sw = 44;
    const u32 h_bp = 148;
    const u32 h_tot = h_active + h_fp + h_sw + h_bp;

    const u32 v_active = 1080;
    const u32 v_fp = 4;
    const u32 v_sw = 5;
    const u32 v_bp = 36;
    const u32 v_tot = v_active + v_fp + v_sw + v_bp;

    if (!enable) {
        max96712_write(priv, 0x1051, 0x00);
        max96712_write(priv, 0x1050, 0x03);
        return;
    }

    /* PCLK 75MHz. */
    max96712_write(priv, 0x0009, 0x01);

    /* Configure Video Timing Generator for 1920x1080 @ 30 fps. */
    max96712_write_bulk_value(priv, 0x1052, 0, 3);
    max96712_write_bulk_value(priv, 0x1055, v_sw * h_tot, 3);
    max96712_write_bulk_value(priv, 0x1058,
                  (v_active + v_fp + + v_bp) * h_tot, 3);
    max96712_write_bulk_value(priv, 0x105b, 0, 3);
    max96712_write_bulk_value(priv, 0x105e, h_sw, 2);
    max96712_write_bulk_value(priv, 0x1060, h_active + h_fp + h_bp, 2);
    max96712_write_bulk_value(priv, 0x1062, v_tot, 2);
    max96712_write_bulk_value(priv, 0x1064,
                  h_tot * (v_sw + v_bp) + (h_sw + h_bp), 3);
    max96712_write_bulk_value(priv, 0x1067, h_active, 2);
    max96712_write_bulk_value(priv, 0x1069, h_fp + h_sw + h_bp, 2);
    max96712_write_bulk_value(priv, 0x106b, v_active, 2);

    /* Generate VS, HS and DE in free-running mode. */
    max96712_write(priv, 0x1050, 0xfb);

    /* Configure Video Pattern Generator. */
    if (priv->pattern == MAX96712_PATTERN_CHECKERBOARD) {
        /* Set checkerboard pattern size. */
        max96712_write(priv, 0x1074, 0x3c); // repeat count of color A. pixels
        max96712_write(priv, 0x1075, 0x3c); // repeat count of color B. pixels
        max96712_write(priv, 0x1076, 0x3c ); //line height, pixels.

        /* Set checkerboard pattern colors. */
        max96712_write_bulk_value(priv, 0x106e, 0x000000, 3);
        max96712_write_bulk_value(priv, 0x1071, 0xffffff, 3);

        /* Generate checkerboard pattern. */
        max96712_write(priv, 0x1051, 0x10);
    } else {
        /* Set gradient increment. */
        max96712_write(priv, 0x106d, 0x8);

        /* Generate gradient pattern. */
        max96712_write(priv, 0x1051, 0xa0);
    }
}

static int max96712_link_oneshot_reset(struct max96712_priv *priv)
{
    max96712_write( priv, 0x0018, 0b00001111);
    msleep(100);
    return 0;
}

static int max96712_link_disable(struct max96712_priv *priv )
{
    max96712_write( priv, 0x0006, 0x00) ; // disable link A B C D
    return 0;
}

// return lock state. 1 locked; 0 not locked.
static int max96712_wait_link_locked(struct max96712_priv *priv, int link)
{
    const int max_retry = 5;
    int retry = 0;
    int rtn = 0;
    for (retry = 0; retry < max_retry; ++retry) {

        msleep(60); // tLOCK2 maximum 60ms, refer to max96722 datasheet.

        rtn = max96712_gmsl_1_link_lock_state(priv, link);
        if (rtn) {
            dev_info(&priv->client->dev, "link %d locked\n", link);
            break;
        }
        dev_info(&priv->client->dev, "link %d not locked, retry %d\n", link, retry);
    }
    return rtn;
}

static int max96712_link_enable(struct max96712_priv *priv, int link)
{
    int rtn = 0;

    dev_info(&priv->client->dev, " enable link , link %d \n", link);
    // enable link A to gmsl 1
    switch (link) {
        case LINK_A:
        {
            max96712_write( priv, 0x0006, 0b00000001);
            max96712_write( priv, 0x0018, 0b00000001);
            rtn = max96712_wait_link_locked(priv, link);
        } break;
        case LINK_B: {
            max96712_write( priv, 0x0006, 0b00000010);
            max96712_write( priv, 0x0018, 0b00000010);
            rtn = max96712_wait_link_locked(priv, link);
        } break;
        case LINK_C:
        {
            max96712_write( priv, 0x0006, 0b00000100);
            max96712_write( priv, 0x0018, 0b00000100);
            rtn = max96712_wait_link_locked(priv, link);
        } break;
        case LINK_D: {
            max96712_write( priv, 0x0006, 0b00001000);
            max96712_write( priv, 0x0018, 0b00001000);
            rtn = max96712_wait_link_locked(priv, link);
        } break;
        default:
            dev_err(&priv->client->dev, " unknown link \n");
        break;
    }
    return rtn;
}

static int max96712_enable_vdd_ldo_reglator(struct max96712_priv *priv)
{
    max96712_write( priv, 0x0017, 0X14); //enable REG_ENABLE
    max96712_write( priv, 0x0019, 0X94) ; // enable REG_MNL
    return 0;
}

static int max96712_rlms_init(struct max96712_priv *priv)
{
    // no document for these registers.
    // support from vendor. works well.
    max96712_write( priv, 0x1445, 0X00);
    max96712_write( priv, 0x1545, 0X00);
    max96712_write( priv, 0x1645, 0X00);
    max96712_write( priv, 0x1745, 0X00);
    max96712_write( priv, 0x14d1, 0X03);
    max96712_write( priv, 0x15d1, 0X03);
    max96712_write( priv, 0x16d1, 0X03);
    max96712_write( priv, 0x17d1, 0X03);
    return 0;
}

static int max96712_cmu_init(struct max96712_priv *priv)
{
    // no document for these registers.
    // support from vendor. works well.
    max96712_write( priv, 0x6c2, 0x10);
    return 0;
}


static int max96712_set_him_mode(struct max96712_priv *priv, int enable)
{
    if (1 == enable) {
        // Turn on HIM on MAX96712
        // 0x0b06 , default 0xef. him is enabled.
        max96712_update_bits(priv, 0x0B06, 0x80, 0x80); //Link A HIM
        max96712_update_bits(priv, 0x0C06, 0x80, 0x80); //Link B HIM
        max96712_update_bits(priv, 0x0D06, 0x80, 0x80); //Link C HIM
        max96712_update_bits(priv, 0x0E06, 0x80, 0x80); //Link D HIM
    } else {
        max96712_update_bits(priv, 0x0B06, 0x80, 0x00); //Link A HIM
        max96712_update_bits(priv, 0x0C06, 0x80, 0x00); //Link B HIM
        max96712_update_bits(priv, 0x0D06, 0x80, 0x00); //Link C HIM
        max96712_update_bits(priv, 0x0E06, 0x80, 0x00); //Link D HIM
    }
    return 0;
}


static int max96712_set_hs_de_mode(struct max96712_priv *priv, int enable)
{
    if (0 == enable) {
        // disable processing HS AND DE signals
        max96712_write(priv, 0x0B0f, 0x01);
        max96712_write(priv, 0x0c0f, 0x01);
        max96712_write(priv, 0x0d0f, 0x01);
        max96712_write(priv, 0x0e0f, 0x01);
    }
    return 0;
}


static int max96712_set_dbl_hven_mode(struct max96712_priv *priv, int enable)
{
    if (1 == enable) {
        // enable hs/vs encoding & double output
        // reg 0xb07 7 dbl 6 drs 5 bws 4 rsvd 3 hibw 2 hven 1 rsvd 0 pxl_crc
        // default 0 0 0 0 0 0 0 0
        // set to  1 0 0 0 0 1 0 0
        max96712_update_bits(priv, 0x0B07, 0x84, 0x84); //Link A dbl & hven
        max96712_update_bits(priv, 0x0C07, 0x84, 0x84); //Link B dbl & hven
        max96712_update_bits(priv, 0x0D07, 0x84, 0x84); //Link C dbl & hven
        max96712_update_bits(priv, 0x0E07, 0x84, 0x84); //Link D dbl & hven
    } else {
        // disable hs/vs encoding & double output
        max96712_update_bits(priv, 0x0B07, 0x84, 0x00); //Link A dbl & hven
        max96712_update_bits(priv, 0x0C07, 0x84, 0x00); //Link B dbl & hven
        max96712_update_bits(priv, 0x0D07, 0x84, 0x00); //Link C dbl & hven
        max96712_update_bits(priv, 0x0E07, 0x84, 0x00); //Link D dbl & hven
    }
    return 0;
}

static int max96712_yuv_mux_mode(struct max96712_priv *priv)
{
    max96712_write( priv, 0x041a, 0Xf0);
    return 0;
}


static int max96712_enable_video_pipe(struct max96712_priv *priv)
{
    max96712_write( priv, 0x00f4, 0X0f);// enable video pipe 0123
    return 0;
}

// software override. manually set DT VC & BPP for incoming data.
// GMSL 1 serializer in parallel mode( serializer with a parallel camera)
// DT & BPP must be adjusted together.
static int max96712_link_software_override(struct max96712_priv *priv)
{
    dev_info(&priv->client->dev, "video pipe 0 1 2 3. software override");
    dev_info(&priv->client->dev, "vc 0; dt 0x1e(yuv422 8 bit); bpp 0x08;");

    // video pipe 0. software override BPP 7:3
    // Warning: datasheet : 0x10 is for DT 0x1E. 0x8 is for DT 0x2A, 0X10 ~ 0X12, 0x31 ~ 0x37.
    //          from vendor: BPP 0x08 & DT 0x1E, it works.
    max96712_write( priv, 0x040b, 0x40);  //   7:3 0x08
    // video pipe 1 2 3. software override BPP is 0x08
    // 4:0 is 0x08; [7:5][1:0]  is 0x08; 6:0 is 0x08
    max96712_write(priv, 0x0411, 0b01001000); // 0b 0100 1000 ; 0x48
    max96712_write(priv, 0x0412, 0b00100000); // 0b 001000 00; 0x20

    // video pipe 0 1 2 3. software override VC
    max96712_write( priv, 0x040c, 0x00);
    max96712_write( priv, 0x040d, 0x00);

    // video pipe 0 1 2 3. software override DT. data type is 0x1e. yuv 422 8 bit.
    max96712_write( priv, 0x040e, 0b01011110); // 0x5e
    max96712_write( priv, 0x040f, 0b01111110); // 0x7e
    max96712_write( priv, 0x0410, 0b01111010); // 0x7a

    return 0;
}


static int max96712_video_pipe_2_mipi_controller(struct max96712_priv *priv)
{
    dev_info(&priv->client->dev, " video pipe 0 -> controller 1 \n");
    //=========  video pipi 0 =====================
    // enable 3 mapping block for video pipe 0.
    // mapping block 0 1 2 is enabled.
    max96712_write( priv, 0x090b, 0X07);

    //0b00 01 01 01
    // 3 mapping blocks' dest is mipi controller 1
    max96712_write( priv, 0x092d, 0X15);

    // set src and dst for mapping blocks.
    max96712_write( priv, 0x090d, 0X1E); // mapping block 0. transfer dt 1e vc0 data
    max96712_write( priv, 0x090E, 0X1E);

    max96712_write( priv, 0x090f, 0X00); // mapping block 1. transfer frame start vc0 data
    max96712_write( priv, 0x0910, 0X00);

    max96712_write( priv, 0x0911, 0X01); // mapping block 2. transfer frame end vc0 data.
    max96712_write( priv, 0x0912, 0X01);


    //=========  video pipi 1 =====================
    // enable 3 mapping block for video pipe 0.
    // mapping block 0 1 2 is enabled.
    max96712_write( priv, 0x094b, 0X07);

    //0b00 01 01 01
    // 3 mapping blocks' dest is mipi controller 1
    max96712_write( priv, 0x096d, 0X15);

    // set src and dst for mapping blocks.
    max96712_write( priv, 0x094d, 0X1E); // mapping block 0. transfer dt 1e vc1 data
    max96712_write( priv, 0x094E, 0X5E);

    max96712_write( priv, 0x094f, 0X00); // mapping block 1. transfer frame start vc0 data
    max96712_write( priv, 0x0950, 0X40);

    max96712_write( priv, 0x0951, 0X01); // mapping block 2. transfer frame end vc0 data.
    max96712_write( priv, 0x0952, 0X41);


    //=========  video pipi 2 =====================
    // enable 3 mapping block for video pipe 0.
    // mapping block 0 1 2 is enabled.
    max96712_write( priv, 0x098b, 0X07);

    //0b00 01 01 01
    // 3 mapping blocks' dest is mipi controller 1
    max96712_write( priv, 0x09ad, 0X15);

    // set src and dst for mapping blocks.
    max96712_write( priv, 0x098d, 0X1E); // mapping block 0. transfer dt 1e vc0 data
    max96712_write( priv, 0x098E, 0X9E);

    max96712_write( priv, 0x098f, 0X00); // mapping block 1. transfer frame start vc0 data
    max96712_write( priv, 0x0990, 0X80);

    max96712_write( priv, 0x0991, 0X01); // mapping block 2. transfer frame end vc0 data.
    max96712_write( priv, 0x0992, 0X81);


    //=========  video pipi 3 =====================
    // enable 3 mapping block for video pipe 0.
    // mapping block 0 1 2 is enabled.
    max96712_write( priv, 0x09cb, 0X07);

    //0b00 01 01 01
    // 3 mapping blocks' dest is mipi controller 1
    max96712_write( priv, 0x09ed, 0X15);

    // set src and dst for mapping blocks.
    max96712_write( priv, 0x09cd, 0X1E); // mapping block 0. transfer dt 1e vc0 data
    max96712_write( priv, 0x09cE, 0XDE);

    max96712_write( priv, 0x09cf, 0X00); // mapping block 1. transfer frame start vc0 data
    max96712_write( priv, 0x09d0, 0XC0);

    max96712_write( priv, 0x09d1, 0X01); // mapping block 2. transfer frame end vc0 data.
    max96712_write( priv, 0x09d2, 0XC1);

    return 0;
}

static int max96712_crossbar_init(struct max96712_priv *priv)
{
    // d2 ~ d9 --> d0 ~ d7
    //====== vrx 0 ===========
    max96712_write( priv, 0x01c0, 0X02);
    max96712_write( priv, 0x01c1, 0X03);
    max96712_write( priv, 0x01c2, 0X04);
    max96712_write( priv, 0x01c3, 0X05);
    max96712_write( priv, 0x01c4, 0X06);
    max96712_write( priv, 0x01c5, 0X07);
    max96712_write( priv, 0x01c6, 0X08);
    max96712_write( priv, 0x01c7, 0X09);


    //====== vrx 1 ===========
    max96712_write( priv, 0x01e0, 0X02);
    max96712_write( priv, 0x01e1, 0X03);
    max96712_write( priv, 0x01e2, 0X04);
    max96712_write( priv, 0x01e3, 0X05);
    max96712_write( priv, 0x01e4, 0X06);
    max96712_write( priv, 0x01e5, 0X07);
    max96712_write( priv, 0x01e6, 0X08);
    max96712_write( priv, 0x01e7, 0X09);

    //====== vrx 2 ===========
    max96712_write( priv, 0x0200, 0X02);
    max96712_write( priv, 0x0201, 0X03);
    max96712_write( priv, 0x0202, 0X04);
    max96712_write( priv, 0x0203, 0X05);
    max96712_write( priv, 0x0204, 0X06);
    max96712_write( priv, 0x0205, 0X07);
    max96712_write( priv, 0x0206, 0X08);
    max96712_write( priv, 0x0207, 0X09);


    //====== vrx 3 ===========
    max96712_write( priv, 0x0220, 0X02);
    max96712_write( priv, 0x0221, 0X03);
    max96712_write( priv, 0x0222, 0X04);
    max96712_write( priv, 0x0223, 0X05);
    max96712_write( priv, 0x0224, 0X06);
    max96712_write( priv, 0x0225, 0X07);
    max96712_write( priv, 0x0226, 0X08);
    max96712_write( priv, 0x0227, 0X09);

    return 0;
}


// internal FSYNC programming for gmsl1
static int max96712_framesync_init(struct max96712_priv *priv)
{
    // 0x04 0b0000 01 00
    // bit 1:0 == 00 manual framesync method.
    // bit 3:2 == 01 framesync generation is on. GPIO is used as fsync output and drives a slave device.
    // bit 4 - 0 . use VS from source.
    // bit 5 - 0 - works only when bit 3:2 = 0b01.select gpio to output fsync signal.
    //             0 - MFP2.   1 - MFP10
    // bit 6 select FSYNC falling transition. 0 - set at the middle of frame.
    // bit 7 - 0 . do not reset framesync generation.
    max96712_write( priv, 0x04a0, 0x04);

    // turn off auto master link selection.
    // bit 7:5 - 0b000 video 0 is selected as master.
    max96712_write( priv, 0x04a2, 0x00);

    //disable overlap window
    max96712_write( priv, 0x04aa, 0x00);
    max96712_write( priv, 0x04ab, 0x00);

    //set to use 25Mhz xtal. AUTO_FS_LINKS = 0, FS_USE_XTAL = 1. FS_LINK_x[3:0] = 0
    // 0x40  0b01000000
    // bit 7 - 0 -  type of FSYNC signal is GMSL 1 type
    // bit 6 - 1 - FS_USE_XTAL = 1.
    // bit 4 - 0 - AUTO_FS_LINKS = 0. include links selected by bit[3:0]
    // bit 3:0 0b1111
    max96712_write( priv, 0x04af, 0x40);

#if defined(SENSOR_30FPS_FRAMESYNC)
    // 30 fps. frame sync period in terms of PCLK. PCLK = 25Mhz FSYNC is 30Hz.
    // 25000000 / 30 = 0x0cb735
    dev_info(&priv->client->dev, " fsync using 30fps \n");
    max96712_write( priv, 0x04a7, 0x0c);
    max96712_write( priv, 0x04a6, 0xb7);
    max96712_write( priv, 0x04a5, 0x35);
#elif defined(SENSOR_25FPS_FRAMESYNC)
    // 25000000 / 25 = 0x0F4240
    dev_info(&priv->client->dev, " fsync using 25fps \n");
    max96712_write( priv, 0x04a7, 0x0F);
    max96712_write( priv, 0x04a6, 0x42);
    max96712_write( priv, 0x04a5, 0x40);
#else // 15 fps
    // 15 fps.
    // 25000000 / 15 = 0x19 6E6A
    dev_info(&priv->client->dev, " fsync using 15fps \n");
    max96712_write( priv, 0x04a7, 0x19);
    max96712_write( priv, 0x04a6, 0x6e);
    max96712_write( priv, 0x04a5, 0x6a);
#endif

    // enable FSYNC transmission to serializer.
    // select GPI_1 to serializer. framesync enabled.
    // 0x71 - 0b01 1 1 0001 - enable fsync signal transmission
    // 0x61 - 0b01 1 0 0001 - disable fsync signal transmission

    // bit 7:6 - GPI_SEL is 0b01. GPI_1
    // bit 5 - GPI_EN enable GPI_to_GPO signal transmission
    // bit 4 - en fsync signal transmission.
    // bit 3 -  rsv - 0.
    // bit 2 - packet-based control-channel mode - 0
    // bit 1:0 control channel CRC length. 0b01 - 5 bit.
    max96712_write( priv, 0x0b08, 0x71);
    max96712_write( priv, 0x0c08, 0x71);
    max96712_write( priv, 0x0d08, 0x71);
    max96712_write( priv, 0x0e08, 0x71);

    return 0;
}

static int max96712_aggregation_init(struct max96712_priv *priv)
{
    // wx4h, video 0 as master. using video 0 1 2 3
    dev_info(&priv->client->dev, "aggregation using 0x0F for wx4h \n");
    max96712_write( priv, 0x0971, 0x0f);
    return 0;
}


static int max96712_set_coax(struct max96712_priv *priv)
{
    max96712_write( priv, 0x0022, 0Xff); //using coax cable
    return 0;
}

static int max96712_set_dbg(struct max96712_priv *priv)
{
    // reg FA.
    // 6 de en 5 hs en 4 vs en 2:0 hvd sel
    // enable de hs vs out to gpio.
    // which pipe's de hs vs.


    // read 0x01dc bit 1. video pipe 0 lock status         1  video channel 0 is locked.

    // read reg 0x08d0 0x08d1
    // controller 0/1/2/3 packet count. -                    0x00 mipi 没有发送 packet

    // read reg 0x04b6. bit 6 means frame sync locked.      -没有使用fsync.仅使用linkA 接一个相机.

    // read reg 0x040a. check whether there is error       - 0x00 没有错误

    //check reg 0x11f0 ~ 0x11f2 whether there is de hs vs det- 0x00. 没有 de hs vs.
    return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int max96712_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *cfg,
                 struct v4l2_subdev_mbus_code_enum *code)
#else
static int max96712_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_pad_config *cfg,
                 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
    if (code->pad != 0)
        return -EINVAL;
    if (code->index >= ARRAY_SIZE(max96712_formats))
        return -EINVAL;

    code->code = max96712_formats[code->index].code;
    return 0;
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int max96712_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *cfg,
                  struct v4l2_subdev_frame_size_enum *fse)
#else
static int max96712_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_pad_config *cfg,
                  struct v4l2_subdev_frame_size_enum *fse)
#endif
{
    if (fse->index >= ARRAY_SIZE(max96712_formats))
        return -EINVAL;

    fse->min_width = max96712_formats[fse->index].min_width;
    fse->min_height = max96712_formats[fse->index].min_height;;
    fse->max_width = max96712_formats[fse->index].max_width;
    fse->max_height = max96712_formats[fse->index].max_height;

    return 0;
}

static int max96712_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct max96712_priv *priv = v4l2_get_subdevdata(sd);

    if (enable) {
        max96712_mipi_enable(priv, true);
    } else {
        max96712_mipi_enable(priv, false);
    }

    return 0;
}

static const struct v4l2_subdev_video_ops max96712_video_ops = {
    .s_stream = max96712_s_stream,
};
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int max96712_get_pad_format(struct v4l2_subdev *sd,
                   struct v4l2_subdev_state *cfg,
                   struct v4l2_subdev_format *format)
#else
static int max96712_get_pad_format(struct v4l2_subdev *sd,
                   struct v4l2_subdev_pad_config *cfg,
                   struct v4l2_subdev_format *format)
#endif
{
    struct max96712_priv *priv = to_max96712_dev(sd);
    struct v4l2_mbus_framefmt *fmt;

    if (format->pad != 0)
        return -EINVAL;

    if (format->which == V4L2_SUBDEV_FORMAT_TRY)
        fmt = v4l2_subdev_get_try_format(&priv->sd, cfg,
                         format->pad);
    else
        fmt = &priv->current_format;

    format->format = *fmt;
    return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int max96712_set_pad_format(struct v4l2_subdev *sd,
              struct v4l2_subdev_state *cfg,
              struct v4l2_subdev_format *fmt)
#else
static int max96712_set_pad_format(struct v4l2_subdev *sd,
              struct v4l2_subdev_pad_config *cfg,
              struct v4l2_subdev_format *fmt)
#endif
{
    struct max96712_priv *priv = to_max96712_dev(sd);

    struct max96712_mode *mode;

    struct v4l2_mbus_framefmt *format;

    unsigned int i;

    dev_info(&priv->client->dev, " %s  w %d, h %d code 0x%x in \n",__func__, fmt->format.width, fmt->format.height, fmt->format.code);

    mutex_lock(&priv->lock);

    mode = v4l2_find_nearest_size(max96712_modes,
                ARRAY_SIZE(max96712_modes),
                width, height,
                fmt->format.width, fmt->format.height);

    fmt->format.width = mode->width;
    fmt->format.height = mode->height;

    for (i = 0; i < ARRAY_SIZE(max96712_formats); i++) {
        if (max96712_formats[i].code == fmt->format.code) {
            dev_err(&priv->client->dev, " find matched format 0x%x, idx %d \n", fmt->format.code, i);
            break;
        }
    }

    if (i >= ARRAY_SIZE(max96712_formats)) {
        i = 0;
        dev_err(&priv->client->dev, " No format. reset i = 0 \n");
    }

    fmt->format.code = max96712_formats[0].code;

    fmt->format.field = V4L2_FIELD_NONE;

    if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
        dev_err(&priv->client->dev, " try format \n");
        format = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
    } else {
        dev_err(&priv->client->dev, " set format, w %d, h %d, code 0x%x \n",
            fmt->format.width, fmt->format.height,
            fmt->format.code);
        format = &priv->current_format;
        priv->current_mode = mode;
    }

    *format = fmt->format;

    mutex_unlock(&priv->lock);

    return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static int max96712_init_cfg(struct v4l2_subdev *subdev,
                         struct v4l2_subdev_state *cfg)
#else
static int max96712_init_cfg(struct v4l2_subdev *subdev,
                            struct v4l2_subdev_pad_config *cfg)
#endif
{
    struct v4l2_subdev_format fmt = { 0 };

    fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
    fmt.format.width = 1280;
    fmt.format.height = 3200;

    max96712_set_pad_format(subdev, cfg, &fmt);

    return 0;
}

static const struct v4l2_subdev_pad_ops max96712_pad_ops = {
    .init_cfg = max96712_init_cfg,

    .enum_mbus_code = max96712_enum_mbus_code,
    .enum_frame_size = max96712_enum_frame_size,

    .get_fmt = max96712_get_pad_format,
    .set_fmt = max96712_set_pad_format,
};


static const struct v4l2_subdev_ops max96712_subdev_ops = {
    .video = &max96712_video_ops,
    .pad = &max96712_pad_ops,
};

static const char * const max96712_test_pattern[] = {
    "Checkerboard",
    "Gradient",
};

static int max96712_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct max96712_priv *priv =
        container_of(ctrl->handler, struct max96712_priv, ctrl_handler);

    switch (ctrl->id) {
    case V4L2_CID_AML_CSI_LANES:
        break;

    case V4L2_CID_TEST_PATTERN:
        priv->pattern = ctrl->val ?
            MAX96712_PATTERN_GRADIENT :
            MAX96712_PATTERN_CHECKERBOARD;
        break;
    }
    return 0;
}

static const struct v4l2_ctrl_ops max96712_ctrl_ops = {
    .s_ctrl = max96712_s_ctrl,
};

static u64 calc_pixel_rate(struct max96712_priv *priv)
{
    u64 rate;

    rate = 1280 * 4 * 800;
    rate *= 25;

    return rate;
}

static struct v4l2_ctrl_config nlane_cfg = {
    .ops = &max96712_ctrl_ops,
    .id = V4L2_CID_AML_CSI_LANES,
    .name = "sensor lanes",
    .type = V4L2_CTRL_TYPE_INTEGER,
    .flags = V4L2_CTRL_FLAG_VOLATILE,
    .min = 1,
    .max = 4,
    .step = 1,
    .def = 4,
};

static int max96712_v4l2_init_ctrl(struct max96712_priv *priv)
{
    v4l2_ctrl_handler_init(&priv->ctrl_handler, 4);

    /* Clock related controls */
    priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE,
                                          0, INT_MAX, 1,
                                          calc_pixel_rate(priv) );

    priv->link_freq = v4l2_ctrl_new_int_menu(&priv->ctrl_handler,
                               NULL,
                               V4L2_CID_LINK_FREQ,
                               ARRAY_SIZE(max96712_link_freq) - 1,
                               0, max96712_link_freq);

    if (priv->link_freq)
        priv->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    priv->data_lanes = v4l2_ctrl_new_custom(&priv->ctrl_handler, &nlane_cfg, NULL);
    if (priv->data_lanes) {
        __v4l2_ctrl_s_ctrl(priv->data_lanes, priv->nlanes);
        priv->data_lanes->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    }

    priv->sd.ctrl_handler = &priv->ctrl_handler;
    return priv->ctrl_handler.error;
}

static int max96712_v4l2_register(struct max96712_priv *priv)
{

    int ret;

    v4l2_i2c_subdev_init(&priv->sd, priv->client, &max96712_subdev_ops);

    snprintf(priv->sd.name, sizeof(priv->sd.name), AML_SENSOR_NAME, priv->index);

    priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE|
                     V4L2_SUBDEV_FL_HAS_EVENTS;

    priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

    priv->pads[0].flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&priv->sd.entity, 1, priv->pads);
    if (ret)
        goto error;

    v4l2_set_subdevdata(&priv->sd, priv);

    ret = v4l2_async_register_subdev(&priv->sd);
    if (ret < 0) {
        dev_err(&priv->client->dev, "Unable to register subdevice\n");
        goto error;
    }

    return 0;
error:
    v4l2_ctrl_handler_free(&priv->ctrl_handler);

    return ret;
}

static const struct regmap_config max96712_i2c_regmap = {
    .reg_bits = 16,
    .val_bits = 8,
    .max_register = 0x1f00,
};


int max96712_avm_init(struct i2c_client *client, void *sdrv)
{
    struct device *dev = &client->dev;
    struct max96712_priv *max96712;
    struct amlsens *sensor = (struct amlsens *)sdrv;
    int ret = -EINVAL;
    int sensor_idx = 0;
    int link_idx = 0;
    int connected_sensor_count = 0;

    max96712 = devm_kzalloc(dev, sizeof(struct max96712_priv), GFP_KERNEL);
    if (!max96712)
        return -ENOMEM;

    max96712->dev = dev;
    max96712->client = client;
    max96712->client->addr = MAX96712_SLAVE_ID;
    max96712->nlanes = 4;

    mutex_init(&max96712->lock);

    if (of_property_read_u32(dev->of_node, "index", &max96712->index)) {
        dev_err(dev, "Failed to read sensor index. default to 0\n");
        max96712->index = 0;
    }

    max96712->regmap = devm_regmap_init_i2c(client, &max96712_i2c_regmap);
    if (IS_ERR(max96712->regmap))
        return PTR_ERR(max96712->regmap);


    //============ init sers & sensors ==============
    for (sensor_idx = 0; sensor_idx < MAX_SENSOR_NUM; ++sensor_idx) {
        max96712->ser_devs[sensor_idx].index = sensor_idx;
        max96712->ser_devs[sensor_idx].client = client;

        max96712->ser_devs[sensor_idx].i2c_addr = MAX96705_I2C_ADDR;
        max96712->ser_devs[sensor_idx].i2c_reg_addr_bytes = 1;
        max96712->ser_devs[sensor_idx].i2c_reg_value_bytes = 1;
        max96712->ser_devs[sensor_idx].ref_des = max96712;

        max96712->sensor_devs[sensor_idx].index = sensor_idx;
        max96712->sensor_devs[sensor_idx].client = client;

        max96712->sensor_devs[sensor_idx].i2c_addr = OX01F10_I2C_ADDR;
        max96712->sensor_devs[sensor_idx].i2c_reg_addr_bytes = 2;
        max96712->sensor_devs[sensor_idx].i2c_reg_value_bytes = 1;
        max96712->sensor_devs[sensor_idx].ref_des = max96712;
    }

    // ==========  hw related init ====================
    dev_info(dev, "config max96712");
    if (0 != max96712_avm_sensor_id(client) ) {
        dev_err(dev,  "FAIL - check id failed!, continue...");
        goto end_i2c_config;
    }


    dev_info(dev, "SUCC - check id succeed!!!");

    dev_info(dev, "SUCC - check id succeed!!!");

    // reset deserializer
    max96712_reset(max96712); // delay is done inside.

    // optional step 2.5  - for 1.2V vdd.
    max96712_enable_vdd_ldo_reglator(max96712);

    max96712_rlms_init(max96712);
    max96712_cmu_init(max96712);

    // using coax.
    max96712_set_coax(max96712);

    // disable mipi
    max96712_mipi_enable(max96712, false);

    max96712_link_disable(max96712);

    max96712_set_him_mode(max96712, 1);
    max96712_set_hs_de_mode(max96712, 0);
    max96712_set_dbl_hven_mode(max96712, 1);

    max96712_yuv_mux_mode(max96712);

    max96712_link_oneshot_reset(max96712);

    max96712_enable_video_pipe(max96712);

    max96712_mipi_configure(max96712, MIPI_MODE_2X4);

    // step 5: video pipe to controller.
    // 2x4 mode. controller 0 is unused. controller 1 is mapped to phy0 & phy1.
    // 4x2 mode. video pipe 0 is mapped to controller 0.
    // no video pipe concept in GMSL1 links. data path can be considered as link to mipi controller.
    max96712_link_software_override(max96712);
    dev_info(&max96712->client->dev, "no pipe-2-controller setting. using aggregation super frame");
    //max96712_video_pipe_2_mipi_controller(max96712);

    //max96712_crossbar_init(max96712);

    max96712_framesync_init(max96712);
    max96712_aggregation_init(max96712);

    // =============  init  each link ===============
    for (link_idx = LINK_A; link_idx <= LINK_D; ++link_idx) {
        u32 id = 0;

        if (max96712_link_enable(max96712, link_idx) ) {
            pr_info("config max96705");
            // do not change 96705 i2c addr
            max96705_write_reg( &(max96712->ser_devs[link_idx]) , 0x07, 0x84);
            msleep(5);
            max96705_write_reg( &(max96712->ser_devs[link_idx]) , 0x06, 0xa4);
        }
        // check video lock state
        ret= max96712_gmsl_1_video_lock_state(max96712,link_idx);
        if (ret) {
            dev_info(&max96712->client->dev, "gmsl 1 video pipe %d locked \n", link_idx);
        } else {
            dev_info(&max96712->client->dev, "gmsl 1  video pipe %d not locked \n", link_idx);
        }

        ret= max96712_gmsl_2_video_lock_state(max96712,link_idx);
        if (ret) {
            dev_info(&max96712->client->dev, "gmsl 2 video pipe %d locked \n", link_idx);
        } else {
            dev_info(&max96712->client->dev, "gmsl 2 video pipe %d not locked \n", link_idx);
        }
    }

    max96712_write( max96712, 0x01d9, 0x59);
    max96712_write( max96712, 0x01f9, 0x59);
    max96712_write( max96712, 0x0219, 0x59);
    max96712_write( max96712, 0x0239, 0x59);

    // enable all links and oneshot reset.
    max96712_write( max96712, 0x0006, 0b00001111);
    max96712_write( max96712, 0x0018, 0b00001111);
    msleep(100);

end_i2c_config:
    // ==========  v4l2 related init ====================

    max96712_init_cfg(&max96712->sd, NULL);

    ret = max96712_v4l2_init_ctrl(max96712);
    if (ret) {
        dev_err(dev, "init ctrl error, continue....");
    }

    ret =  max96712_v4l2_register(max96712);
    if (ret) {
        dev_err(dev, "v4l2 register error, continue");
    }
    dev_info(dev, "probe success");

    return 0;
}

int max96712_avm_deinit(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct max96712_priv *max96712 = to_max96712_dev(sd);
    struct device *dev = &client->dev;

    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(sd->ctrl_handler);

    mutex_destroy(&max96712->lock);
    devm_kfree(dev, max96712);

    return 0;
}

int max96712_avm_sensor_id(struct i2c_client *client)
{
    int rtn = -EINVAL;
    u8 id = 0;

    i2c_read_a16d8(client, MAX96712_SLAVE_ID, 0x0d, &id);
    if ( id == MAX96722_ID) {
        pr_info("found connected max96722\n");
        return 0;
    }

    i2c_read_a16d8(client, MAX96712_SLAVE_ID, 0x4a, &id );
    if ( id == MAX96712_ID) {
        pr_info("found connected max96712\n");
        return 0;
    }

    pr_err( "no max96712 or max96722 connected\n");

    return -1;
}

int max96712_avm_power_on(struct device *dev, struct sensor_gpio *gpio)
{
    // do nothing;
    // max96722 is not powered from an400 board;
    // it is a daughter board;
    return 0;
}

int max96712_avm_power_off(struct device *dev, struct sensor_gpio *gpio)
{
    // do nothing;
    // max96722 is not powered from an400 board;
    // it is a daughter board;

    return 0;
}

int max96712_avm_power_suspend(struct device *dev)
{
    // do nothing;
    // max96722 is not powered from an400 board;
    // it is a daughter board;

    return 0;
}

int max96712_avm_power_resume(struct device *dev)
{
    // do nothing;
    // max96722 is not powered from an400 board;
    // it is a daughter board;

    return 0;
}
