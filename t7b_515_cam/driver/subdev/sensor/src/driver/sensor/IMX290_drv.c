/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2018 ARM or its affiliates
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

#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>

#include "acamera_types.h"
#include "system_spi.h"
#include "system_sensor.h"
#include "sensor_bus_config.h"
#include "acamera_command_api.h"
#include "acamera_sbus_api.h"
#include "acamera_sensor_api.h"
#include "system_timer.h"
#include "sensor_init.h"
#include "IMX290_seq.h"
#include "IMX290_config.h"
#include "acamera_math.h"
#include "system_am_mipi.h"
#include "system_am_adap.h"
#include "sensor_bsp_common.h"

#define NEED_CONFIG_BSP 1

#define AGAIN_MAX_DB 0x64
#define DGAIN_MAX_DB 0x6e

#define FS_LIN_1080P 1
#define FS_LIN_1080P_60FPS 0

static void start_streaming( void *ctx );
static void stop_streaming( void *ctx );

static imx290_private_t imx290_ctx;
static sensor_context_t sensor_ctx;

static uint32_t initial_sensor = 0;

// 0 - reset & power-enable
// 1 - reset-sub & power-enable-sub
// 2 - reset-ssub & power-enable-ssub
static const int32_t config_sensor_idx = 0;                  // 1 2 3
static const char * reset_dts_pin_name = "reset";             // reset-sub  reset-ssub
static const char * pwr_dts_pin_name   = "pwdn";              // pwdn-sub pwdn-ssub


static sensor_mode_t supported_modes[] = {
    {
        .wdr_mode = WDR_MODE_LINEAR, // 4 Lanes
        .fps = 30 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 12,
        .exposures = 1,
        .lanes = 2,
        .bps = 446,
        .bayer = BAYER_RGGB,
        .dol_type = DOL_NON,
        .num = 7,
    },
    {
        .wdr_mode = WDR_MODE_FS_LIN, // 8 Lanes
        .fps = 30 * 256,
#if FS_LIN_1080P
        .resolution.width = 1920,
        .resolution.height = 1080,
#else
        .resolution.width = 1280,
        .resolution.height = 720,
#endif
        .bits = 10,
        .exposures = 2,
        .lanes = 4,
        .bps = 446,
        .bayer = BAYER_RGGB,
        .dol_type = DOL_LINEINFO,
        .num = 4,
    },
#if FS_LIN_1080P_60FPS
    {
        .wdr_mode = WDR_MODE_FS_LIN, // 8 Lanes
        .fps = 60 * 256,
        .resolution.width = 1920,
        .resolution.height = 1080,
        .bits = 10,
        .exposures = 2,
        .lanes = 4,
        .bps = 446,
        .bayer = BAYER_RGGB,
        .dol_type = DOL_LINEINFO,
        .num = 5,
    }
#endif
};

//-------------------------------------------------------------------------------------
#if SENSOR_BINARY_SEQUENCE
static const char p_sensor_data[] = SENSOR_IMX290_SEQUENCE_DEFAULT;
static const char p_isp_data[] = SENSOR_IMX290_ISP_CONTEXT_SEQ;
#else
static const acam_reg_t **p_sensor_data = imx290_seq_table;
static const acam_reg_t **p_isp_data = isp_seq_table;
#endif

//--------------------RESET------------------------------------------------------------
static void sensor_hw_reset_enable( void )
{
    system_reset_sensor( 0 );
}

static void sensor_hw_reset_disable( void )
{
    system_reset_sensor( 3 );
}

//-------------------------------------------------------------------------------------
static int32_t sensor_alloc_analog_gain( void *ctx, int32_t gain )
{
    sensor_context_t *p_ctx = ctx;

    uint16_t again = ( gain * 20 ) >> LOG2_GAIN_SHIFT;

    if ( again > p_ctx->again_limit ) again = p_ctx->again_limit;

    if ( p_ctx->again[0] != again ) {
        p_ctx->gain_cnt = p_ctx->again_delay + 1;
        p_ctx->again[0] = again;
    }

    return ( ( (int32_t)again ) << LOG2_GAIN_SHIFT ) / 20;
}

static int32_t sensor_alloc_digital_gain( void *ctx, int32_t gain )
{
    return 0;
}

static void sensor_alloc_integration_time( void *ctx, uint16_t *int_time_S, uint16_t *int_time_M, uint16_t *int_time_L )
{
    sensor_context_t *p_ctx = ctx;
    imx290_private_t *p_imx290 = (imx290_private_t *)p_ctx->sdrv;
    uint16_t tmp;

    switch ( p_ctx->wdr_mode ) {
    case WDR_MODE_LINEAR: // Normal mode
        if ( *int_time_S > p_ctx->vmax_adjust - 2 ) *int_time_S = p_ctx->vmax_adjust - 2;
        if ( *int_time_S < 1 ) *int_time_S = 1;
        tmp = p_ctx->vmax_adjust - *int_time_S - 1;
        if ( p_ctx->int_time_S != tmp ) {
            p_ctx->int_cnt = 2;
            p_ctx->int_time_S = tmp;
        }
        break;
    case WDR_MODE_FS_LIN: // DOL2 Frames
        if ( *int_time_S < 2 ) *int_time_S = 2;
        if ( *int_time_S > p_ctx->max_S ) *int_time_S = p_ctx->max_S;
        if ( *int_time_L < 2 ) *int_time_L = 2;
        if ( *int_time_L > p_ctx->max_L ) *int_time_L = p_ctx->max_L;

        if ( p_ctx->int_time_S != *int_time_S || p_ctx->int_time_L != *int_time_L ) {
            p_ctx->int_cnt = 2;

            p_ctx->int_time_S = *int_time_S;
            p_ctx->int_time_L = *int_time_L;

            p_imx290->shs2 = p_ctx->frame - *int_time_L - 1;
            p_imx290->shs1 = p_imx290->rhs1 - *int_time_S - 1;
        }

        break;
    }
}

static int32_t sensor_ir_cut_set( void *ctx, int32_t ir_cut_state )
{
    sensor_context_t *t_ctx = ctx;
    int ret;
    sensor_bringup_t* sensor_bp = t_ctx->sbp;

    LOG( LOG_ERR, "ir_cut_state = %d", ir_cut_state);
    LOG( LOG_INFO, "entry ir cut" );

   //ir_cut_GPIOZ_7 =1 && ir_cut_GPIOZ_11=0, open ir cut
   //ir_cut_GPIOZ_7 =0 && ir_cut_GPIOZ_11=1, close ir cut
   //ir_cut_srate, 2: no operation

   if (sensor_bp->ir_gname[0] <= 0 && sensor_bp->ir_gname[1] <= 0) {
       pr_err("get gpio id fail\n");
       return 0;
   }

   if (ir_cut_state == 1) {
      ret = pwr_ir_cut_enable(sensor_bp, sensor_bp->ir_gname[1], 0);
      if (ret < 0 )
         pr_err("set power fail\n");

      ret = pwr_ir_cut_enable(sensor_bp, sensor_bp->ir_gname[0], 1);
      if (ret < 0 )
         pr_err("set power fail\n");

      mdelay(500);
      ret = pwr_ir_cut_enable(sensor_bp, sensor_bp->ir_gname[1], 1);
      if (ret < 0 )
         pr_err("set power fail\n");
   } else if (ir_cut_state == 0) {
      ret = pwr_ir_cut_enable(sensor_bp, sensor_bp->ir_gname[1], 1);
      if (ret < 0 )
         pr_err("set power fail\n");

      ret = pwr_ir_cut_enable(sensor_bp, sensor_bp->ir_gname[0], 0);
      if (ret < 0 )
         pr_err("set power fail\n");

      mdelay(500);
      ret = pwr_ir_cut_enable(sensor_bp, sensor_bp->ir_gname[0], 1);
      if (ret < 0 )
         pr_err("set power fail\n");
   }

   LOG( LOG_INFO, "exit ir cut" );

   return 0;
}

static uint32_t sensor_vmax_fps( void *ctx, uint32_t framerate )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;

    if ( framerate == 0 )
        return p_ctx->vmax_fps;

    if (framerate > p_ctx->s_fps )
        return 0;

    sensor_param_t *param = &p_ctx->param;
    uint32_t vmax = ( (( p_ctx->s_fps * 1000) / framerate ) * p_ctx->vmax ) / 1000;
    acamera_sbus_write_u8( p_sbus, 0x3018, vmax & 0xFF );
    acamera_sbus_write_u8( p_sbus, 0x3019, vmax >> 8 );

    if ( p_ctx->wdr_mode == WDR_MODE_LINEAR ) {
        p_ctx->max_L = 2 * vmax - 4;
        param->integration_time_limit = vmax - 2;
        param->integration_time_max = vmax - 2;
    } else {
        p_ctx->max_L = 2 * vmax - 4;
        param->integration_time_limit = p_ctx->max_S;
        param->integration_time_max = p_ctx->max_S;
        param->integration_time_long_max = ( vmax << 1 ) - 256;
        param->lines_per_second = param->lines_per_second << 1;
        p_ctx->frame = vmax << 1;
    }

    p_ctx->vmax_adjust = vmax;
    p_ctx->vmax_fps = framerate;

    LOG(LOG_INFO,"framerate:%d, vmax:%d, p_ctx->max_L:%d, param->integration_time_long_max:%d",
        framerate, vmax, p_ctx->max_L, param->integration_time_long_max);

    return 0;
}

static void sensor_update( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    imx290_private_t *p_imx290 = (imx290_private_t *)p_ctx->sdrv;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;

    if ( p_ctx->int_cnt || p_ctx->gain_cnt ) {
        // ---------- Start Changes -------------
        acamera_sbus_write_u8( p_sbus, 0x3001, 1 );

        // ---------- Analog Gain -------------
        if ( p_ctx->gain_cnt ) {
            p_ctx->gain_cnt--;
            acamera_sbus_write_u8( p_sbus, 0x3014, p_ctx->again[p_ctx->again_delay] );
        }

        // -------- Integration Time ----------
        if ( p_ctx->int_cnt ) {
            p_ctx->int_cnt--;
            switch ( p_ctx->wdr_mode ) {
            case WDR_MODE_LINEAR:
                acamera_sbus_write_u8( p_sbus, 0x3021, ( p_ctx->int_time_S >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x3020, ( p_ctx->int_time_S >> 0 ) & 0xFF );
                break;
            case WDR_MODE_FS_LIN:
                p_imx290->shs2_old = p_imx290->shs2;
                p_imx290->shs1_old = p_imx290->shs1;

                // SHS1
                acamera_sbus_write_u8( p_sbus, 0x3021, ( p_imx290->shs1_old >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x3020, ( p_imx290->shs1_old >> 0 ) & 0xFF );

                // SHS2
                acamera_sbus_write_u8( p_sbus, 0x3025, ( p_imx290->shs2_old >> 8 ) & 0xFF );
                acamera_sbus_write_u8( p_sbus, 0x3024, ( p_imx290->shs2_old >> 0 ) & 0xFF );
                break;
            }
        }

        // ---------- End Changes -------------
        acamera_sbus_write_u8( p_sbus, 0x3001, 0 );
    }
    p_imx290->shs1_old = p_imx290->shs1;
    p_imx290->shs2_old = p_imx290->shs2;
    p_ctx->again[3] = p_ctx->again[2];
    p_ctx->again[2] = p_ctx->again[1];
    p_ctx->again[1] = p_ctx->again[0];
}

static uint16_t sensor_get_id( void *ctx )
{
    /* return that sensor id register does not exist */

    sensor_context_t *p_ctx = ctx;
    uint16_t sensor_id = 0;

    sensor_id |= acamera_sbus_read_u8(&p_ctx->sbus, 0x301e) << 8;
    sensor_id |= acamera_sbus_read_u8(&p_ctx->sbus, 0x301f);

    if (sensor_id != SENSOR_CHIP_ID) {
        LOG(LOG_CRIT, "%s: Failed to read sensor id\n", __func__);
        return 0xFFFF;
    }

    LOG(LOG_INFO, "%s: success to read sensor %x\n", __func__, sensor_id);
    return sensor_id;
}

static void sensor_set_mode( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;
    imx290_private_t *p_imx290 = (imx290_private_t *)p_ctx->sdrv;
    sensor_param_t *param = &p_ctx->param;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    uint8_t setting_num = param->modes_table[mode].num;

    if (initial_sensor ++ >= 1) {
        reset_am_enable(p_ctx->sbp, reset_dts_pin_name, config_sensor_idx, 0);
        sensor_hw_reset_enable();
        system_timer_usleep( 10000 );
        sensor_hw_reset_disable();
        system_timer_usleep( 10000 );
        reset_am_enable(p_ctx->sbp, reset_dts_pin_name, config_sensor_idx, 1);
    }

    if (sensor_get_id(ctx) != SENSOR_CHIP_ID) {
        LOG(LOG_INFO, "%s: check sensor failed\n", __func__);
        return;
    }

    switch ( param->modes_table[mode].wdr_mode ) {
    case WDR_MODE_LINEAR:
        sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, setting_num );
        p_ctx->again_delay = 0;
        param->integration_time_apply_delay = 2;
        param->isp_exposure_channel_delay = 0;
        break;
    case WDR_MODE_FS_LIN:
        p_ctx->again_delay = 0;
        param->integration_time_apply_delay = 2;
        param->isp_exposure_channel_delay = 0;

        if ( param->modes_table[mode].exposures == 2 ) {
#if FS_LIN_1080P
            sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, setting_num);
#else
            sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_IMX290_SEQUENCE_DEFAULT_WDR_720P );
#endif
        } else {
#if FS_LIN_1080P
            sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_IMX290_SEQUENCE_DEFAULT_WDR_1080P );
#else
            sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_IMX290_SEQUENCE_DEFAULT_WDR_720P );
#endif
        }
        break;
    default:
        return;
    }

    uint8_t r = ( acamera_sbus_read_u8( p_sbus, 0x3007 ) >> 4 );
    switch ( r ) {
    case 0: // HD 1080p
        p_ctx->max_L = 2236;
        p_ctx->max_S = 198;
        p_imx290->rhs1 = 201;
        break;
    case 1:
        p_ctx->max_L = 2502;
        p_ctx->max_S = 60;
        p_imx290->rhs1 = 427;
        p_imx290->rhs2 = 494;
        break;
    default:
        // 4- Window cropping from 1080p, Other- Prohibited
        break;
    }

    // Enable syncs on XHS and XVS pins
    //acamera_sbus_write_u8(p_sbus, 0x024b, 0x0A);
    if (param->modes_table[mode].exposures > 1) {
        acamera_sbus_write_u8( p_sbus, 0x3031, ( p_imx290->rhs1 >> 8 ) & 0xFF );
        acamera_sbus_write_u8( p_sbus, 0x3030, ( p_imx290->rhs1 >> 0 ) & 0xFF );
        //acamera_sbus_write_u8( p_sbus, 0x3035, ( p_ctx->rhs2 >> 8 ) & 0xFF );
        //acamera_sbus_write_u8( p_sbus, 0x3034, ( p_ctx->rhs2 >> 0 ) & 0xFF );
    }

    param->active.width = param->modes_table[mode].resolution.width;
    param->active.height = param->modes_table[mode].resolution.height;

    param->total.width = ( (uint16_t)acamera_sbus_read_u8( p_sbus, 0x301D ) << 8 ) | acamera_sbus_read_u8( p_sbus, 0x301C );
    param->total.height = ((uint32_t)acamera_sbus_read_u8( p_sbus, 0x3019 ) << 8 ) | acamera_sbus_read_u8( p_sbus, 0x3018 );

    p_ctx->s_fps = param->modes_table[mode].fps >> 8;
    p_ctx->pixel_clock = param->total.width * param->total.height * p_ctx->s_fps;
    p_ctx->vmax = param->total.height;

    param->lines_per_second = p_ctx->pixel_clock / param->total.width;
    param->pixels_per_line = param->total.width;
    param->integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    if ( param->modes_table[mode].wdr_mode == WDR_MODE_LINEAR ) {
        param->integration_time_limit = p_ctx->vmax - 2;
        param->integration_time_max = p_ctx->vmax - 2;
    } else {
        param->integration_time_limit = p_ctx->max_S;
        param->integration_time_max = p_ctx->max_S;
        if ( param->modes_table[mode].exposures == 2 ) {
            param->integration_time_long_max = ( p_ctx->vmax << 1 ) - 256;
            param->lines_per_second = param->lines_per_second << 1;
            p_ctx->frame = p_ctx->vmax << 1;
        } else {
            param->integration_time_long_max = ( p_ctx->vmax << 2 ) - 256;
            param->lines_per_second = param->lines_per_second >> 2;
            p_ctx->frame = p_ctx->vmax << 2;
        }
    }

    param->sensor_exp_number = param->modes_table[mode].exposures;
    param->mode = mode;
    param->bayer = param->modes_table[mode].bayer;
    p_ctx->wdr_mode = param->modes_table[mode].wdr_mode;
    p_ctx->vmax_adjust = p_ctx->vmax;
    p_ctx->vmax_fps = p_ctx->s_fps;

    //sensor_set_iface(&param->modes_table[mode], p_ctx->win_offset, p_ctx);

    LOG( LOG_INFO, "Output resolution from sensor: %dx%d", param->active.width, param->active.height ); // LOG_NOTICE Causes errors in some projects
}

static const sensor_param_t *sensor_get_parameters( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    return (const sensor_param_t *)&p_ctx->param;
}

static void sensor_disable_isp( void *ctx )
{
}

static uint32_t read_register( void *ctx, uint32_t address )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    return acamera_sbus_read_u8( p_sbus, address );
}

static void write_register( void *ctx, uint32_t address, uint32_t data )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    acamera_sbus_write_u8( p_sbus, address, data );
}

static void stop_streaming( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    p_ctx->streaming_flg = 0;
    p_ctx->dcam_mode = 0;

    acamera_sbus_write_u8( p_sbus, 0x3000, 0x01 );

    reset_sensor_bus_counter();
    sensor_iface_disable(p_ctx);
}

uint32_t write2_reg(uint32_t val, unsigned long addr)
{
    void __iomem *io_addr;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
    io_addr = ioremap(addr, 8);
#else
    io_addr = ioremap_nocache(addr, 8);
#endif
    if (io_addr == NULL) {
        LOG(LOG_ERR, "%s: Failed to ioremap addr\n", __func__);
        return -1;
    }
    __raw_writel(val, io_addr);
    iounmap(io_addr);
    return 0;
}

static void start_streaming( void *ctx )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    sensor_param_t *param = &p_ctx->param;
    sensor_set_iface(&param->modes_table[param->mode], p_ctx->win_offset, p_ctx);
    p_ctx->streaming_flg = 1;
    acamera_sbus_write_u8( p_sbus, 0x3000, 0x00 );
}

static void sensor_test_pattern( void *ctx, uint8_t mode )
{
    sensor_context_t *p_ctx = ctx;
    acamera_sbus_ptr_t p_sbus = &p_ctx->sbus;
    if (mode == 0xff) {
        LOG(LOG_CRIT, "Donot skip initial sensor: %d", initial_sensor);
        initial_sensor += 1;
        return;
    }
    sensor_load_sequence( p_sbus, p_ctx->seq_width, p_sensor_data, SENSOR_IMX290_SEQUENCE_DEFAULT_TEST_PATTERN );
}

static void sensor_dcam_mode( void *ctx, int32_t mode )
{
    sensor_context_t *p_ctx = ctx;
    LOG(LOG_CRIT, "imx290 set dcam mode:%d", mode);
    p_ctx->dcam_mode = mode;
}

void sensor_deinit_imx290( void *ctx )
{
    sensor_context_t *t_ctx = ctx;
    reset_sensor_bus_counter();
    acamera_sbus_deinit(&t_ctx->sbus,  sbus_i2c);
    if (t_ctx != NULL && t_ctx->sbp != NULL)
        gp_pl_am_disable(t_ctx->sbp, "mclk_0");
}

static sensor_context_t *sensor_global_parameter(void* sbp)
{
// Local sensor data structure
    int ret;
    sensor_bringup_t* sensor_bp = (sensor_bringup_t*) sbp;
    sensor_ctx.sbp = sbp;
    sensor_ctx.sdrv = &imx290_ctx;

/*
    write2_reg(0x01800000, 0xfe007cc4);
    write2_reg(0x00001100, 0xfe007cc8);
    write2_reg(0x10022300, 0xfe007ccc);
    write2_reg(0x00300000, 0xfe007cd0);
    write2_reg(0x00089688, 0xfe007cd8);
    write2_reg(0x01f18863, 0xfe007cc0);
    write2_reg(0x11f18863, 0xfe007cc0);
    write2_reg(0x15f18863, 0xfe007cc0);
    write2_reg(0x00001120, 0xfe007cc8);

    write2_reg(0x11300000, 0xfe000428);
    write2_reg(0x9000, 0xfe007cd4);
*/
#if PLATFORM_C308X
        sensor_bp->pin = devm_pinctrl_get_select(sensor_bp->dev,"mclk37_pin");
        if (IS_ERR(sensor_bp->pin)) {
                LOG(LOG_CRIT, "set sensor_bp->pin %s error\n","mclk37_pin");
        } else {
                LOG(LOG_INFO, "set sensor_bp->pin %s: %p\n",
                        "mclk37_pin", sensor_bp->pin);
        }
#endif

#if NEED_CONFIG_BSP
    ret = gp_pl_am_enable(sensor_bp, "mclk_0", 37125000);
    if (ret < 0 )
        pr_info("set mclk fail\n");
    udelay(30);
#if PLATFORM_T7
    pwr_am_enable(sensor_bp, pwr_dts_pin_name, config_sensor_idx, 0);
#endif
    ret = reset_am_enable(sensor_bp, reset_dts_pin_name, config_sensor_idx, 1);
    if (ret < 0 )
       pr_info("set reset fail\n");
#endif

    sensor_ctx.sbus.mask = SBUS_MASK_SAMPLE_8BITS | SBUS_MASK_ADDR_16BITS | SBUS_MASK_ADDR_SWAP_BYTES;
    sensor_ctx.sbus.control = 0;
    sensor_ctx.sbus.bus = 0;
    sensor_ctx.sbus.device = SENSOR_DEV_ADDRESS;
    acamera_sbus_init( &sensor_ctx.sbus, sbus_i2c );

    // Initial local parameters
    sensor_ctx.address = SENSOR_DEV_ADDRESS;
    sensor_ctx.seq_width = 1;
    sensor_ctx.streaming_flg = 0;
    sensor_ctx.again[0] = 0;
    sensor_ctx.again[1] = 0;
    sensor_ctx.again[2] = 0;
    sensor_ctx.again[3] = 0;
    sensor_ctx.again_limit = AGAIN_MAX_DB + DGAIN_MAX_DB;
    sensor_ctx.pixel_clock = 148500000;

    sensor_ctx.param.again_accuracy = 1 << LOG2_GAIN_SHIFT;
    sensor_ctx.param.sensor_exp_number = 1;
    sensor_ctx.param.again_log2_max = ( ( AGAIN_MAX_DB + DGAIN_MAX_DB ) << LOG2_GAIN_SHIFT ) / 20;
    sensor_ctx.param.dgain_log2_max = 0;
    sensor_ctx.param.integration_time_apply_delay = 2;
    sensor_ctx.param.isp_exposure_channel_delay = 0;
    sensor_ctx.param.modes_table = supported_modes;
    sensor_ctx.param.modes_num = array_size_s( supported_modes );
    sensor_ctx.param.sensor_ctx = &sensor_ctx;
    sensor_ctx.param.isp_context_seq.sequence = p_isp_data;
    sensor_ctx.param.isp_context_seq.seq_num= SENSOR_IMX290_CONTEXT_SEQ;
    sensor_ctx.param.isp_context_seq.seq_table_max = array_size_s( isp_seq_table );

    sensor_ctx.again_delay = 0;
    sensor_ctx.param.integration_time_apply_delay = 2;
    sensor_ctx.param.isp_exposure_channel_delay = 0;
    sensor_ctx.s_fps = 30;
    sensor_ctx.vmax = 1125;
    sensor_ctx.vmax_fps = sensor_ctx.s_fps;
    sensor_ctx.vmax_adjust = sensor_ctx.vmax;
    sensor_ctx.param.active.width = 1920;
    sensor_ctx.param.active.height = 1080;
    sensor_ctx.max_L = 2236;
    sensor_ctx.max_S = 98;
    imx290_ctx.rhs1 = 201;
    sensor_ctx.param.total.width = 0x1130;
    sensor_ctx.param.lines_per_second = sensor_ctx.pixel_clock / sensor_ctx.param.total.width;
    sensor_ctx.param.total.height = (uint16_t)sensor_ctx.vmax;
    sensor_ctx.param.pixels_per_line = sensor_ctx.param.total.width;
    sensor_ctx.param.integration_time_min = SENSOR_MIN_INTEGRATION_TIME;
    sensor_ctx.param.integration_time_limit = sensor_ctx.vmax - 2;
    sensor_ctx.param.integration_time_max = sensor_ctx.vmax - 2;
    sensor_ctx.param.sensor_exp_number = 1;
    sensor_ctx.param.mode = 0;
    sensor_ctx.wdr_mode = DOL_NON;
    sensor_ctx.param.bayer = BAYER_RGGB;
    if ((acamera_sbus_read_u8(&sensor_ctx.sbus, 0x300c) & 0x01) == 1) {
        sensor_ctx.param.sensor_exp_number = 2;
        sensor_ctx.param.mode = 1;
        sensor_ctx.wdr_mode = WDR_MODE_FS_LIN;
        sensor_ctx.param.integration_time_long_max = 1125 * 2 - 256;
        sensor_ctx.param.integration_time_limit = 198;
    }

    sensor_ctx.win_offset.offset_x = 12;
    sensor_ctx.win_offset.offset_y = 0;
#if PLATFORM_G12B
    sensor_ctx.win_offset.long_offset = 0xa;
    sensor_ctx.win_offset.short_offset = 0x1d;
#elif PLATFORM_C308X || PLATFORM_C305X
    sensor_ctx.win_offset.long_offset = 0x8;
    sensor_ctx.win_offset.short_offset = 0x8;
#endif

    sensor_ctx.cam_isp_path = CAM0_ACT;
    sensor_ctx.cam_fe_path = FRONTEND0_IO;

    return &sensor_ctx;
}

//--------------------Initialization------------------------------------------------------------
void sensor_init_imx290( void **ctx, sensor_control_t *ctrl, void* sbp)
{
    *ctx = sensor_global_parameter(sbp);

    ctrl->alloc_analog_gain = sensor_alloc_analog_gain;
    ctrl->alloc_digital_gain = sensor_alloc_digital_gain;
    ctrl->alloc_integration_time = sensor_alloc_integration_time;
    ctrl->ir_cut_set= sensor_ir_cut_set;
    ctrl->sensor_update = sensor_update;
    ctrl->set_mode = sensor_set_mode;
    ctrl->get_id = sensor_get_id;
    ctrl->get_parameters = sensor_get_parameters;
    ctrl->disable_sensor_isp = sensor_disable_isp;
    ctrl->read_sensor_register = read_register;
    ctrl->write_sensor_register = write_register;
    ctrl->start_streaming = start_streaming;
    ctrl->stop_streaming = stop_streaming;
    ctrl->sensor_test_pattern = sensor_test_pattern;
    ctrl->vmax_fps = sensor_vmax_fps;
    ctrl->dcam_mode = sensor_dcam_mode;
    // Reset sensor during initialization
    sensor_hw_reset_enable();
    system_timer_usleep( 1000 ); // reset at least 1 ms
    sensor_hw_reset_disable();
    system_timer_usleep( 1000 );
}

int sensor_detect_imx290( void* sbp)
{
    int ret = 0;
    sensor_bringup_t* sensor_bp = (sensor_bringup_t*) sbp;
    sensor_ctx.sbp = sbp;

#if NEED_CONFIG_BSP
    ret = gp_pl_am_enable(sensor_bp, "mclk_0", 37125000);
    if (ret < 0 )
        pr_info("set mclk fail\n");
    udelay(30);

#if PLATFORM_T7
    pwr_am_enable(sensor_bp, pwr_dts_pin_name, config_sensor_idx, 0);
#endif

    ret = reset_am_enable(sensor_bp, reset_dts_pin_name, config_sensor_idx, 1);
    if (ret < 0 )
       pr_info("set reset fail\n");
#endif

    sensor_ctx.sbus.mask = SBUS_MASK_SAMPLE_8BITS | SBUS_MASK_ADDR_16BITS | SBUS_MASK_ADDR_SWAP_BYTES;
    sensor_ctx.sbus.control = 0;
    sensor_ctx.sbus.bus = 0;
    sensor_ctx.sbus.device = SENSOR_DEV_ADDRESS;
    acamera_sbus_init( &sensor_ctx.sbus, sbus_i2c );

    ret = 0;
    if (sensor_get_id(&sensor_ctx) == 0xFFFF)
        ret = -1;
    else
        pr_info("sensor_detect_imx290:%d\n", sensor_get_id(&sensor_ctx));

    acamera_sbus_deinit(&sensor_ctx.sbus,  sbus_i2c);
    gp_pl_am_disable(sensor_bp, "mclk_0");
    return ret;
}

//********************CONSTANT SECTION END*********************************************
//*************************************************************************************
