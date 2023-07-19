/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */
#define LOG_TAG "ov08a10Cfg"

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <semaphore.h>

#include "aml_isp_api.h"

#include "ov08a10_sdr_calibration.h"
#include "ov08a10_wdr_calibration.h"
#include "ov08a10_api.h"
#include "logs.h"


typedef struct
{
    int  enWDRMode;
    ALG_SENSOR_DEFAULT_S snsAlgInfo;
    struct media_entity  * sensor_ent;
} ISP_SNS_STATE_S;

static ISP_SNS_STATE_S sensor;

void cmos_set_sensor_entity_ov08a10(struct media_entity * sensor_ent, int wdr)
{
    sensor.sensor_ent = sensor_ent;
    sensor.enWDRMode = wdr;
}

void cmos_get_sensor_calibration_ov08a10(struct media_entity *sensor_ent, aisp_calib_info_t *calib)
{
    if (sensor.enWDRMode == 1)
        Ov08a10WdrCalibration::dynamic_wdr_calibrations_init_ov08a10(calib);
    else
        Ov08a10SdrCalibration::dynamic_sdr_calibrations_init_ov08a10(calib);
}

int cmos_get_ae_default_ov08a10(int ViPipe, ALG_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    INFO("cmos_get_ae_default\n");

    sensor.snsAlgInfo.active.width = 3840;
    sensor.snsAlgInfo.active.height = 2160;
    sensor.snsAlgInfo.fps = 30;
    sensor.snsAlgInfo.sensor_exp_number = 1;
    sensor.snsAlgInfo.bits = 10;

    //sensor.snsAlgInfo.sensor_gain_number = 1;
    sensor.snsAlgInfo.total.width = 0x818; //hts
    sensor.snsAlgInfo.total.height = 0x90A; //vts

    sensor.snsAlgInfo.lines_per_second = sensor.snsAlgInfo.total.height*30;
    sensor.snsAlgInfo.pixels_per_line = sensor.snsAlgInfo.total.width;

    if (sensor.enWDRMode == 1) {

        sensor.snsAlgInfo.integration_time_min = 4 << SHUTTER_TIME_SHIFT;
        sensor.snsAlgInfo.integration_time_max = sensor.snsAlgInfo.total.height << SHUTTER_TIME_SHIFT;
        sensor.snsAlgInfo.integration_time_long_max = sensor.snsAlgInfo.total.height << SHUTTER_TIME_SHIFT;
        sensor.snsAlgInfo.integration_time_limit = sensor.snsAlgInfo.total.height << SHUTTER_TIME_SHIFT;

    } else {
        sensor.snsAlgInfo.integration_time_min = 1<<SHUTTER_TIME_SHIFT;
        sensor.snsAlgInfo.integration_time_max = sensor.snsAlgInfo.total.height<<SHUTTER_TIME_SHIFT;
        sensor.snsAlgInfo.integration_time_long_max = sensor.snsAlgInfo.total.height<<SHUTTER_TIME_SHIFT;
        sensor.snsAlgInfo.integration_time_limit = sensor.snsAlgInfo.total.height<<SHUTTER_TIME_SHIFT;
    }

    sensor.snsAlgInfo.again_log2_max = (4)<<(LOG2_GAIN_SHIFT); //2^4 = 16, 15.5
    sensor.snsAlgInfo.again_high_log2_max = (4)<<(LOG2_GAIN_SHIFT);
    sensor.snsAlgInfo.dgain_log2_max = 0;
    sensor.snsAlgInfo.dgain_high_log2_max = 0;
    sensor.snsAlgInfo.dgain_high_accuracy_fmt = 0;
    sensor.snsAlgInfo.dgain_high_accuracy = 1;
    sensor.snsAlgInfo.dgain_accuracy_fmt = 0;
    sensor.snsAlgInfo.dgain_accuracy = 1;
    sensor.snsAlgInfo.again_high_accuracy_fmt = 1;
    sensor.snsAlgInfo.again_high_accuracy = (1<<(LOG2_GAIN_SHIFT))/20;
    sensor.snsAlgInfo.again_accuracy_fmt = 1;
    sensor.snsAlgInfo.expos_lines = (0x8E5<<(LOG2_GAIN_SHIFT));//check value
    sensor.snsAlgInfo.again_accuracy = (1<<(LOG2_GAIN_SHIFT))/20;
    sensor.snsAlgInfo.expos_accuracy = (1<<(SHUTTER_TIME_SHIFT));
    sensor.snsAlgInfo.sexpos_accuracy = (1<<(SHUTTER_TIME_SHIFT));
    sensor.snsAlgInfo.vsexpos_accuracy = (1<<(SHUTTER_TIME_SHIFT));
    sensor.snsAlgInfo.vvsexpos_accuracy = (1<<(SHUTTER_TIME_SHIFT));
    sensor.snsAlgInfo.gain_apply_delay = 0;
    sensor.snsAlgInfo.integration_time_apply_delay = 0;

    memcpy(pstAeSnsDft, &sensor.snsAlgInfo, sizeof(ALG_SENSOR_DEFAULT_S));

    return 0;
}

static int aisp_math_exp2( int64_t val, int32_t shift_in, int32_t shift_out )
{
    uint32_t fract_part = (((uint32_t)val) & ( ( 1 << shift_in ) - 1 ) );
    uint32_t int_part = ((uint32_t)val) >> shift_in;
    uint32_t res, tmp;
    uint32_t pow_lut[33] = {
    1073741824, 1097253708, 1121280436, 1145833280, 1170923762, 1196563654, 1222764986, 1249540052,
    1276901417, 1304861917, 1333434672, 1362633090, 1392470869, 1422962010, 1454120821, 1485961921,
    1518500250, 1551751076, 1585730000, 1620452965, 1655936265, 1692196547, 1729250827, 1767116489,
    1805811301, 1845353420, 1885761398, 1927054196, 1969251188, 2012372174, 2056437387, 2101467502,
    2147483648};
    if ( shift_in <= 5 ) {
        uint32_t lut_index = fract_part << ( 5 - shift_in );
        res = pow_lut[lut_index] >> ( 30 - shift_out - int_part );
        return res;
    } else {
        uint32_t lut_index = fract_part >> ( shift_in - 5 );
        uint32_t lut_fract = fract_part & ( ( 1 << ( shift_in - 5 ) ) - 1 );
        uint32_t a = pow_lut[lut_index];
        uint32_t b = pow_lut[lut_index + 1];
        res = ( (uint64_t)( b - a ) * lut_fract ) >> ( shift_in - 5 );
        tmp =  ( 30 - shift_out - int_part ) - 1;
        res = ( res + a + (1<<tmp) ) >> ( 30 - shift_out - int_part );

        return ((int64_t)res);
    }
}

void cmos_again_calc_table_ov08a10(int ViPipe, uint32_t *pu32AgainLin, uint32_t *pu32AgainDb)
{
    //INFO("cmos_again_calc_table: %d, %d\n", *pu32AgainLin, *pu32AgainDb);
    uint32_t again_reg;
    //uint32_t u32AgainDb;

    //u32AgainDb = *pu32AgainDb;
    //u32AgainDb = ((u32AgainDb*20)>>LOG2_GAIN_SHIFT);
    again_reg = aisp_math_exp2( *pu32AgainLin, SHUTTER_TIME_SHIFT, 8 );

    //again_reg = (uint32_t)(u32AgainDb);
       INFO("again_reg1 = %d \n",again_reg);
    if (again_reg > 1984) //72dB, 0.3dB step.
        again_reg = 1984;

    if (sensor.snsAlgInfo.u32AGain[0] != again_reg) {
        sensor.snsAlgInfo.u16GainCnt = sensor.snsAlgInfo.gain_apply_delay + 1;
        sensor.snsAlgInfo.u32AGain[0] = again_reg;
    }

}

void cmos_dgain_calc_table_ov08a10(int ViPipe, uint32_t *pu32DgainLin, uint32_t *pu32DgainDb)
{
    //INFO("cmos_dgain_calc_table: %d, %d\n", *pu32DgainLin, *pu32DgainDb);
}

void cmos_inttime_calc_table_ov08a10(int ViPipe, uint32_t pu32ExpL, uint32_t pu32ExpS, uint32_t pu32ExpVS, uint32_t pu32ExpVVS)
{
    //INFO("cmos_inttime_calc_table: %d, %d, %d, %d\n", pu32ExpL, pu32ExpS, pu32ExpVS, pu32ExpVVS);
    uint32_t shutter_time_lines = pu32ExpL >> SHUTTER_TIME_SHIFT;
    uint32_t shutter_time_line_each_frame = sensor.snsAlgInfo.total.height;

    uint32_t shutter_time_lines_short = pu32ExpS >> SHUTTER_TIME_SHIFT;

    INFO("expo: %d\n", shutter_time_lines);
    shutter_time_lines = shutter_time_lines_short;
    if (sensor.enWDRMode == 0) {
        if (shutter_time_lines > sensor.snsAlgInfo.total.height )
            shutter_time_lines = sensor.snsAlgInfo.total.height;

        if (shutter_time_lines < 8)
            shutter_time_lines = 8;
    } else {
        if (shutter_time_lines_short < 8)
            shutter_time_lines_short = 8;
        shutter_time_lines_short = 225 - shutter_time_lines_short - 1;
        shutter_time_lines = shutter_time_line_each_frame * 2  - shutter_time_lines - 1;
    }

    if (sensor.snsAlgInfo.u32Inttime[0][0] != shutter_time_lines || sensor.snsAlgInfo.u32Inttime[1][0] != shutter_time_lines_short) {
        sensor.snsAlgInfo.u16IntTimeCnt = sensor.snsAlgInfo.integration_time_apply_delay + 1;
        sensor.snsAlgInfo.u32Inttime[0][0] = shutter_time_lines;
        sensor.snsAlgInfo.u32Inttime[1][0] = shutter_time_lines_short;
    }
}

void cmos_fps_set_ov08a10(int ViPipe, float f32Fps, ALG_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    INFO("cmos_fps_set: %f\n", f32Fps);
}

void cmos_alg_update_ov08a10(int ViPipe)
{
    uint32_t shutter_time_lines = 0;//, shutter_time_lines_short = 0;
    uint32_t i = 0;

    if ( sensor.snsAlgInfo.u16GainCnt || sensor.snsAlgInfo.u16IntTimeCnt ) {
        if ( sensor.snsAlgInfo.u16GainCnt ) {
            sensor.snsAlgInfo.u16GainCnt--;
            struct v4l2_ext_control gain;
            gain.id = V4L2_CID_GAIN;
            gain.value = sensor.snsAlgInfo.u32AGain[sensor.snsAlgInfo.gain_apply_delay];
            INFO("gain value = %d \n",gain.value);
            v4l2_subdev_set_ctrls(sensor.sensor_ent, &gain, 1);
        }

        // -------- Integration Time ----------
        if ( sensor.snsAlgInfo.u16IntTimeCnt ) {
            sensor.snsAlgInfo.u16IntTimeCnt--;
            shutter_time_lines = sensor.snsAlgInfo.u32Inttime[0][sensor.snsAlgInfo.integration_time_apply_delay];
            if (sensor.enWDRMode == 0) {
                struct v4l2_ext_control expo;
                expo.id = V4L2_CID_EXPOSURE;
                expo.value = shutter_time_lines;
                INFO("expo.value = %d \n",expo.value);
                v4l2_subdev_set_ctrls(sensor.sensor_ent, &expo, 1);
            }

            if (sensor.enWDRMode) {
                //shutter_time_lines_short = sensor.snsAlgInfo.u32Inttime[1][sensor.snsAlgInfo.integration_time_apply_delay];
                //ov08a10_write_register(ViPipe, 0x3020, shutter_time_lines_short & 0xff);
                //ov08a10_write_register(ViPipe, 0x3021, (shutter_time_lines_short>>8) & 0xff);
                //ov08a10_write_register(ViPipe, 0x3024, shutter_time_lines&0xff);
                //ov08a10_write_register(ViPipe, 0x3025, (shutter_time_lines>>8) & 0xff);
                //INFO("sensor expo: %d, %d\n", shutter_time_lines, shutter_time_lines_short);
            }
        }
    }

    for ( i = 3; i > 0; i --) {
        sensor.snsAlgInfo.u32AGain[i] = sensor.snsAlgInfo.u32AGain[i - 1];
        sensor.snsAlgInfo.u32Inttime[0][i] = sensor.snsAlgInfo.u32Inttime[0][i - 1];
        sensor.snsAlgInfo.u32Inttime[1][i] = sensor.snsAlgInfo.u32Inttime[1][i - 1];
    }

}
