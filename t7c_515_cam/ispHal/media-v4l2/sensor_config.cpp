/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */
#define LOG_TAG "sensorConfig"

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

#include "sensor_config.h"

#include "imx290_api.h"
#include "ov08a10_api.h"
#include "imx415_api.h"
#include "logs.h"




#define ARRAY_SIZE(array)   (sizeof(array) / sizeof((array)[0]))

struct sensorConfig imx290Cfg = {
    imx290Cfg.expFunc.pfn_cmos_fps_set = cmos_fps_set_imx290,
    imx290Cfg.expFunc.pfn_cmos_get_alg_default = cmos_get_ae_default_imx290,
    imx290Cfg.expFunc.pfn_cmos_alg_update = cmos_alg_update_imx290,
    imx290Cfg.expFunc.pfn_cmos_again_calc_table = cmos_again_calc_table_imx290,
    imx290Cfg.expFunc.pfn_cmos_dgain_calc_table = cmos_dgain_calc_table_imx290,
    imx290Cfg.expFunc.pfn_cmos_inttime_calc_table = cmos_inttime_calc_table_imx290,
    imx290Cfg.cmos_set_sensor_entity = cmos_set_sensor_entity_imx290,
    imx290Cfg.cmos_get_sensor_calibration = cmos_get_sensor_calibration_imx290,
    imx290Cfg.sensorWidth      = 1920,
    imx290Cfg.sensorHeight     = 1080,
    imx290Cfg.sensorName       = "imx290",
    imx290Cfg.wdrFormat        = MEDIA_BUS_FMT_SRGGB10_1X10,
    imx290Cfg.sdrFormat        = MEDIA_BUS_FMT_SRGGB12_1X12,
    imx290Cfg.type             = sensor_raw,
};

struct sensorConfig imx415Cfg = {
    imx415Cfg.expFunc.pfn_cmos_fps_set = cmos_fps_set_imx415,
    imx415Cfg.expFunc.pfn_cmos_get_alg_default = cmos_get_ae_default_imx415,
    imx415Cfg.expFunc.pfn_cmos_alg_update = cmos_alg_update_imx415,
    imx415Cfg.expFunc.pfn_cmos_again_calc_table = cmos_again_calc_table_imx415,
    imx415Cfg.expFunc.pfn_cmos_dgain_calc_table = cmos_dgain_calc_table_imx415,
    imx415Cfg.expFunc.pfn_cmos_inttime_calc_table = cmos_inttime_calc_table_imx415,
    imx415Cfg.cmos_set_sensor_entity = cmos_set_sensor_entity_imx415,
    imx415Cfg.cmos_get_sensor_calibration = cmos_get_sensor_calibration_imx415,
    imx415Cfg.sensorWidth      = 3840,
    imx415Cfg.sensorHeight     = 2160,
    imx415Cfg.sensorName       = "imx415",
    imx415Cfg.wdrFormat        = MEDIA_BUS_FMT_SRGGB10_1X10,
    imx415Cfg.sdrFormat        = MEDIA_BUS_FMT_SRGGB12_1X12,
    imx415Cfg.type             = sensor_raw,
};

struct sensorConfig ov08a10Cfg = {
    ov08a10Cfg.expFunc.pfn_cmos_fps_set = cmos_fps_set_ov08a10,
    ov08a10Cfg.expFunc.pfn_cmos_get_alg_default = cmos_get_ae_default_ov08a10,
    ov08a10Cfg.expFunc.pfn_cmos_alg_update = cmos_alg_update_ov08a10,
    ov08a10Cfg.expFunc.pfn_cmos_again_calc_table = cmos_again_calc_table_ov08a10,
    ov08a10Cfg.expFunc.pfn_cmos_dgain_calc_table = cmos_dgain_calc_table_ov08a10,
    ov08a10Cfg.expFunc.pfn_cmos_inttime_calc_table = cmos_inttime_calc_table_ov08a10,
    ov08a10Cfg.cmos_set_sensor_entity = cmos_set_sensor_entity_ov08a10,
    ov08a10Cfg.cmos_get_sensor_calibration = cmos_get_sensor_calibration_ov08a10,
    ov08a10Cfg.sensorWidth      = 3840,
    ov08a10Cfg.sensorHeight     = 2160,
    ov08a10Cfg.sensorName       = "ov08a10",
    ov08a10Cfg.wdrFormat        = MEDIA_BUS_FMT_SBGGR10_1X10,
    ov08a10Cfg.sdrFormat        = MEDIA_BUS_FMT_SBGGR10_1X10,
    ov08a10Cfg.type             = sensor_raw,
};

struct sensorConfig *supportedCfgs[] = {
    &imx290Cfg,
    &imx415Cfg,
    &ov08a10Cfg,
};

struct sensorConfig *matchSensorConfig(media_stream_t *stream) {
    for (int i = 0; i < ARRAY_SIZE(supportedCfgs); i++) {
        if (strstr(stream->sensor_ent_name, supportedCfgs[i]->sensorName)) {
            return supportedCfgs[i];
        }
    }
    ERR("fail to match sensorConfig");
    return nullptr;
}

struct sensorConfig *matchSensorConfig(const char* sensorEntityName) {
    for (int i = 0; i < ARRAY_SIZE(supportedCfgs); i++) {
        if (strstr(sensorEntityName, supportedCfgs[i]->sensorName)) {
            return supportedCfgs[i];
        }
    }
    ERR("fail to match sensorConfig %s", sensorEntityName);
    return nullptr;
}

void cmos_sensor_control_cb(struct sensorConfig *cfg, ALG_SENSOR_EXP_FUNC_S *stSnsExp)
{
    stSnsExp->pfn_cmos_alg_update = cfg->expFunc.pfn_cmos_alg_update;
    stSnsExp->pfn_cmos_get_alg_default = cfg->expFunc.pfn_cmos_get_alg_default;
    stSnsExp->pfn_cmos_again_calc_table = cfg->expFunc.pfn_cmos_again_calc_table;
    stSnsExp->pfn_cmos_dgain_calc_table = cfg->expFunc.pfn_cmos_dgain_calc_table;
    stSnsExp->pfn_cmos_inttime_calc_table = cfg->expFunc.pfn_cmos_inttime_calc_table;
    stSnsExp->pfn_cmos_fps_set = cfg->expFunc.pfn_cmos_fps_set;
}

void cmos_set_sensor_entity(struct sensorConfig *cfg, struct media_entity *sensor_ent, int wdr)
{
    (cfg->cmos_set_sensor_entity)(sensor_ent, wdr);
}

void cmos_get_sensor_calibration(struct sensorConfig *cfg, struct media_entity * sensor_ent, aisp_calib_info_t *calib)

{
    (cfg->cmos_get_sensor_calibration)(sensor_ent, calib);
}

