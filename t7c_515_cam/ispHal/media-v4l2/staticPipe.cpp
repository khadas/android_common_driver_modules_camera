/*
 * Copyright (c) 2021 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */

#define LOG_TAG "staticPipe"

#include <cstdlib>
#include <vector>

#include "staticPipe.h"
#include "logs.h"



namespace android {

int staticPipe::fetchPipeMaxResolution(media_stream_t *stream, uint32_t& width, uint32_t &height) {
    auto cfg = matchSensorConfig(stream);
    if (cfg) {
        width = cfg->sensorWidth;
        height = cfg->sensorHeight;
        INFO("find matched sensor configs %dx%d", width, height);
        return 0;
    }
    ERR("do not find matched sensor configs");
    return -1;
}

int staticPipe::fetchSensorFormat(media_stream_t *stream, int hdrEnable) {
    auto cfg = matchSensorConfig(stream);
    if (cfg) {
        return hdrEnable ? cfg->wdrFormat : cfg->sdrFormat;
    }
    ERR("do not find matched");
    return -1;
}
sensorType staticPipe::fetchSensorType(media_stream_t * stream) {
    auto cfg = matchSensorConfig(stream);
    if (cfg) {
        return cfg->type;
    }
    return sensor_NULL;
}
}
