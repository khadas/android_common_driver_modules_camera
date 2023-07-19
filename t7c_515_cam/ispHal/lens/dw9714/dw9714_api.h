/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */

#ifndef __DW9714_API_H__
#define __DW9714_API_H__

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

#include "mediactl.h"
#include "v4l2subdev.h"
#include "v4l2videodev.h"
#include "mediaApi.h"

void vcm_set_ent_dw9714(struct media_entity *ent);
void vcm_set_pos_dw9714(uint32_t ctx, uint16_t position);
uint8_t vcm_is_moving_dw9714(uint32_t ctx);
const LENS_PARAM_T *vcm_get_param_dw9714(uint32_t ctx);

#endif

