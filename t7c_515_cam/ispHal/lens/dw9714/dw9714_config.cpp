/*
 * Copyright (c) 2018 Amlogic, Inc. All rights reserved.
 *
 * This source code is subject to the terms and conditions defined in the
 * file 'LICENSE' which is part of this source code package.
 *
 * Description:
 */
#define LOG_TAG "dw9714Cfg"

#include <sys/ioctl.h>
#include "dw9714_api.h"
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
#include <logs.h>

#define MIN_STEP 64

struct dw9714_info {
	struct media_entity  *ent;
	LENS_PARAM_T param;
};

static struct dw9714_info vcm_info;

static struct dw9714_info *dw9714_get_info(void)
{
	return &vcm_info;
}

void vcm_set_ent_dw9714(struct media_entity *ent)
{
	struct dw9714_info *info = dw9714_get_info();
	LENS_PARAM_T *param = &info->param;

	info->ent = ent;

	param->min_step = MIN_STEP;
}

void vcm_set_pos_dw9714(uint32_t ctx, uint16_t position)
{
    INFO("CY: dw9714w set pos: %u\n", position);
    int ret = 0;

	struct v4l2_ext_control pos;
	struct dw9714_info *info = dw9714_get_info();

	memset(&pos, 0, sizeof(pos));
	pos.id = V4L2_CID_FOCUS_ABSOLUTE;
	pos.value = ((position) / info->param.min_step) & 0x3FF;
	//pos.value = 45;
	ret = v4l2_subdev_set_ctrls(info->ent, &pos, 1);
	if (ret < 0)
		ERR("dw9714w set pos fail");
	else
		INFO("dw9714w set pos success");
}

uint8_t vcm_is_moving_dw9714(uint32_t ctx)
{
	INFO("CY: dw9714w moving\n");
	int ret = 0;
	struct dw9714_info *info = dw9714_get_info();
	struct v4l2_control ctrl;
	ctrl.id = V4L2_CID_AML_LENS_MOVING;
	ret = ioctl(info->ent->fd, VIDIOC_G_CTRL, &ctrl);
	if (ret < 0) {
		ERR("CY: get moving state fail");
		return 1;
	}
	return ctrl.value;
}

const LENS_PARAM_T *vcm_get_param_dw9714(uint32_t ctx)
{
	struct dw9714_info *info = dw9714_get_info();

	INFO("CY: dw9714 get param\n");

	return &info->param;
}

