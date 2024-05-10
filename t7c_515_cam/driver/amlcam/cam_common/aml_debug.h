/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Copyright (c) 2021 Amlogic, Inc. All rights reserved.
 */

#ifndef _AML_LOG_H_
#define _AML_LOG_H_

#include <stdarg.h>
#include <linux/printk.h>

#define AML_CAM_LOG_LEVEL_DEBUG 1

extern unsigned int aml_cam_log_level;
extern int debug_test_mipi_reset;

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "aml-cam: " fmt

#define aml_cam_log_info(fmt, ...) \
	pr_info(fmt, ##__VA_ARGS__)

#define aml_cam_log_err(fmt, ...) \
	pr_err(fmt, ##__VA_ARGS__)

#define aml_cam_log_dbg(fmt, ...) \
	do { \
		if ((aml_cam_log_level & 0xff) >= AML_CAM_LOG_LEVEL_DEBUG) { \
			pr_info(fmt, ##__VA_ARGS__); \
		} \
	} while (0)

#endif
