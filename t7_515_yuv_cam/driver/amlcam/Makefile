# SPDX-License-Identifier: (GPL-2.0+ OR MIT)

M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

obj-m += amlogic-camera-yuv.o

DRV_SRC := $(wildcard $(M_PATH)/*.c) \
			$(wildcard $(M_PATH)/cam_common/*.c) \
			$(wildcard $(M_PATH)/cam_adapter/*.c) \
			$(wildcard $(M_PATH)/cam_adapter/*/*.c) \
			$(wildcard $(M_PATH)/cam_csiphy/*.c) \
			$(wildcard $(M_PATH)/cam_csiphy/*/*.c)

DRV_SRC_NAME := $(DRV_SRC:$(M_PATH)/%=%)

DRV_SRC_OBJ := $(DRV_SRC_NAME:.c=.o)

PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"amlcam: \" fmt"
ccflags-y += $(PR_FMT_DEFINE)

ccflags-y := -I$(M_PATH)/ -I$(M_PATH)/cam_common -Wno-implicit-fallthrough

ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable
ccflags-y += -Wno-declaration-after-statement

#
#ccflags-y += -DDEBUG_TEST_MIPI_RESET=1
#

#ccflags-y += -DDUMP_GE2D_IN=1
#ccflags-y += -DDUMP_GE2D_OUT=1

amlogic-camera-yuv-y := $(DRV_SRC_OBJ)
