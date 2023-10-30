# SPDX-License-Identifier: (GPL-2.0+ OR MIT)

M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

obj-m += amlogic-yuvcam-droid.o
#obj-m += amlogic-yuvcam-rtos.o

COMMON_DRV_SRC :=  $(wildcard $(M_PATH)/cam_common/*.c) \
			$(wildcard $(M_PATH)/cam_adapter/*.c) \
			$(wildcard $(M_PATH)/cam_adapter/*/*.c) \
			$(wildcard $(M_PATH)/cam_csiphy/*.c) \
			$(wildcard $(M_PATH)/cam_csiphy/*/*.c)

RTOS_DRV_SRC := $(COMMON_DRV_SRC) \
             $(wildcard $(M_PATH)/*_rtos.c)

DROID_DRV_SRC := $(COMMON_DRV_SRC) \
             $(wildcard $(M_PATH)/*_droid.c)

RTOS_DRV_SRC_NAME := $(RTOS_DRV_SRC:$(M_PATH)/%=%)
DROID_DRV_SRC_NAME := $(DROID_DRV_SRC:$(M_PATH)/%=%)


RTOS_DRV_SRC_OBJ := $(RTOS_DRV_SRC_NAME:.c=.o)
DROID_DRV_SRC_OBJ := $(DROID_DRV_SRC_NAME:.c=.o)


PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"amlcam: \" fmt"
ccflags-y += $(PR_FMT_DEFINE)

ccflags-y := -I$(M_PATH)/ -I$(M_PATH)/cam_common -Wno-implicit-fallthrough

ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable
ccflags-y += -Wno-declaration-after-statement

#
#ccflags-y += -DDEBUG_TEST_MIPI_RESET=1
#

amlogic-yuvcam-droid-y := $(DROID_DRV_SRC_OBJ)
#amlogic-yuvcam-rtos-y := $(RTOS_DRV_SRC_OBJ)
