# SPDX-License-Identifier: (GPL-2.0+ OR MIT)

M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

MODULE_NAME = ov9285

PR_FMT = $(subst amlogic-,,$(MODULE_NAME))
PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"[$(PR_FMT)]: \" fmt"
ccflags-y += $(PR_FMT_DEFINE)

LOCAL_INCLUDES := -I$(M_PATH)/../../amlcam/cam_common
ccflags-y += $(LOCAL_INCLUDES)

obj-m += $(MODULE_NAME).o

$(MODULE_NAME)-y += aml_ov9285.o 
