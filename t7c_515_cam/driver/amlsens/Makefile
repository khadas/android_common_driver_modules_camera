M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

MODULE_NAME = amlsens
obj-m += $(MODULE_NAME).o

$(MODULE_NAME)-y += aml_sens.o
$(MODULE_NAME)-y += sensor_drv.o
$(MODULE_NAME)-y += imx290/aml_imx290.o
$(MODULE_NAME)-y += imx335/aml_imx335.o
$(MODULE_NAME)-y += imx378/aml_imx378.o
$(MODULE_NAME)-y += imx415/aml_imx415.o
#$(MODULE_NAME)-y += imx577/aml_imx577.o
$(MODULE_NAME)-y += lt6911c/aml_lt6911c.o
$(MODULE_NAME)-y += ov08a10/aml_ov08a10.o
$(MODULE_NAME)-y += ov13b10/aml_ov13b10.o
$(MODULE_NAME)-y += ov16a1q/aml_ov16a1q.o
$(MODULE_NAME)-y += ov5640/aml_ov5640.o
#$(MODULE_NAME)-y += ov9285/aml_ov9285.o
$(MODULE_NAME)-y += ov13855/aml_ov13855.o

PR_FMT = $(subst amlogic-,,$(MODULE_NAME))
PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"[$(PR_FMT)]: \" fmt"
ccflags-y += $(PR_FMT_DEFINE)

ccflags-y += -DT7C_CHIP -DANDROID_OS
ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable

LOCAL_INCLUDES := -I$(M_PATH)/ -I$(M_PATH)/include

ccflags-y += $(LOCAL_INCLUDES)
ccflags-y += -Wno-declaration-after-statement
ccflags-y += -Idrivers/staging/android/
ccflags-y += -Wno-error

EXTRA_CFLAGS += $(LOCAL_INCLUDES)
