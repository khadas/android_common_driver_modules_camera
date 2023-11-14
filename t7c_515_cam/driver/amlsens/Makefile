M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

obj-m += amlsens.o
obj-m += amlsens-csi-a.o
obj-m += amlsens-csi-b.o

COMMON_DRV_SRC :=  $(wildcard $(M_PATH)/sensor_drv.c) \
			$(wildcard $(M_PATH)/imx290/*.c) \
			$(wildcard $(M_PATH)/imx335/*.c) \
			$(wildcard $(M_PATH)/imx378/*.c) \
			$(wildcard $(M_PATH)/imx415/*.c) \
			$(wildcard $(M_PATH)/imx577/*.c) \
			$(wildcard $(M_PATH)/lt6911c/*.c) \
			$(wildcard $(M_PATH)/ov08a10/*.c) \
			$(wildcard $(M_PATH)/ov13b10/*.c) \
			$(wildcard $(M_PATH)/ov16a1q/*.c) \
			$(wildcard $(M_PATH)/ov5640/*.c) \
			$(wildcard $(M_PATH)/ov13855/*.c) \
			$(wildcard $(M_PATH)/max96712/*.c)


COMMON_DRV_SRC_NAME := $(COMMON_DRV_SRC:$(M_PATH)/%=%)

COMMON_DRV_SRC_OBJ := $(COMMON_DRV_SRC_NAME:.c=.o)

PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"[amlsens]: \" fmt"
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

amlsens-y := $(COMMON_DRV_SRC_OBJ) aml_sens.o
amlsens-csi-a-y := $(COMMON_DRV_SRC_OBJ) aml_sens_csi_a.o
amlsens-csi-b-y := $(COMMON_DRV_SRC_OBJ) aml_sens_csi_b.o
