M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

MODULE_NAME = iv009_isp_lens
obj-m += $(MODULE_NAME).o

$(MODULE_NAME)-y += app/soc_lens.o
$(MODULE_NAME)-y += src/platform/system_log.o
$(MODULE_NAME)-y += src/platform/system_i2c.o
$(MODULE_NAME)-y += src/platform/system_spi.o
$(MODULE_NAME)-y += src/platform/system_sensor.o
$(MODULE_NAME)-y += src/platform/system_timer.o
$(MODULE_NAME)-y += src/platform/system_hw_io.o
$(MODULE_NAME)-y += src/platform/system_stdlib.o
$(MODULE_NAME)-y += src/platform/system_sw_io.o
$(MODULE_NAME)-y += src/platform/system_spinlock.o
$(MODULE_NAME)-y += src/fw_lib/acamera_logger.o
$(MODULE_NAME)-y += src/fw_lib/acamera_sbus.o
$(MODULE_NAME)-y += src/fw_lib/acamera_sbus_isp.o
$(MODULE_NAME)-y += src/fw_lib/acamera_sbus_spi.o
$(MODULE_NAME)-y += src/fw_lib/acamera_sbus_isp_sw.o
$(MODULE_NAME)-y += src/fw_lib/acamera_math.o
$(MODULE_NAME)-y += src/fw_lib/acamera_sbus_i2c.o
$(MODULE_NAME)-y += src/fw_lib/sensor_init.o
$(MODULE_NAME)-y += src/driver/lens/lens_init.o
$(MODULE_NAME)-y += src/driver/lens/null_vcm.o
$(MODULE_NAME)-y += src/driver/sensor/sensor_bus_config.o
$(MODULE_NAME)-y += src/driver/sensor/ACameraDefault.o
$(MODULE_NAME)-y += src/driver/sensor/Dummy_drv.o

LOCAL_INCLUDES := -I$(M_PATH)/app -I$(M_PATH)/inc -I$(M_PATH)/app/control \
                  -I$(M_PATH)/inc/api -I$(M_PATH)/inc/isp -I$(M_PATH)/inc/sys \
                  -I$(M_PATH)/src/platform -I$(M_PATH)/src/fw \
                  -I$(M_PATH)/src/fw_lib -I$(M_PATH)/src/calibration \
                  -I$(M_PATH)/src/driver/sensor -I$(M_PATH)/src/driver/lens

ccflags-y += -Wno-declaration-after-statement
ccflags-y += -Idrivers/staging/android/
ccflags-y += -Wno-error
ccflags-y += $(LOCAL_INCLUDES)

EXTRA_CFLAGS += $(LOCAL_INCLUDES)
