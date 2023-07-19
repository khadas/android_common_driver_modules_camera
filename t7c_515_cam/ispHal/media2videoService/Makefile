LOCAL_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

ISP_ROOT_DIR := $(LOCAL_PATH)/..
LOCAL_INCLUDES := -I$(ISP_ROOT_DIR)/include/
LOCAL_INCLUDES += -I$(ISP_ROOT_DIR)/sensor_calibration/imx290/
LOCAL_INCLUDES += -I$(ISP_ROOT_DIR)/sensor_calibration/ov08a10/


TARGET := media2videoService

SRC_OBJ := media2videoService.o

#CFLAGS := -Wall -Werror -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast
CFLAGS := -Wall -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast
CFLAGS += $(LOCAL_INCLUDES)
CFLAGS += -I./external -pthread -fPIE
CFLAGS += -L$(ISP_ROOT_DIR)/firmware -lispaml -ldl -L$(ISP_ROOT_DIR)/media-v4l2 -lmediaAPI -L$(ISP_ROOT_DIR)/sensor_calibration -ltuning -lm -L$(ISP_ROOT_DIR)/lens -llens
CFLAGS += -fPIC -D_FORTIFY_SOURCE=2 -O3

#CFLAGS += -L./external -lH264VideoStreamer -lx264 -lcrypto -lssl

.PHONY:clean

all: $(TARGET)

%.o: %.cpp
	$(CC) -c $(CFLAGS) $< -o $@

$(TARGET): $(SRC_OBJ)
	$(CXX) $(CFLAGS) $^ -o $@

clean:
	rm -rf $(TARGET)
