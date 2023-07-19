MEDIA_API_LOCAL_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))
MEDIA_API_SRC = $(shell find $(MEDIA_API_LOCAL_PATH) -name '*.cpp')
MEDIA_API_OBJ = $(patsubst %.cpp,%.o,$(MEDIA_API_SRC))


ISP_ROOT_DIR := $(MEDIA_API_LOCAL_PATH)/..
LOCAL_INCLUDES := -I$(ISP_ROOT_DIR)/include/
LOCAL_INCLUDES+= -I$(ISP_ROOT_DIR)/sensor_calibration/imx290/
LOCAL_INCLUDES+= -I$(ISP_ROOT_DIR)/sensor_calibration/ov08a10/
LOCAL_INCLUDES+= -I$(ISP_ROOT_DIR)/sensor_calibration/imx415/
LOCAL_INCLUDES+= -I$(ISP_ROOT_DIR)/lens/dw9714/

CFLAGS += -I ./include/
CFLAGS += -I ./kernel-headers/
CFLAGS += $(LOCAL_INCLUDES)

LIBMEDIA_API = libmediaAPI.so


.PHONY: clean

# rules
all: $(LIBMEDIA_API)

%.o: %.cpp
	$(CC) -c -fPIC  $(CFLAGS) $^ -o $@

$(LIBMEDIA_API): $(MEDIA_API_OBJ)
	$(CXX) -shared  -Wl,-soname,$(LIBMEDIA_API) -fPIC $(CFLAGS) $^ -o $(LIBMEDIA_API)

clean:
	rm -f $(MEDIA_API_OBJ) $(LIBMEDIA_API)

