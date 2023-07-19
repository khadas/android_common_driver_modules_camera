TUNING_SUBDIR := $(shell dirname $(lastword $(MAKEFILE_LIST)))
TUNING_SUBDIR_DTS := $(TUNING_SUBDIR)/imx290
TUNING_SUBDIR_DTS_1 := $(TUNING_SUBDIR)/ov08a10
TUNING_SRC = $(shell find $(TUNING_SUBDIR) -name '*.cpp')
LIBTUNING_OBJ = $(patsubst %.cpp,%.o,$(TUNING_SRC))

#LOCAL_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

ISP_ROOT_DIR := $(TUNING_SUBDIR)/..
LOCAL_INCLUDES := -I$(ISP_ROOT_DIR)/include/
LOCAL_INCLUDES += -I$(TUNING_SUBDIR_DTS)
LOCAL_INCLUDES += -I$(TUNING_SUBDIR_DTS_1)

CFLAGS += -I ./include/
CFLAGS += -I ./kernel-headers/
CFLAGS += $(LOCAL_INCLUDES)

LIBTUNING = libtuning.so


.PHONY: clean

# rules
all: $(LIBTUNING)

%.o: %.cpp
#%.o: $(TUNING_SUBDIR) %.cpp
	$(CC) -c -fPIC  $(CFLAGS) $^ -o $@

$(LIBTUNING): $(LIBTUNING_OBJ)
	$(CXX) -shared  -Wl,-soname,$(LIBTUNING) -fPIC $(CFLAGS) $^ -o $(LIBTUNING)

clean:
	rm -f $(LIBTUNING_OBJ) $(LIBTUNING)

