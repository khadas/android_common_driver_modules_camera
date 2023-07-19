LENS_SUBDIR := $(shell dirname $(lastword $(MAKEFILE_LIST)))
LENS_SUBDIR_DTS := $(LENS_SUBDIR)/dw9714
LENS_SRC = $(shell find $(LENS_SUBDIR) -name '*.cpp')
LIBLENS_OBJ = $(patsubst %.cpp,%.o,$(LENS_SRC))

#LOCAL_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

ISP_ROOT_DIR := $(LENS_SUBDIR)/..
LOCAL_INCLUDES := -I$(ISP_ROOT_DIR)/include/
LOCAL_INCLUDES += -I$(LENS_SUBDIR_DTS)

CFLAGS += -I ./include/
CFLAGS += -I ./kernel-headers/
CFLAGS += $(LOCAL_INCLUDES)

LIBLENS = liblens.so


.PHONY: clean

# rules
all: $(LIBLENS)

%.o: %.cpp
#%.o: $(TUNING_SUBDIR) %.cpp
	$(CC) -c -fPIC  $(CFLAGS) $^ -o $@

$(LIBLENS): $(LIBLENS_OBJ)
	$(CXX) -shared  -Wl,-soname,$(LIBLENS) -fPIC $(CFLAGS) $^ -o $(LIBLENS)

clean:
	rm -f $(LIBLENS_OBJ) $(LIBLENS)

