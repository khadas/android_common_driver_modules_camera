KBUILD_CFLAGS_MODULE += $(GKI_EXT_MODULE_PREDEFINE)

ifeq ($(O),)
out_dir := .
else
out_dir := $(O)
endif
include $(out_dir)/include/config/auto.conf

obj-y += v4l2_dev/
obj-y += subdev/

modules:
	@$(MAKE) -C $(KERNEL_SRC) M=$(M)  modules

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	$(Q)mkdir -p ${out_dir}/../vendor_lib/modules
	$(Q)if [ -z "$(CONFIG_AMLOGIC_KERNEL_VERSION)" ]; then \
                cd ${out_dir}/$(M)/; find -name "*.ko" -exec cp {} ${out_dir}/../vendor_lib/modules/mali.ko \; ; \
        else \
                find $(INSTALL_MOD_PATH)/lib/modules/*/$(INSTALL_MOD_DIR) -name "*.ko" -exec cp {} ${out_dir}/../vendor_lib/modules/mali.ko \; ; \
        fi

clean:
	@$(MAKE) -C $(KERNEL_SRC) M=$(M) clean

