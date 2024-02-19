M_PATH := $(shell dirname $(lastword $(MAKEFILE_LIST)))

MODULE_NAME = amlcam
obj-m += $(MODULE_NAME).o

$(MODULE_NAME)-y += aml_cam.o
$(MODULE_NAME)-y += cam_isp/aml_isp.o
$(MODULE_NAME)-y += cam_isp/aml_isp_global.o
$(MODULE_NAME)-y += cam_isp/aml_isp_stream.o
$(MODULE_NAME)-y += cam_isp/aml_isp_ctrls.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_post_pg2_ctrst.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_cmpr_raw.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_post_pg1_peak_cnr.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_dms.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_tnr.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_post_cm2.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_disp.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_rdmif1.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_apb_dma.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_top.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_dpc.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_fed.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_post_pg0_top.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_3a_flkr.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_intf_top.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_snr.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_patgen.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_ofe.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_rdmif0.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_nr_cac.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_ptnr_mif.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_lens.o
$(MODULE_NAME)-y += cam_isp/hw/aml_isp_hw.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_ltm.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_wrmifx3.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_gtm.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_mcnr_mif.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_post_tnr.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_cnr.o
$(MODULE_NAME)-y += cam_isp/hw/t7_isp_ofe_wdr.o
$(MODULE_NAME)-y += cam_pattern/aml_pattern.o
$(MODULE_NAME)-y += cam_common/aml_video.o
$(MODULE_NAME)-y += cam_common/aml_subdev.o
$(MODULE_NAME)-y += cam_adapter/aml_adapter.o
$(MODULE_NAME)-y += cam_adapter/aml_adapt_global.o
$(MODULE_NAME)-y += cam_adapter/aml_adap_stream.o
$(MODULE_NAME)-y += cam_adapter/hw/aml_adapter_hw.o
$(MODULE_NAME)-y += cam_csiphy/aml_csiphy.o
$(MODULE_NAME)-y += cam_csiphy/hw/aml_csiphy_hw.o

PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"[amlcam]: \" fmt"
ccflags-y += $(PR_FMT_DEFINE)

ccflags-y += -DT7C_CHIP -DANDROID_OS
ccflags-y += -Wno-unused-function
ccflags-y += -Wno-unused-variable

#
#ccflags-y += -DDEBUG_TEST_MIPI_RESET=1
#

LOCAL_INCLUDES := -I$(M_PATH)/ -I$(M_PATH)/cam_common \
                -I$(M_PATH)/cam_isp -I$(M_PATH)/cam_isp/hw/ \
                -I$(M_PATH)/cam_adapter -I$(M_PATH)/cam_adapter/hw/ \
                -I$(M_PATH)/cam_csiphy -I$(M_PATH)/cam_csiphy/hw/ \
                -I$(M_PATH)/cam_pattern


ccflags-y += $(LOCAL_INCLUDES)
ccflags-y += -Wno-declaration-after-statement 
ccflags-y += -Idrivers/staging/android/
ccflags-y += -Wno-error

EXTRA_CFLAGS += $(LOCAL_INCLUDES)
