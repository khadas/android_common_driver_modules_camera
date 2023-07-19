# SPDX-License-Identifier: (GPL-2.0+ OR MIT)

MODULE_NAME = ov13b10

PR_FMT = $(subst amlogic-,,$(MODULE_NAME))
PR_FMT_DEFINE="-Dpr_fmt(fmt)= \"[$(PR_FMT)]: \" fmt"
ccflags-y += $(PR_FMT_DEFINE)

obj-m += $(MODULE_NAME).o

$(MODULE_NAME)-y += aml_ov13b10.o 
