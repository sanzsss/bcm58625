PORTMACRO_ENABLE := y
INC_DIRS = $(srctree)/drivers/net/ethernet/broadcom/brm
AMAC_DIR = $(srctree)/drivers/net/ethernet/broadcom/amac

ifeq ($(PORTMACRO_ENABLE),y)
INC_DIRS += $(srctree)/drivers/net/ethernet/broadcom/amac/include
INC_DIRS += $(srctree)/drivers/net/ethernet/broadcom/amac/phymod/include
#INC_DIRS += $(srctree)/drivers/net/ethernet/broadcom/amac/phymod/pmd-orig/include/
INC_DIRS += $(AMAC_DIR)
ccflags-y += -DPHYMOD_INCLUDE_CUSTOM_CONFIG
endif

ccflags-y += $(INC_DIRS:%=-I%) #-DAMAC_DEBUG=0 -DDEBUG
ccflags-y += -DLINUX -D__LINUX

obj-$(CONFIG_NET_BCM_AMAC) += bcmamac.o
bcmamac-y :=  bcm-amac-enet.o bcm-amac-gphy.o bcm-amac-core.o bcm-amac-ethtool.o

ifeq ($(PORTMACRO_ENABLE),y)
PM_SOURCES += lm_hw_pm.c
PM_SOURCES += pm_intf.c
PM_SOURCES += phymod/core/phymod.c
PM_SOURCES += phymod/core/phymod_acc.c
PM_SOURCES += phymod/core/phymod_debug.c
PM_SOURCES += phymod/core/phymod_dispatch.c
PM_SOURCES += phymod/acc/phymod_tsc_iblk.c
PM_SOURCES += phymod/util/phymod_util.c
PM_SOURCES += phymod/chip/tsce/tier2/tsce.c
PM_SOURCES += phymod/chip/tsce/tier2/tsce_ucode.c
PM_SOURCES += phymod/chip/tsce/tsce_dispatch.c
PM_SOURCES += phymod/chip/tsce/tier1/temod_cfg_seq.c
PM_SOURCES += phymod/chip/tsce/tier1/temod_sc_lkup_table.c
#PM_SOURCES += phymod/chip/tscf/tier2/tscf.c
#PM_SOURCES += phymod/chip/falcon/tier2/tscf_ucode.c
#PM_SOURCES += phymod/chip/tscf/tscf_dispatch.c
#PM_SOURCES += phymod/chip/tscf/tier1/tefmod_cfg_seq.c
#PM_SOURCES += phymod/chip/tscf/tier1/tefmod_enum_defines.c
#PM_SOURCES += phymod/chip/tscf/tier1/tefmod_sc_lkup_table.c
PM_SOURCES += phymod/chip/eagle/eagle_dispatch.c
PM_SOURCES += phymod/chip/eagle/tier1/eagle_cfg_seq.c
PM_SOURCES += phymod/chip/eagle/tier1/eagle_tsc_functions.c
PM_SOURCES += phymod/chip/eagle/tier1/eagle_tsc_dependencies.c
PM_SOURCES += phymod/chip/eagle/tier2/eagle.c
PM_SOURCES += phymod/chip/eagle/tier2/eagle_common.c
#PM_SOURCES += phymod/chip/falcon/tier2/falcon.c
#PM_SOURCES += phymod/chip/falcon/falcon_dispatch.c
#PM_SOURCES += phymod/chip/falcon/tier1/falcon_cfg_seq.c
#PM_SOURCES += phymod/chip/falcon/tier1/falcon_tsc_functions.c
#PM_SOURCES += phymod/chip/falcon/tier1/falcon_tsc_dependencies.c

PM_OBJS = $(PM_SOURCES:%.c=%.o)

bcmamac-y +=  $(PM_OBJS)
endif
