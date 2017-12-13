
ccflags-y += -I$(src)/ -DWILC_ASIC_A0 -DWILC_DEBUGFS
ccflags-y += -DDISABLE_PWRSAVE_AND_SCAN_DURING_IP

wilc-objs := wilc_wfi_cfgoperations.o linux_wlan.o linux_mon.o \
			coreconfigurator.o host_interface.o \
			wilc_wlan_cfg.o wilc_debugfs.o \
			wilc_wlan.o sysfs.o wilc_bt.o

obj-$(CONFIG_WILC_SDIO) += wilc-sdio.o
wilc-sdio-objs += $(wilc-objs)
wilc-sdio-objs += wilc_sdio.o

obj-$(CONFIG_WILC_SPI) += wilc-spi.o
wilc-spi-objs += $(wilc-objs)
wilc-spi-objs += wilc_spi.o 
