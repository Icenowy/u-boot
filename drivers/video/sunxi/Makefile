# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2000-2007
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.

obj-$(CONFIG_VIDEO_SUNXI) += sunxi_display.o simplefb_common.o lcdc.o tve_common.o ../videomodes.o
ifeq ($(CONFIG_SUNXI_DE3),y)
obj-$(CONFIG_VIDEO_DE2) += sunxi_de3.o sun50i_dw_hdmi.o simplefb_common.o lcdc.o ../dw_hdmi.o
else
obj-$(CONFIG_VIDEO_DE2) += sunxi_de2.o sunxi_dw_hdmi.o simplefb_common.o lcdc.o ../dw_hdmi.o sunxi_lcd.o
endif
