# Makefile for OMAP3 ISP driver

isp-mod-objs += \
	isp.o ispccdc.o \
	isppreview.o ispresizer.o isph3a.o isphist.o isp_af.o ispcsi2.o \
	ispstat.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_PREVIEWER) += \
	omap_previewer.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_RESIZER) += \
	omap_resizer.o

obj-$(CONFIG_VIDEO_OMAP34XX_OV7675_ISP_GLUE) += \
	omap_ov7675_isp.o

obj-$(CONFIG_VIDEO_OMAP34XX_MT9D113_ISP_GLUE) += \
	omap_mt9d113_isp.o

obj-$(CONFIG_VIDEO_OMAP3) += isp-mod.o

