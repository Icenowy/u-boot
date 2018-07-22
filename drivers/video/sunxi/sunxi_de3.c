#define DEBUG

/*
 * Allwinner DE2 display driver
 *
 * (C) Copyright 2017 Jernej Skrabec <jernej.skrabec@siol.net>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <display.h>
#include <dm.h>
#include <edid.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <video.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/display2.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#include "simplefb_common.h"

DECLARE_GLOBAL_DATA_PTR;

enum {
	/* Maximum LCD size we support */
	LCD_MAX_WIDTH		= 3840,
	LCD_MAX_HEIGHT		= 2160,
	LCD_MAX_LOG2_BPP	= VIDEO_BPP32,
};

static void sunxi_de3_composer_init(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	u32 reg_value;

	/* set SRAM for video use (H6 only) */
	reg_value = readl(SUNXI_SRAMC_BASE + 0x04);
	reg_value &= ~(0x01 << 24);
	writel(reg_value, SUNXI_SRAMC_BASE + 0x04);

	writel(CCM_PLL10_DEFAULT, &ccm->pll10_cfg);
	while (!(readl(&ccm->pll10_cfg) & CCM_PLL10_LOCK))
		;

	/* Set DE parent to pll10 */
	writel(CCM_DE_CTRL_PLL10 | CCM_DE_CTRL_M(1), &ccm->de_clk_cfg);

	/* Set ahb gating to pass */
	writel(BIT(0) | BIT(RESET_SHIFT), &ccm->de_gate_reset);

	/* Clock on */
	setbits_le32(&ccm->de_clk_cfg, CCM_DE_CTRL_ENABLE);

	mdelay(100);
}

static void sunxi_de3_mode_set(int mux, const struct display_timing *mode,
			       int bpp, ulong address, bool is_composite)
{
	ulong de_mux_base = (mux == 0) ?
			    SUNXI_DE2_MUX0_BASE : SUNXI_DE2_MUX1_BASE;
	struct de_clk * const de_clk_regs =
		(struct de_clk *)(SUNXI_DE3_BASE);
	struct de_glb * const de_glb_regs =
		(struct de_glb *)(de_mux_base +
				  SUNXI_DE2_MUX_GLB_REGS);
	struct de_bld * const de_bld_regs =
		(struct de_bld *)(de_mux_base +
				  SUNXI_DE2_MUX_BLD_REGS);
	struct de_ui * const de_ui_regs =
		(struct de_ui *)(de_mux_base +
				 SUNXI_DE2_MUX_CHAN_REGS +
				 SUNXI_DE2_MUX_CHAN_SZ * 1);
	u32 size = SUNXI_DE2_WH(mode->hactive.typ, mode->vactive.typ);
	int channel;
	u32 format;

	/* enable clock */
	setbits_le32(&de_clk_regs->rst_cfg, BIT(mux));
	setbits_le32(&de_clk_regs->gate_cfg, BIT(mux));
	setbits_le32(&de_clk_regs->bus_cfg, BIT(mux));

	clrbits_le32(&de_clk_regs->sel_cfg, 1);

	/* wait for clocks*/
	mdelay(1);

	writel(SUNXI_DE2_MUX_GLB_CTL_EN, &de_glb_regs->ctl);
	writel(0, &de_glb_regs->status);
	writel(1, &de_glb_regs->dbuff);
	writel(size, &de_glb_regs->size);

	for (channel = 0; channel < 4; channel++) {
		void *ch = (void *)(de_mux_base + SUNXI_DE2_MUX_CHAN_REGS +
				    SUNXI_DE2_MUX_CHAN_SZ * channel);
		memset(ch, 0, SUNXI_DE2_MUX_CHAN_SZ);
	}
	memset(de_bld_regs, 0, sizeof(struct de_bld));

	writel(0x00000101, &de_bld_regs->fcolor_ctl);

	writel(1, &de_bld_regs->route);

	writel(0, &de_bld_regs->premultiply);
	writel(0xff000000, &de_bld_regs->bkcolor);

	writel(0x03010301, &de_bld_regs->bld_mode[0]);

	writel(size, &de_bld_regs->output_size);
	writel(mode->flags & DISPLAY_FLAGS_INTERLACED ? 2 : 0,
	       &de_bld_regs->out_ctl);
	writel(0, &de_bld_regs->ck_ctl);

	writel(0xff000000, &de_bld_regs->attr[0].fcolor);
	writel(size, &de_bld_regs->attr[0].insize);

	/* Disable all scaler units */
	writel(0, de_mux_base + SUNXI_DE2_MUX_VSU_REGS);
	writel(0, de_mux_base + SUNXI_DE2_MUX_GSU1_REGS);
	writel(0, de_mux_base + SUNXI_DE2_MUX_GSU2_REGS);
	writel(0, de_mux_base + SUNXI_DE2_MUX_GSU3_REGS);

	switch (bpp) {
	case 16:
		format = SUNXI_DE2_UI_CFG_ATTR_FMT(SUNXI_DE2_FORMAT_RGB_565);
		break;
	case 32:
	default:
		format = SUNXI_DE2_UI_CFG_ATTR_FMT(SUNXI_DE2_FORMAT_XRGB_8888);
		break;
	}

	writel(SUNXI_DE2_UI_CFG_ATTR_EN | format, &de_ui_regs->cfg[0].attr);
	writel(size, &de_ui_regs->cfg[0].size);
	writel(0, &de_ui_regs->cfg[0].coord);
	writel((bpp / 8) * mode->hactive.typ, &de_ui_regs->cfg[0].pitch);
	writel(address, &de_ui_regs->cfg[0].top_laddr);
	writel(size, &de_ui_regs->ovl_size);

	/* apply settings */
	writel(1, &de_glb_regs->dbuff);
}

static int sunxi_de3_init(struct udevice *dev, ulong fbbase,
			  enum video_log2_bpp l2bpp,
			  struct udevice *disp, int mux, bool is_composite)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct display_timing timing;
	struct display_plat *disp_uc_plat;
	int ret;

	disp_uc_plat = dev_get_uclass_platdata(disp);
	debug("Using device '%s', disp_uc_priv=%p\n", disp->name, disp_uc_plat);
	if (display_in_use(disp)) {
		debug("   - device in use\n");
		return -EBUSY;
	}

	disp_uc_plat->source_id = mux;

	ret = device_probe(disp);
	if (ret) {
		debug("%s: device '%s' display won't probe (ret=%d)\n",
		      __func__, dev->name, ret);
		return ret;
	}

	ret = display_read_timing(disp, &timing);
	if (ret) {
		debug("%s: Failed to read timings\n", __func__);
		return ret;
	}

	sunxi_de3_composer_init();
	sunxi_de3_mode_set(mux, &timing, 1 << l2bpp, fbbase, is_composite);

	ret = display_enable(disp, 1 << l2bpp, &timing);
	if (ret) {
		debug("%s: Failed to enable display\n", __func__);
		return ret;
	}

	uc_priv->xsize = timing.hactive.typ;
	uc_priv->ysize = timing.vactive.typ;
	uc_priv->bpix = l2bpp;
	debug("fb=%lx, size=%d %d\n", fbbase, uc_priv->xsize, uc_priv->ysize);

	return 0;
}

static int sunxi_de3_probe(struct udevice *dev)
{
	struct video_uc_platdata *plat = dev_get_uclass_platdata(dev);
	struct udevice *disp;
	int ret;

	/* Before relocation we don't need to do anything */
	if (!(gd->flags & GD_FLG_RELOC))
		return 0;

	ret = uclass_find_device_by_name(UCLASS_DISPLAY,
					 "sun50i_dw_hdmi", &disp);
	if (ret) {
		debug("%s: hdmi display not found (ret=%d)\n", __func__, ret);
		return ret;
	}

	ret = sunxi_de3_init(dev, plat->base, VIDEO_BPP32, disp, 0,
			     false);
	if (ret)
		return ret;

	video_set_flush_dcache(dev, 1);
	return 0;
}

static int sunxi_de3_bind(struct udevice *dev)
{
	struct video_uc_platdata *plat = dev_get_uclass_platdata(dev);

	plat->size = LCD_MAX_WIDTH * LCD_MAX_HEIGHT *
		(1 << LCD_MAX_LOG2_BPP) / 8;

	return 0;
}

static const struct video_ops sunxi_de3_ops = {
};

U_BOOT_DRIVER(sunxi_de3) = {
	.name	= "sunxi_de3",
	.id	= UCLASS_VIDEO,
	.ops	= &sunxi_de3_ops,
	.bind	= sunxi_de3_bind,
	.probe	= sunxi_de3_probe,
	.flags	= DM_FLAG_PRE_RELOC,
};

U_BOOT_DEVICE(sunxi_de3) = {
	.name = "sunxi_de3"
};

/*
 * Simplefb support.
 */
#if defined(CONFIG_OF_BOARD_SETUP) && defined(CONFIG_VIDEO_DT_SIMPLEFB)
int sunxi_simplefb_setup(void *blob)
{
	struct udevice *de2, *hdmi;
	struct video_priv *de2_priv;
	struct video_uc_platdata *de2_plat;
	int offset, ret;
	u64 start, size;
	const char *pipeline = NULL;

	debug("Setting up simplefb\n");

	/* Skip simplefb setting if DE2 / HDMI is not present */
	ret = uclass_find_device_by_name(UCLASS_VIDEO,
					 "sunxi_de3", &de2);
	if (ret) {
		debug("DE3 not present\n");
		return 0;
	} else if (!device_active(de2)) {
		debug("DE3 present but not probed\n");
		return 0;
	}

	ret = uclass_find_device_by_name(UCLASS_DISPLAY,
					 "sun50i_dw_hdmi", &hdmi);
	if (ret) {
		debug("HDMI not present\n");
	} else if (device_active(hdmi)) {
		pipeline = "mixer0-tcon_tv0-hdmi";
	} else {
		debug("HDMI present but not probed\n");
	}

	if (!pipeline) {
		debug("No active display present\n");
		return 0;
	}

	de2_priv = dev_get_uclass_priv(de2);
	de2_plat = dev_get_uclass_platdata(de2);

	offset = sunxi_simplefb_fdt_match(blob, pipeline);
	if (offset < 0) {
		eprintf("Cannot setup simplefb: node not found\n");
		return 0; /* Keep older kernels working */
	}

	start = gd->bd->bi_dram[0].start;
	size = de2_plat->base - start;
	ret = fdt_fixup_memory_banks(blob, &start, &size, 1);
	if (ret) {
		eprintf("Cannot setup simplefb: Error reserving memory\n");
		return ret;
	}

	ret = fdt_setup_simplefb_node(blob, offset, de2_plat->base,
			de2_priv->xsize, de2_priv->ysize,
			VNBYTES(de2_priv->bpix) * de2_priv->xsize,
			"x8r8g8b8");
	if (ret)
		eprintf("Cannot setup simplefb: Error setting properties\n");

	return ret;
}
#endif /* CONFIG_OF_BOARD_SETUP && CONFIG_VIDEO_DT_SIMPLEFB */
