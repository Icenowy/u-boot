/*
 * Allwinner DW HDMI bridge
 *
 * (C) Copyright 2017 Jernej Skrabec <jernej.skrabec@siol.net>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <display.h>
#include <dm.h>
#include <dw_hdmi.h>
#include <edid.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/lcdc.h>
#include <i2c.h>
#include <linux/delay.h>

struct sunxi_dw_hdmi_priv {
	struct dw_hdmi hdmi;
};

static const struct hdmi_phy_config sunxi_phy_config[] = {
	{
		.mpixelclock = 25175000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0232,
	}, {
		.mpixelclock = 27000000,
		.sym_ctr = 0x8009, .term = 0x0007, .vlev_ctr = 0x02b0,
	}, {
		.mpixelclock = 44900000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0232,
	}, {
		.mpixelclock = 73250000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0232,
	}, {
		.mpixelclock = 74250000,
		.sym_ctr = 0x8009, .term = 0x0006, .vlev_ctr = 0x022d,
	}, {
		.mpixelclock = 90000000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0232,
	}, {
		.mpixelclock = 148250000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0232,
	}, {
		.mpixelclock = 148500000,
		.sym_ctr = 0x8029, .term = 0x0006, .vlev_ctr = 0x0270,
	}, {
		.mpixelclock = 182750000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0230,
	}, {
		.mpixelclock = 218250000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0230,
	}, {
		.mpixelclock = 288000000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0273,
	}, {
		.mpixelclock = 297000000,
		.sym_ctr = 0x8009, .term = 0x0005, .vlev_ctr = 0x01ab,
	}, {
		.mpixelclock = 340000000,
		.sym_ctr = 0x8029, .term = 0x0004, .vlev_ctr = 0x0273,
	}, {
		.mpixelclock = 552750000,
		.sym_ctr = 0x8039, .term = 0x0004, .vlev_ctr = 0x014a,
	}, {
		.mpixelclock = 594000000,
		.sym_ctr = 0x8029, .term = 0x0000, .vlev_ctr = 0x008a,
	}, {
		.mpixelclock = ~0ul,
		.sym_ctr = 0x0000, .term = 0x0000, .vlev_ctr = 0x0000,
	}
};

static const struct hdmi_mpll_config sunxi_mpll_cfg[] = {
	{
		.mpixelclock = 25175000,
		.cpce = 0x00b3, .gmp = 0x0000, .curr = 0x0000,
	}, {
		.mpixelclock = 27000000,
		.cpce = 0x00b3, .gmp = 0x0000, .curr = 0x0012,
	}, {
		.mpixelclock = 44900000,
		.cpce = 0x00b3, .gmp = 0x0000, .curr = 0x0000,
	}, {
		.mpixelclock = 73250000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0008,
	}, {
		.mpixelclock = 74250000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0013,
	}, {
		.mpixelclock = 90000000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0008,
	}, {
		.mpixelclock = 148250000,
		.cpce = 0x0051, .gmp = 0x0002, .curr = 0x001b,
	}, {
		.mpixelclock = 148500000,
		.cpce = 0x0051, .gmp = 0x0002, .curr = 0x0019,
	}, {
		.mpixelclock = 182750000,
		.cpce = 0x0051, .gmp = 0x0002, .curr = 0x001b,
	}, {
		.mpixelclock = 218250000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0036,
	}, {
		.mpixelclock = 288000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0036,
	}, {
		.mpixelclock = 297000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0019,
	}, {
		.mpixelclock = 340000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0036,
	}, {
		.mpixelclock = 552750000,
		.cpce = 0x1A40, .gmp = 0x0003, .curr = 0x003f,
	}, {
		.mpixelclock = ~0ul,
		.cpce = 0x1A7c, .gmp = 0x0003, .curr = 0x0010,
	}
};

static void sunxi_dw_hdmi_pll_set(uint clk_khz)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int best_n = 0, best_m = 0, best_div = 0, best_diff = 0x0FFFFFFF;
	int value, n, m, div, diff;

	/*
	 * Find the lowest divider resulting in a matching clock. If there
	 * is no match, pick the closest lower clock, as monitors tend to
	 * not sync to higher frequencies.
	 */
	for (div = 1; div <= 16; div++)
		for (m = 1; m <= 2; m++) {
			n = (m * div * clk_khz) / 6000;

			if ((n >= 1) && (n <= 254)) {
				value = (6000 * n) / m / div;
				diff = clk_khz - value;
				if (diff < best_diff) {
					best_diff = diff;
					best_m = m;
					best_n = n;
					best_div = div;
				}
			}
		}

	clock_set_pll3_factors(best_m, best_n);
	setbits_le32(&ccm->hdmi_clk_cfg, CCM_HDMI_CTRL_M(best_div));
	setbits_le32(&ccm->tcon_tv0_clk_cfg, CCM_LCD0_CTRL_M(best_div));
	debug("dotclock: %dkHz = %dkHz: (6MHz * %d) / %d / %d\n",
	      clk_khz, (clock_get_pll3() / 1000) / best_div,
	      best_n, best_m, best_div);
}

static void sunxi_dw_hdmi_lcdc_init(const struct display_timing *edid, int bpp)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_lcdc_reg * const lcdc =
		(struct sunxi_lcdc_reg *)SUNXI_TCON_TV0_BASE;

	/* Set lcd parent to pll3 */
	clrsetbits_le32(&ccm->tcon_tv0_clk_cfg, CCM_LCD0_CTRL_PLL_MASK,
			CCM_LCD0_CTRL_PLL_VIDEO0);

	/* Reset off & bus on*/
	setbits_le32(&ccm->tcon_top_gate_reset,
		     CCM_LCD_TOP_CTRL_RST | CCM_LCD_TOP_CTRL_GATE);

	/* Reset off & bus on*/
	setbits_le32(&ccm->tcon_tv_gate_reset,
		     CCM_TCON_TV_BGR_CTRL_RST | CCM_TCON_TV_BGR_CTRL_GATE);

	/* Clock on */
	setbits_le32(&ccm->tcon_tv0_clk_cfg, CCM_LCD0_CTRL_GATE);

	/* enable select right tcon for hdmi */
	/* TODO: move to de3 driver */
	writel(2, 0x0651001c);
	writel(BIT(20) | BIT(28), 0x06510020);

	lcdc_init(lcdc);
	lcdc_tcon1_mode_set(lcdc, edid, false, false);
	lcdc_enable(lcdc, bpp);
}

static int sunxi_dw_hdmi_read_edid(struct udevice *dev, u8 *buf, int buf_size)
{
	struct sunxi_dw_hdmi_priv *priv = dev_get_priv(dev);

	return dw_hdmi_read_edid(&priv->hdmi, buf, buf_size);
}

static int sunxi_dw_hdmi_enable(struct udevice *dev, int panel_bpp,
				const struct display_timing *edid)
{
	struct sunxi_dw_hdmi_priv *priv = dev_get_priv(dev);
	int ret;

	sunxi_dw_hdmi_pll_set(edid->pixelclock.typ/1000);

	/* enable external resistor */
	writeb(0xc0, SUNXI_HDMI_BASE + 0x10006);
	writeb(0x80, SUNXI_HDMI_BASE + 0x10007);

	ret = dw_hdmi_enable(&priv->hdmi, edid);
	if (ret)
		return ret;

	sunxi_dw_hdmi_lcdc_init(edid, panel_bpp);

	return 0;
}

static int sunxi_dw_hdmi_probe(struct udevice *dev)
{
	struct sunxi_dw_hdmi_priv *priv = dev_get_priv(dev);
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	int ret;

	/* Toggle HSCL, HSDA pins that are muxed via PIO */
	sunxi_gpio_set_cfgpin(SUNXI_GPH(8), SUN50I_H6_GPH_HDMI);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(9), SUN50I_H6_GPH_HDMI);

	/* Set pll3 to 297 MHz */
	clock_set_pll3(297000000);

	/* Set hdmi parent to pll3 */
	clrsetbits_le32(&ccm->hdmi_clk_cfg, CCM_HDMI_CTRL_PLL_MASK,
			CCM_HDMI_CTRL_PLL3);

	setbits_le32(&ccm->hdmi_slow_clk_cfg, CCM_HDMI_SLOW_CTRL_GATE);

	/* Reset HDMI */
	clrbits_le32(&ccm->hdmi_gate_reset,
		     CCM_HDMI_BUS_CTRL_RST0 |
		     CCM_HDMI_BUS_CTRL_RST1);

	udelay(100);

	/* Set ahb gating to pass */
	setbits_le32(&ccm->hdmi_gate_reset,
		     CCM_HDMI_BUS_CTRL_GATE |
		     CCM_HDMI_BUS_CTRL_RST0 |
		     CCM_HDMI_BUS_CTRL_RST1);

	/* Clock on */
	setbits_le32(&ccm->hdmi_clk_cfg, CCM_HDMI_CTRL_GATE);

	priv->hdmi.ioaddr = SUNXI_HDMI_BASE;
	priv->hdmi.i2c_clk_high = 221;
	priv->hdmi.i2c_clk_low = 245;
	priv->hdmi.reg_io_width = 1;
	priv->hdmi.phy_set = dw_hdmi_phy_cfg;
	priv->hdmi.mpll_cfg = sunxi_mpll_cfg;
	priv->hdmi.phy_cfg = sunxi_phy_config;

	ret = dw_hdmi_phy_wait_for_hpd(&priv->hdmi);
	if (ret < 0) {
		debug("hdmi can not get hpd signal\n");
		return -1;
	}

	dw_hdmi_init(&priv->hdmi);

	return 0;
}

static const struct dm_display_ops sunxi_dw_hdmi_ops = {
	.read_edid = sunxi_dw_hdmi_read_edid,
	.enable = sunxi_dw_hdmi_enable,
};

U_BOOT_DRIVER(sunxi_dw_hdmi) = {
	.name	= "sun50i_dw_hdmi",
	.id	= UCLASS_DISPLAY,
	.ops	= &sunxi_dw_hdmi_ops,
	.probe	= sunxi_dw_hdmi_probe,
	.priv_auto_alloc_size = sizeof(struct sunxi_dw_hdmi_priv),
};

U_BOOT_DEVICE(sunxi_dw_hdmi) = {
	.name = "sun50i_dw_hdmi"
};
