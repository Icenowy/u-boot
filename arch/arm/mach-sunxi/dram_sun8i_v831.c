// SPDX-License-Identifier: GPL-2.0+
/*
 * sun8i V831 platform dram controller init
 *
 * (C) Copyright 2007-2015 Allwinner Technology Co.
 *                         Jerry Wang <wangflord@allwinnertech.com>
 * (C) Copyright 2015      Vishnu Patekar <vishnupatekar0510@gmail.com>
 * (C) Copyright 2015      Hans de Goede <hdegoede@redhat.com>
 * (C) Copyright 2015      Jens Kuske <jenskuske@gmail.com>
 * (C) Copyright 2020      Icenowy Zheng <icenowy@aosc.io>
 */
#include <common.h>
#include <init.h>
#include <log.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/arch/cpu.h>
#include <linux/delay.h>
#include <linux/kconfig.h>

static void mctl_phy_init(u32 val)
{
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	writel(val | PIR_INIT, &mctl_ctl->pir);
	mctl_await_completion(&mctl_ctl->pgsr[0], PGSR_INIT_DONE, 0x1);
}

static void mctl_set_bit_delays(struct dram_para *para)
{
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;
	int i, j;

	clrbits_le32(&mctl_ctl->pgcr[0], 1 << 26);

	for (i = 0; i < NR_OF_BYTE_LANES; i++)
		for (j = 0; j < LINES_PER_BYTE_LANE; j++)
			writel(DXBDLR_WRITE_DELAY(para->dx_write_delays[i][j]) |
			       DXBDLR_READ_DELAY(para->dx_read_delays[i][j]),
			       &mctl_ctl->dx[i].bdlr[j]);

	for (i = 0; i < 31; i++)
		writel(ACBDLR_WRITE_DELAY(para->ac_delays[i]),
		       &mctl_ctl->acbdlr[i]);

	/* DQSn, DMn, DQn output enable bit delay */
	writel(0x4 << 24, &mctl_ctl->dx[0].sdlr);
	writel(0x2 << 24, &mctl_ctl->dx[1].sdlr);

	setbits_le32(&mctl_ctl->pgcr[0], 1 << 26);
}

static void mctl_apply_para(struct dram_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	clrsetbits_le32(&mctl_com->unk_0x008, 0x3f00, 0x2000);

	writel(MCTL_CR_BL8 | MCTL_CR_INTERLEAVED |
#if defined CONFIG_SUNXI_DRAM_DDR2
	       MCTL_CR_DDR2 | MCTL_CR_2T |
#else
#error Unsupported DRAM type!
#endif
	       (para->bank_bits == 3 ? MCTL_CR_EIGHT_BANKS : MCTL_CR_FOUR_BANKS) |
	       MCTL_CR_BUS_FULL_WIDTH(para->bus_full_width) |
	       (para->dual_rank ? MCTL_CR_DUAL_RANK : MCTL_CR_SINGLE_RANK) |
	       MCTL_CR_PAGE_SIZE(para->page_size) |
	       MCTL_CR_ROW_BITS(para->row_bits), &mctl_com->cr);

	if (para->dual_rank)
		writel(0x00000303, &mctl_ctl->odtmap);
	else
		writel(0x00000201, &mctl_ctl->odtmap);

	if (!para->bus_full_width)
		writel(0x0, &mctl_ctl->dx[1].gcr);

	/* TODO: asymmetric dual rank */
}

static void mctl_sys_init(struct dram_para *para)
{
	struct sunxi_ccm_reg * const ccm =
			(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	/* Put all DRAM-related blocks to reset state */
	clrbits_le32(&ccm->mbus_cfg, MBUS_ENABLE | MBUS_RESET);
	clrbits_le32(&ccm->dram_gate_reset, BIT(0));
	udelay(5);
	writel(0, &ccm->dram_gate_reset);
	clrbits_le32(&ccm->pll5_cfg, CCM_PLL5_CTRL_EN);
	clrbits_le32(&ccm->dram_clk_cfg, DRAM_MOD_RESET);

	udelay(5);

	/* Set PLL5 rate to doubled DRAM clock rate */
	writel(CCM_PLL5_CTRL_EN | CCM_PLL5_LOCK_EN | CCM_PLL5_OUT_EN |
	       CCM_PLL5_CTRL_N(CONFIG_DRAM_CLK * 2 / 24 - 1), &ccm->pll5_cfg);
	mctl_await_completion(&ccm->pll5_cfg, CCM_PLL5_LOCK, CCM_PLL5_LOCK);

	/* Configure DRAM mod clock */
	writel(DRAM_CLK_SRC_PLL5, &ccm->dram_clk_cfg);
	setbits_le32(&ccm->dram_clk_cfg, DRAM_CLK_UPDATE);
	writel(BIT(RESET_SHIFT), &ccm->dram_gate_reset);
	udelay(5);
	setbits_le32(&ccm->dram_gate_reset, BIT(0));

	/* Disable all masters */
	writel(1, &mctl_com->maer0);
	writel(0, &mctl_com->maer1);
	writel(0, &mctl_com->maer2);

	/* Configure MBUS and enable DRAM mod reset */
	setbits_le32(&ccm->mbus_cfg, MBUS_RESET);
	setbits_le32(&ccm->mbus_cfg, MBUS_ENABLE);
	setbits_le32(&ccm->dram_clk_cfg, DRAM_MOD_RESET);
	udelay(5);

	/* Unknown hack from the BSP, which enables access of mctl_ctl regs */
	writel(0x8000, &mctl_ctl->clken);
}

/* These are more guessed based on some Allwinner code. */
#define DX_GCR_ODT_DYNAMIC	(0x0 << 4)
#define DX_GCR_ODT_ALWAYS_ON	(0x1 << 4)
#define DX_GCR_ODT_OFF		(0x2 << 4)

static int mctl_channel_init(struct dram_para *para)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	unsigned int i;

	clrsetbits_le32(&mctl_ctl->iovcr[0], 0x7f7f7f7f, 0x48484848);
	clrsetbits_le32(&mctl_ctl->iovcr[1], 0x7f, 0x48);

	mctl_apply_para(para);
	/* TODO: some magic bits are poked at 0x04002500 when DDR3 */
	mctl_set_timing_params(SOCID_V831, para);

	clrsetbits_le32(&mctl_com->tmr, 0xfff, (CONFIG_DRAM_CLK / 2));

	/* dphy & aphy phase select ? */
	clrsetbits_le32(&mctl_ctl->pgcr[2], (0x3 << 10) | (0x3 << 8),
			(0x0 << 10) | (0x3 << 8));

	/* set dramc odt */
	for (i = 0; i < 4; i++) {
		u32 clearmask = 0xf61e;
		u32 setmask = IS_ENABLED(CONFIG_DRAM_ODT_EN) ?
				DX_GCR_ODT_DYNAMIC : DX_GCR_ODT_OFF;
		if (CONFIG_DRAM_CLK > 672)
			setmask |= 0x400;

		clrsetbits_le32(&mctl_ctl->dx[i].gcr, clearmask, setmask);
	}

	/* AC PDR should always ON */
	clrsetbits_le32(&mctl_ctl->aciocr, 0, 0x1 << 1);

	mctl_set_bit_delays(para);

	/* set DQS auto gating PD mode */
	setbits_le32(&mctl_ctl->pgcr[2], 0x3 << 6);

	/* data training configuration */
	clrsetbits_le32(&mctl_ctl->dtcr, 0x0fffffff,
			(para->dual_rank ? 0x3 : 0x1) << 24 | 1);

	/* TODO: BSP doing something about standby here */
	udelay(50);

	clrsetbits_le32(&mctl_ctl->zqcr, 0x3ffffff, 0x2000000 | CONFIG_DRAM_ZQ);

	/* TODO: non-DDR2 types */
	mctl_phy_init(PIR_ZCAL | PIR_PLLINIT | PIR_DCAL | PIR_PHYRST |
		      PIR_QSGATE | PIR_DRAMINIT);

	/* detect ranks and bus width */
	if (readl(&mctl_ctl->pgsr[0]) & (0xfe << 20)) {
		/* only one rank */
		if (((readl(&mctl_ctl->dx[0].gsr[0]) >> 24) & 0x2)
		    ) {
			clrsetbits_le32(&mctl_ctl->dtcr, 0xf << 24, 0x1 << 24);
			para->dual_rank = 0;
		}

		/* only half DQ width */
		if ((readl(&mctl_ctl->dx[1].gsr[0]) >> 24) & 0x1) {
			writel(0x0, &mctl_ctl->dx[1].gcr);
			para->bus_full_width = 0;
		}

		mctl_apply_para(para);
		udelay(20);

		/* re-train */
		mctl_phy_init(PIR_QSGATE);
		if (readl(&mctl_ctl->pgsr[0]) & (0xfe << 20))
			return 1;
	}

	/* check the dramc status */
	mctl_await_completion(&mctl_ctl->statr, 0x1, 0x1);

	/* liuke added for refresh debug */
	setbits_le32(&mctl_ctl->rfshctl0, 0x1 << 31);
	udelay(10);
	clrbits_le32(&mctl_ctl->rfshctl0, 0x1 << 31);
	udelay(10);

	setbits_le32(&mctl_com->unk_0x014, BIT(31));

	clrbits_le32(&mctl_ctl->pgcr[3], 0x06000000);

	return 0;
}

static void mctl_auto_detect_dram_size(struct dram_para *para)
{
	/* detect row address bits */
	para->page_size = 512;
	para->row_bits = 16;
	para->bank_bits = 2;
	mctl_apply_para(para);

	for (para->row_bits = 11; para->row_bits < 13; para->row_bits++)
		if (mctl_mem_matches((1 << (para->row_bits + para->bank_bits)) * para->page_size))
			break;

	/* detect bank address bits */
	para->bank_bits = 3;
	mctl_apply_para(para);

	for (para->bank_bits = 2; para->bank_bits < 2; para->bank_bits++)
		if (mctl_mem_matches((1 << para->bank_bits) * para->page_size))
			break;

	/* detect page size */
	para->page_size = 8192;
	mctl_apply_para(para);

	for (para->page_size = 512; para->page_size < 2048; para->page_size *= 2)
		if (mctl_mem_matches(para->page_size))
			break;
}

#define SUN8I_V831_DX_READ_DELAYS					\
	{{  6,  6,  6,  6,  6,  6,  6,  6,  6,  0,  0 },	\
	 {  4,  4,  4,  4,  4,  4,  4,  4,  4,  0,  0 },	\
	 {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },	\
	 {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }}
#define SUN8I_V831_DX_WRITE_DELAYS				\
	{{  0,  0,  0,  0,  0,  0,  0,  0,  0,  4,  0 },	\
	 {  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  0 },	\
	 {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },	\
	 {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }}
#define SUN8I_V831_AC_DELAYS					\
	{  0,  0,  0,  0,  0,  0,  0,  0,			\
	   0,  0,  0,  0,  0,  0,  0,  0,			\
	   0,  0,  0,  0,  0,  0,  0,  0,			\
	   0,  0,  0,  0,  0,  0,  0      }

unsigned long sunxi_dram_init(void)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;

	struct dram_para para = {
		.dual_rank = 1,
		.bus_full_width = 1,
		.row_bits = 16,
		.bank_bits = 3,
		.page_size = 8192,

		.dx_read_delays  = SUN8I_V831_DX_READ_DELAYS,
		.dx_write_delays = SUN8I_V831_DX_WRITE_DELAYS,
		.ac_delays	 = SUN8I_V831_AC_DELAYS,
	};

	/* Unknown magic */
	writel(0x10, 0x07010250);
	writel(0x330000, 0x07010310);
	writel(0x330003, 0x07010310);

	mctl_sys_init(&para);
	if (mctl_channel_init(&para))
		return 0;

	udelay(1);

	clrbits_le32(&mctl_ctl->unk_0x0a0, 0xffff);
	clrbits_le32(&mctl_ctl->pwrctl, 0x1);

	clrbits_le32(&mctl_ctl->pgcr[0], 0xf000);

	/* power down zq calibration module for power save */
	setbits_le32(&mctl_ctl->zqcr, ZQCR_PWRDOWN);

	setbits_le32(&mctl_ctl->vtfcr, 0x300);

	/* DQ hold disable (tpr13[26] == 1) */
	clrbits_le32(&mctl_ctl->pgcr[2], (1 << 13));

	mctl_auto_detect_dram_size(&para);
	mctl_apply_para(&para);

	/* enable master access */
	writel(0xffffffff, &mctl_com->maer0);
	writel(0x7f, &mctl_com->maer1);
	writel(0xffff, &mctl_com->maer2);

	return (1UL << (para.row_bits + para.bank_bits)) * para.page_size *
	       (para.dual_rank ? 2 : 1);
}
