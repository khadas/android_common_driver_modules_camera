/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2020 Amlogic or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt)  "aml-csiphy:%s:%d: " fmt, __func__, __LINE__

#include "../aml_t7_csiphy.h"
#include "aml_t7_csiphy_hw.h"

#define CYCLE_TIME 5

//#define CSI_B_USING_CLK_D  1

static int mipi_reg_write(void *c_dev, int mod, u32 addr, u32 val)
{
	int rtn = -1;
	void __iomem *base = NULL;
	struct csiphy_dev_t *csiphy_dev = c_dev;

	switch (mod) {
	case DPHY_MD:
		base = csiphy_dev->csi_dphy;
	break;
	case HOST_MD:
		base = csiphy_dev->csi_host;
	break;
	case APHY_MD:
		base = csiphy_dev->csi_aphy;
	break;
	default:
		pr_err("Error input idx\n");
	return rtn;
	}

	writel(val, base + addr);

	return 0;
}

static int mipi_reg_read(void *c_dev, int mod, u32 addr)
{
	int rtn = -1;
	void __iomem *base = NULL;
	struct csiphy_dev_t *csiphy_dev = c_dev;

	switch (mod) {
	case DPHY_MD:
		base = csiphy_dev->csi_dphy;
	break;
	case HOST_MD:
		base = csiphy_dev->csi_host;
	break;
	case APHY_MD:
		base = csiphy_dev->csi_aphy;
	break;
	default:
		pr_err("Error input idx\n");
	return rtn;
	}

	return readl(base + addr);
}


static int aphy_cfg(void *c_dev, int bps, int lanes)
{
	u32 module = APHY_MD;
	struct csiphy_dev_t *csiphy_dev = c_dev;

	switch (csiphy_dev->index) {
	case 0:
		mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL0, 0x3f425c00);
		if (lanes == 4)
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL1, 0x033a0000);
		else if (lanes == 2) {
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL1, 0x333a0000);
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL2, 0x03800000);
		}
	break;
	case 1:
		mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL4, 0x3f425c00);
		if (lanes == 4) {
#ifdef CSI_B_USING_CLK_D
			// hs hsd0 hsd1 hsd2 hsd3 use hsout 5 or 2 clk;
			//3f - 00111111 - use hsout5_ck
			//03 - 00000011 - use hsout2_ck
			pr_info("722 use clk-1");
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL5, 0x3f3a0000);
#else
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL5, 0x33a0000);
#endif
		} else if (lanes == 2) {
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL5, 0x333a0000);
			mipi_reg_write(c_dev, module, MIPI_CSI_PHY_CNTL6, 0x3800000);
		}
	break;
	default:
		pr_err("Error csiphy index %u\n", csiphy_dev->index);
	break;
	}

	return 0;
}

static int dphy_cfg(void *c_dev, u32 bps, int lanes)
{
	u32 ui_val = 0;
	u32 settle = 0;
	u32 module = DPHY_MD;
	struct csiphy_dev_t *csiphy_dev = c_dev;

	ui_val = 1000 / bps;
	if ((1000 % bps) != 0) {
		ui_val += 1;
	}

	settle = (85 + 145 + (16 * ui_val)) / 2;
	settle = settle / CYCLE_TIME;

	mipi_reg_write(c_dev, module, MIPI_PHY_CLK_LANE_CTRL ,0x3d8);// 0x3d8 continus mode; 0x58
	mipi_reg_write(c_dev, module, MIPI_PHY_TCLK_MISS ,0x9);
	mipi_reg_write(c_dev, module, MIPI_PHY_TCLK_SETTLE, 0x1f);
	mipi_reg_write(c_dev, module, MIPI_PHY_THS_EXIT ,0x08);   // hs exit = 160 ns --(x>100ns)
	mipi_reg_write(c_dev, module, MIPI_PHY_THS_SKIP ,0xa);   // hs skip = 55 ns --(40ns<x<55ns+4*UI)
	mipi_reg_write(c_dev, module, MIPI_PHY_THS_SETTLE ,settle);   //85ns ~145ns.
	mipi_reg_write(c_dev, module, MIPI_PHY_TINIT ,0x4e20);  // >100us
	mipi_reg_write(c_dev, module, MIPI_PHY_TMBIAS ,0x100);
	mipi_reg_write(c_dev, module, MIPI_PHY_TULPS_C ,0x1000);
	mipi_reg_write(c_dev, module, MIPI_PHY_TULPS_S ,0x100);
	mipi_reg_write(c_dev, module, MIPI_PHY_TLP_EN_W ,0x0c);
	mipi_reg_write(c_dev, module, MIPI_PHY_TLPOK ,0x100);
	mipi_reg_write(c_dev, module, MIPI_PHY_TWD_INIT ,0x400000);
	mipi_reg_write(c_dev, module, MIPI_PHY_TWD_HS ,0x400000);
	mipi_reg_write(c_dev, module, MIPI_PHY_DATA_LANE_CTRL , 0x0);
	mipi_reg_write(c_dev, module, MIPI_PHY_DATA_LANE_CTRL1 , 0x3 | (0x1f << 2 ) | (0x3 << 7));      // enable data lanes pipe line and hs sync bit err.
	switch (csiphy_dev->index) {
	case 0:
		if (lanes == 2) {
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL0 , 0x000001ff);      //config input mux
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL1 , 0x000201ff);
		} else if (lanes == 4) {
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL0 , 0x00000123); 	 //config input mux
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL1 , 0x00020123);

		}
	break;
	case 1:
		if (lanes == 2) {
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL0 , 0x000001ff);      //config input mux
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL1 , 0x000201ff);
		} else if (lanes == 4) {
#ifdef  CSI_B_USING_CLK_D
			pr_info("722 use clk-1");
			// for 96722
			// bit 17:16 select clk: 0b01 for analog clk1
			// SFEN0 bit 15:12 select analog data 0
			// SFEN1 bit 11:8 select analog data 1
			// SFEN2 bit 7:4 select analog data 2
			// SFEN3 bit 3:0 select analog data 3
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL0 , 0x00010123);    //config input mux

			// bit 17 analog clk1 input: 0 from D-PHY SCNN; clk1 take effects.
			// bit 16 analog clk0 input: 1 from input 0

			// bit 15:12 analog data lane 0 from SFEN0
			// bit 11:8 analog data lane 1 from SFEN1
			// bit 7:4 analog data lane 2 from SFEN2
			// bit 3:0 analog data lane 3 from SFEN3
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL1 , 0x00010123);
#else
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL0 , 0x00000123); 	 //config input mux
			mipi_reg_write(c_dev, module, MIPI_PHY_MUX_CTRL1 , 0x00020123);
#endif
		}
	break;
	default:
		pr_err("Error csiphy index %u\n", csiphy_dev->index);
	break;
	}

	mipi_reg_write(c_dev, module, MIPI_PHY_CTRL, 0);          //  (0 << 9) | (((~chan) & 0xf ) << 5) | 0 << 4 | ((~chan) & 0xf) );

	return 0;
}

static int host_cfg(void *c_dev, u32 lanes)
{
	u32 module = HOST_MD;

	mipi_reg_write(c_dev, module, CSI2_HOST_CSI2_RESETN, 0); // csi2 reset
	mipi_reg_write(c_dev, module, CSI2_HOST_CSI2_RESETN, 0xffffffff); // release csi2 reset
	mipi_reg_write(c_dev, module, CSI2_HOST_DPHY_RSTZ, 0xffffffff); // release DPHY reset
	mipi_reg_write(c_dev, module, CSI2_HOST_N_LANES, (lanes - 1) & 3);  //set lanes
	mipi_reg_write(c_dev, module, CSI2_HOST_PHY_SHUTDOWNZ, 0xffffffff); // enable power

	return 0;
}

static void csiphy_reset(void *c_dev)
{
	u32 data = 0x1f;
	u32 module;

	data |= 1 << 31;
	module = DPHY_MD;
	mipi_reg_write(c_dev, module, MIPI_PHY_CTRL, data);

	module = HOST_MD;
	mipi_reg_write(c_dev, module, CSI2_HOST_PHY_SHUTDOWNZ, 0); // enable power
	mipi_reg_write(c_dev, module, CSI2_HOST_DPHY_RSTZ, 0); // release DPHY reset
	mipi_reg_write(c_dev, module, CSI2_HOST_CSI2_RESETN, 0); // csi2 reset
}

static u32 csiphy_hw_version(void *c_dev)
{
	u32 version = 0xc0ff;

	return version;
}

static void csiphy_hw_reset(void *c_dev)
{
	csiphy_reset(c_dev);
}

static int csiphy_hw_start(void *c_dev, int lanes, s64 link_freq)
{
	u32 bps = 0;
	u64 freq = 0;

	freq = (u64)link_freq;

	bps = link_freq / 1000000;

	pr_info("csiphy hw bps %d, lanes %d \n", bps, lanes);

	aphy_cfg(c_dev, bps, lanes);
	dphy_cfg(c_dev, bps, lanes);
	host_cfg(c_dev, lanes);

	pr_info("Success csiphy hw start\n");

	return 0;
}

static void csiphy_hw_stop(void *c_dev)
{
	csiphy_reset(c_dev);

	pr_info("Success csiphy hw stop\n");

	return;
}


void csiphy_hw_dump_reg(struct csiphy_dev_t * ctx)
{
	uint32_t  val;
	uint32_t  addr;
	int i = 0;

	pr_err( "begin	= dump csiphy hw regs\n");

	pr_err( "begin	=  aphy \n");
	for (i = 0; i <= (MIPI_CSI_PHY_CNTL7>>2) ; ++i) {
		addr = i << 2;
		val = mipi_reg_read( ctx, APHY_MD, addr);
		pr_err( "addr 0x%08x, val 0x%08x \n", addr, val);
	}

	pr_err( "begin	=  dphy \n");
	for (i = 0; i <= (MIPI_PHY_INT_STS>>2) ; ++i) {
		addr = i << 2;
		val = mipi_reg_read( ctx, DPHY_MD, addr);
		pr_err( "addr 0x%08x, val 0x%08x \n", addr, val);
	}

	addr = MIPI_PHY_MUX_CTRL0;
	val = mipi_reg_read( ctx, DPHY_MD, addr);
	pr_err( "addr 0x%08x, val 0x%08x \n", addr, val);

	addr = MIPI_PHY_MUX_CTRL1;
	val = mipi_reg_read( ctx, DPHY_MD, addr);
	pr_err( "addr 0x%08x, val 0x%08x \n", addr, val);

	pr_err( "begin = host \n");
	for (i = 0; i <= (CSI2_HOST_PHY_TST_CTRL1>>2) ; ++i) {
		addr = i << 2;
		val = mipi_reg_read( ctx, HOST_MD, addr);
		pr_err( "addr 0x%08x, val 0x%08x \n", addr, val);
	}

	pr_err( "end  == dump adap hw regs\n");
}


const struct csiphy_dev_ops csiphy_dev_hw_ops = {
	.hw_reset = csiphy_hw_reset,
	.hw_version = csiphy_hw_version,
	.hw_start = csiphy_hw_start,
	.hw_stop = csiphy_hw_stop,
};


