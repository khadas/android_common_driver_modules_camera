/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2018 Amlogic or its affiliates
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
#define pr_fmt(fmt) "AM_SC2: " fmt

#include "system_am_sc2.h"
#include <linux/irqreturn.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-contiguous.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_fdt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>


#define AM_SC_NAME "amlogic, isp-sc"
static int buffer_id;

#define ENABLE_SC_BOTTOM_HALF_TASKLET

#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
// tasklet structure
struct sc_tasklet_t {
    struct tasklet_struct tasklet_obj;
    struct kfifo sc_fifo_out;
};
static int frame_id = 0;
static struct sc_tasklet_t sc_tasklet;
#endif

static bool stop_flag = true;
static tframe_t* temp_buf = NULL;
static tframe_t* pre_frame[3];
static u32 frame_delay = 2;

static const int isp_filt_coef0[] =   //bicubic
{
    0x00800000,
    0x007f0100,
    0xff7f0200,
    0xfe7f0300,
    0xfd7e0500,
    0xfc7e0600,
    0xfb7d0800,
    0xfb7c0900,
    0xfa7b0b00,
    0xfa7a0dff,
    0xf9790fff,
    0xf97711ff,
    0xf87613ff,
    0xf87416fe,
    0xf87218fe,
    0xf8701afe,
    0xf76f1dfd,
    0xf76d1ffd,
    0xf76b21fd,
    0xf76824fd,
    0xf76627fc,
    0xf76429fc,
    0xf7612cfc,
    0xf75f2ffb,
    0xf75d31fb,
    0xf75a34fb,
    0xf75837fa,
    0xf7553afa,
    0xf8523cfa,
    0xf8503ff9,
    0xf84d42f9,
    0xf84a45f9,
    0xf84848f8
};

#if 0
const int isp_filt_coef1[] =  // 2point bilinear
{
    0x00800000,
    0x007e0200,
    0x007c0400,
    0x007a0600,
    0x00780800,
    0x00760a00,
    0x00740c00,
    0x00720e00,
    0x00701000,
    0x006e1200,
    0x006c1400,
    0x006a1600,
    0x00681800,
    0x00661a00,
    0x00641c00,
    0x00621e00,
    0x00602000,
    0x005e2200,
    0x005c2400,
    0x005a2600,
    0x00582800,
    0x00562a00,
    0x00542c00,
    0x00522e00,
    0x00503000,
    0x004e3200,
    0x004c3400,
    0x004a3600,
    0x00483800,
    0x00463a00,
    0x00443c00,
    0x00423e00,
    0x00404000
};
#endif

static int isp_filt_coef2[] =  // 2point bilinear, bank_length == 2
{
    0x80000000,
    0x7e020000,
    0x7c040000,
    0x7a060000,
    0x78080000,
    0x760a0000,
    0x740c0000,
    0x720e0000,
    0x70100000,
    0x6e120000,
    0x6c140000,
    0x6a160000,
    0x68180000,
    0x661a0000,
    0x641c0000,
    0x621e0000,
    0x60200000,
    0x5e220000,
    0x5c240000,
    0x5a260000,
    0x58280000,
    0x562a0000,
    0x542c0000,
    0x522e0000,
    0x50300000,
    0x4e320000,
    0x4c340000,
    0x4a360000,
    0x48380000,
    0x463a0000,
    0x443c0000,
    0x423e0000,
    0x40400000
};

#define ZOOM_BITS       20
#define PHASE_BITS      16

typedef enum {
    F2V_IT2IT = 0,
    F2V_IB2IB,
    F2V_IT2IB,
    F2V_IB2IT,
    F2V_P2IT,
    F2V_P2IB,
    F2V_IT2P,
    F2V_IB2P,
    F2V_P2P,
    F2V_TYPE_MAX
} f2v_vphase_type_t;   /* frame to video conversion type */

typedef struct {
    u8 rcv_num; //0~15
    u8 rpt_num; // 0~3
    u16 phase;
    //s8 repeat_skip_chroma;
    //u8 phase_chroma;
} f2v_vphase_t;

typedef struct ISP_MIF_TYPE {
    int reg_rev_x;
    int reg_rev_y;
    int reg_little_endian;
    int reg_bit10_mode;
    int reg_enable_3ch;
    int reg_only_1ch;
    int reg_words_lim;
    int reg_burst_lim;
    int reg_rgb_mode;
    int reg_hconv_mode;
    int reg_vconv_mode;
    u16 reg_hsizem1;
    u16 reg_vsizem1;
    u16 reg_start_x;
    u16 reg_start_y;
    u16 reg_end_x;
    u16 reg_end_y;
    int reg_pingpong_en;
    u16 reg_canvas_strb_luma;
    u16 reg_canvas_strb_chroma;
    u16 reg_canvas_strb_r;
    u32 reg_canvas_baddr_luma;
    u32 reg_canvas_baddr_chroma;
    u32 reg_canvas_baddr_r;
    u32 reg_canvas_baddr_luma_other;
    u32 reg_canvas_baddr_chroma_other;
    u32 reg_canvas_baddr_r_other;
} ISP_MIF_t;

static const u8 f2v_420_in_pos_luma[F2V_TYPE_MAX] = {0, 2, 0, 2, 0, 0, 0, 2, 0};
//static const u8 f2v_420_in_pos_chroma[F2V_TYPE_MAX] = {1, 5, 1, 5, 2, 2, 1, 5, 2};
static const u8 f2v_420_out_pos[F2V_TYPE_MAX] = {0, 2, 2, 0, 0, 2, 0, 0, 0};

static int rgb2yuvpre[3] = {0, 0, 0};
static int rgb2yuvpos[3] = {64, 512, 512}; //BT601_fullrange
static int rgb2yuvpos_invert[3] = {512,  512, 64};
static int yuv2rgbpre[3] = {-64, -512, -512};
static int yuv2rgbpos[3] = {0, 0, 0};
//static int rgb2ycbcr[15] = {230,594,52,-125,-323,448,448,-412,-36,0,0,0,0,0,0};
static int rgb2ycbcr[15] = {263,516,100,-152,-298,450,450,-377,-73,0,0,0,0,0,0}; //BT601_fullrange
static int rgb2ycbcr_invert[15] = {-125, -323, 448, 448,-412,-36, 230, 594, 52, 0,0,0,0,0,0};
static int ycbcr2rgb[15] = {1197,0,1726,1197,-193,-669,1197,2202,0,0,0,0,0,0,0};
static ISP_MIF_t isp_frame = {
    0, // int  reg_rev_x
    0, // int  reg_rev_y
    1, // int  reg_little_endian
    0, // int  reg_bit10_mode
    0, // int  reg_enable_3ch
    0, // int  reg_only_1ch
    4, // int  reg_words_lim
    3, // int  reg_burst_lim
    1, // int  reg_rgb_mode
    2, // int  reg_hconv_mode
    2, // int  reg_vconv_mode
    1279, // int  reg_hsizem1
    719, // int  reg_vsizem1
    0, // int  reg_start_x
    0, // int  reg_start_y
    1279, // int  reg_end_x
    719, // int  reg_end_y
    1, // pingpong en
    0x780, // int16_t reg_canvas_strb_luma
    0x780, // int16_t reg_canvas_strb_chroma
    0x780, // int16_t reg_canvas_strb_r
    0x4000000, // int32_t reg_canvas_baddr_luma
    0x5000000, // int32_t reg_canvas_baddr_chroma
    0x6000000, // int32_t reg_canvas_baddr_r
    0x7000000, // int32_t reg_canvas_baddr_luma_other
    0x8000000, // int32_t reg_canvas_baddr_chroma_other
    0x9000000 // int32_t reg_canvas_baddr_r_other
};

static u32 start_delay_th = 2;
static u32 start_delay_cnt;
static u8 ch_mode = 0; // 0: normal, 1: force 1 ch, 3: force 3 ch;
static u32 last_end_frame = 0;
extern u32 sc1_isr_count;
u32 sc2_isr_count;
static struct am_sc2 *g_sc;

static void f2v_get_vertical_phase(
    u32 zoom_ratio,
    f2v_vphase_type_t type,
    u8 bank_length,
    f2v_vphase_t *vphase)
{
    int offset_in, offset_out;

    /* luma */
    offset_in = f2v_420_in_pos_luma[type] << PHASE_BITS;
    offset_out = (f2v_420_out_pos[type] * zoom_ratio)
        >> (ZOOM_BITS - PHASE_BITS);

    vphase->rcv_num = bank_length;
    if (bank_length == 4 || bank_length == 3)
        vphase->rpt_num = 1;
    else
        vphase->rpt_num = 0;

    if (offset_in > offset_out) {
        vphase->rpt_num = vphase->rpt_num + 1;
        vphase->phase =
            ((4 << PHASE_BITS) + offset_out - offset_in) >> 2;
    } else {
        while ((offset_in + (4 << PHASE_BITS)) <= offset_out) {
            if (vphase->rpt_num == 1)
                vphase->rpt_num = 0;
            else
                vphase->rcv_num++;
            offset_in += 4 << PHASE_BITS;
        }
        vphase->phase = (offset_out - offset_in) >> 2;
    }
}

static inline void update_wr_reg_bits(
    unsigned int reg,
    unsigned int mask,
    unsigned int val)
{
    unsigned int tmp, orig;
    void __iomem *base = g_sc->base_addr;

    if (base !=  NULL) {
        orig = readl(base + reg);
        tmp = orig & ~mask;
        tmp |= val & mask;
        writel(tmp, base + reg);
    }
}

static inline void sc_wr_reg_bits(
    unsigned int adr, unsigned int val,
    unsigned int start, unsigned int len)
{
    update_wr_reg_bits(adr,
        ((1 << len) - 1) << start, val << start);
}

static inline void sc_reg_wr(
    int addr, uint32_t val)
{
    void __iomem *base = g_sc->base_addr;

    if (base != NULL) {
        base = base + addr;
        writel(val, base);
    } else
        pr_err("isp-sc write register failed.\n");

}

static inline void sc_reg_rd(
    int addr, uint32_t *val)
{
    void __iomem *base = g_sc->base_addr;

    if (base != NULL && val) {
        base = base + addr;
        *val = readl(base);
    } else
        pr_err("isp-sc read register failed.\n");

}

static inline uint32_t sc_get_reg(int addr)
{
    void __iomem *base = g_sc->base_addr;
    uint32_t val = 0;

    if (base != NULL) {
        base = base + addr;
        val = readl(base);
    } else
        pr_err("isp-sc read register failed.\n");

    return val;
}

static void isp_sc_setting(
    u32 src_w, u32 src_h,
    u32 dst_w, u32 dst_h)
{
    f2v_vphase_t vphase;
    s32 i;
    s32 hsc_en, vsc_en;
    s32 prehsc_en,prevsc_en;
    s32 vsc_double_line_mode;
    u32 p_src_w, p_src_h;
    u32 vert_phase_step, horz_phase_step;
    u8 top_rcv_num, bot_rcv_num;
    u8 top_rpt_num, bot_rpt_num;
    u16 top_vphase, bot_vphase;
    u8 is_frame;
    s32  vert_bank_length = 4;
    s32 *filt_coef0 = (s32 *)&isp_filt_coef0[0];
    //s32 *filt_coef1 = (s32 *)&isp_filt_coef1[0];
    s32 *filt_coef2 = (s32 *)&isp_filt_coef2[0];
    f2v_vphase_type_t top_conv_type = F2V_P2P;
    f2v_vphase_type_t bot_conv_type = F2V_P2P;

    prehsc_en = 0;
    prevsc_en = 0;
    vsc_double_line_mode = 0;

    if (src_h != dst_h)
        vsc_en = 1;
    else
        vsc_en = 0;
    if (src_w != dst_w)
        hsc_en = 1;
    else
        hsc_en = 0;

    p_src_w = prehsc_en ? ((src_w + 1) >> 1) : src_w;
    p_src_h = prevsc_en ? ((src_h + 1) >> 1) : src_h;

    sc_reg_wr(ISP_SC_HOLD_LINE, 0x10);

    if (p_src_w > 2048) {
        //force vert bank length = 2
        vert_bank_length = 2;
        vsc_double_line_mode = 1;
    }

    if (stop_flag) {
        //write vert filter coefs
        sc_reg_wr(ISP_SC_COEF_IDX, 0x0000);
        for (i = 0; i < 33; i++) {
            if (vert_bank_length == 2)
                sc_reg_wr(ISP_SC_COEF, filt_coef2[i]); //bilinear
            else
                sc_reg_wr(ISP_SC_COEF, filt_coef0[i]); //bicubic
        }

        //write horz filter coefs
        sc_reg_wr(ISP_SC_COEF_IDX, 0x0100);
        for (i = 0; i < 33; i++) {
            sc_reg_wr(ISP_SC_COEF, filt_coef0[i]); //bicubic
        }
    }

    if (p_src_h > 2048)
        vert_phase_step =
            ((p_src_h << 18) / dst_h) << 2;
    else
        vert_phase_step = (p_src_h << 20) / dst_h;

    if (p_src_w > 2048)
        horz_phase_step =
            ((p_src_w << 18) / dst_w) << 2;
    else
        horz_phase_step = (p_src_w << 20) / dst_w;

    is_frame = (top_conv_type == F2V_IT2P)
        || (top_conv_type == F2V_IB2P)
        || (top_conv_type == F2V_P2P);

    if (is_frame) {
        f2v_get_vertical_phase(
            vert_phase_step, top_conv_type,
            vert_bank_length, &vphase);
        top_rcv_num = vphase.rcv_num;
        top_rpt_num = vphase.rpt_num;
        top_vphase  = vphase.phase;
        bot_rcv_num = 0;
        bot_rpt_num = 0;
        bot_vphase  = 0;
    } else {
        f2v_get_vertical_phase(
            vert_phase_step, top_conv_type,
            vert_bank_length, &vphase);
        top_rcv_num = vphase.rcv_num;
        top_rpt_num = vphase.rpt_num;
        top_vphase = vphase.phase;

        f2v_get_vertical_phase(
            vert_phase_step, bot_conv_type,
            vert_bank_length, &vphase);
        bot_rcv_num = vphase.rcv_num;
        bot_rpt_num = vphase.rpt_num;
        bot_vphase = vphase.phase;
    }

    vert_phase_step = (vert_phase_step << 4);
    horz_phase_step = (horz_phase_step << 4);

    sc_reg_wr(ISP_SC_LINE_IN_LENGTH, src_w);
    sc_reg_wr(ISP_SC_PIC_IN_HEIGHT, src_h);
    sc_reg_wr(ISP_VSC_REGION12_STARTP, 0);
    sc_reg_wr(ISP_VSC_REGION34_STARTP,
        ((dst_h << 16) | dst_h));
    sc_reg_wr(ISP_VSC_REGION4_ENDP, dst_h - 1);

    sc_reg_wr(ISP_VSC_START_PHASE_STEP, vert_phase_step);

    if (stop_flag) {
        sc_reg_wr(ISP_VSC_REGION0_PHASE_SLOPE, 0);
        sc_reg_wr(ISP_VSC_REGION1_PHASE_SLOPE, 0);
        sc_reg_wr(ISP_VSC_REGION3_PHASE_SLOPE, 0);
        sc_reg_wr(ISP_VSC_REGION4_PHASE_SLOPE, 0);
    }

    sc_reg_wr(ISP_VSC_PHASE_CTRL,
        (vsc_double_line_mode << 17) |
        ((!is_frame) << 16) |
        (0 << 15) |
        (bot_rpt_num << 13) |
        (bot_rcv_num << 8) |
        (0 << 7) |
        (top_rpt_num << 5) |
        (top_rcv_num << 0));
    sc_reg_wr(ISP_VSC_INI_PHASE,
        (bot_vphase << 16) | top_vphase);
    sc_reg_wr(ISP_HSC_REGION12_STARTP, 0);
    sc_reg_wr(ISP_HSC_REGION34_STARTP,
        (dst_w << 16) | dst_w);
    sc_reg_wr(ISP_HSC_REGION4_ENDP, dst_w - 1);

    sc_reg_wr(ISP_HSC_START_PHASE_STEP, horz_phase_step);

    if (stop_flag) {
        sc_reg_wr(ISP_HSC_REGION0_PHASE_SLOPE, 0);
        sc_reg_wr(ISP_HSC_REGION1_PHASE_SLOPE, 0);
        sc_reg_wr(ISP_HSC_REGION3_PHASE_SLOPE, 0);
        sc_reg_wr(ISP_HSC_REGION4_PHASE_SLOPE, 0);

        sc_reg_wr(ISP_HSC_PHASE_CTRL, (1 << 21) | (4 << 16) | 0);
    }

    sc_reg_wr(ISP_SC_MISC,
        (prevsc_en << 21) |
        (prehsc_en << 20) | // prehsc_en
        (prevsc_en << 19) | // prevsc_en
        (vsc_en << 18) | // vsc_en
        (hsc_en << 17) | // hsc_en
        (1 << 16) | // sc_top_en
        (1 << 15) | // vd1 sc out enable
        (0 << 12) | // horz nonlinear 4region enable
        (4 << 8) | // horz scaler bank length
        (0 << 5) | // vert scaler phase field mode enable
        (0 << 4) | // vert nonlinear 4region enable
        (vert_bank_length << 0));  // vert scaler bank length
}

static void isp_mtx_setting(s32 mode)
{
    s32 mat_conv_en = 0;
    s32 i, pre_offset[3] ={0, 0, 0}, post_offset[3]= {0, 0, 0};
    s32 mat_coef[15];
    bool invert = false;

    if ((ch_mode == 0) && (mode == 1)
        && (isp_frame.reg_rgb_mode == 1))
        invert = true;

    if (mode == 1) {
        mat_conv_en = 1;
        for (i = 0; i < 3; i++) {
            pre_offset[i] = rgb2yuvpre[i];
            post_offset[i] = (invert == true) ?
                rgb2yuvpos_invert[i] :
                rgb2yuvpos[i];
        }
        for (i = 0; i < 15; i++)
            mat_coef[i] = (invert == true) ?
                rgb2ycbcr_invert[i] :
                rgb2ycbcr[i];
    } else if (mode == 2) {
        mat_conv_en = 1;
        for (i = 0; i < 3; i++) {
            pre_offset[i] = yuv2rgbpre[i];
            post_offset[i] = yuv2rgbpos[i];
        }
        for (i = 0; i < 15; i++)
            mat_coef[i] = ycbcr2rgb[i];
    }
    sc_reg_wr(ISP_MATRIX_COEF00_01,
        (mat_coef[0 * 3 + 0] << 16) |
        (mat_coef[0 * 3 + 1] & 0x1FFF));
    sc_reg_wr(ISP_MATRIX_COEF02_10,
        (mat_coef[0 * 3 + 2] << 16) |
        (mat_coef[1 * 3 + 0] & 0x1FFF));
    sc_reg_wr(ISP_MATRIX_COEF11_12,
        (mat_coef[1 * 3 + 1] << 16) |
        (mat_coef[1 * 3 + 2] & 0x1FFF));
    sc_reg_wr(ISP_MATRIX_COEF20_21,
        (mat_coef[2 * 3 + 0] << 16) |
        (mat_coef[2 * 3 + 1] & 0x1FFF));
    sc_reg_wr(ISP_MATRIX_COEF22,
        mat_coef[2 * 3 + 2]);
    sc_reg_wr(ISP_MATRIX_OFFSET0_1,
        (post_offset[0] << 16) |
        (post_offset[1] & 0xFFF));
    sc_reg_wr(ISP_MATRIX_OFFSET2,
        post_offset[2]);
    sc_reg_wr(ISP_MATRIX_PRE_OFFSET0_1,
        (pre_offset[0] << 16) |
        (pre_offset[1] & 0xFFF));
    sc_reg_wr(ISP_MATRIX_PRE_OFFSET2,
        pre_offset[2]);
    sc_reg_wr(ISP_MATRIX_EN_CTRL,
        mat_conv_en);
}

static void isp_mif_setting(ISP_MIF_t *wr_mif)
{
    u8 swap_uv = 0;
    if (g_sc->info.in_fmt == RGB24) {
        if (g_sc->info.out_fmt == NV12_YUV) {
            swap_uv = 1;
        } else if (g_sc->info.out_fmt == NV12_YVU) {
            swap_uv = 0;
        }
    } else if (g_sc->info.in_fmt == AYUV) {
        if ((g_sc->info.out_fmt == NV12_YUV) ||
            (g_sc->info.out_fmt == NV12_YVU)) {
           swap_uv = 1;
        }
    }

    sc_wr_reg_bits(ISP_SCWR_MIF_CTRL0,
        ((1 << 0) |
        (0 << 1) |
        (0 << 2) |
        (0 << 3) |
        (wr_mif->reg_rev_x << 4) |
        (wr_mif->reg_rev_y << 5) |
        (wr_mif->reg_little_endian << 6) |
        (0 << 7) |
        (1 << 8) |
        (swap_uv << 9) |
        (0 <<10) |
        (wr_mif->reg_bit10_mode << 11) |
        (wr_mif->reg_enable_3ch << 12) |
        (wr_mif->reg_only_1ch << 13)),
        0, 14);
    sc_wr_reg_bits(ISP_SCWR_MIF_CTRL0,
        wr_mif->reg_pingpong_en, 17, 1);
    sc_reg_wr(ISP_SCWR_MIF_CTRL1,
        wr_mif->reg_words_lim |
        (wr_mif->reg_burst_lim << 4) |
        (wr_mif->reg_rgb_mode << 8) |
        (wr_mif->reg_hconv_mode << 10) |
        (wr_mif->reg_vconv_mode << 12) |
        (0 << 16) |
        (0 << 20));
    sc_reg_wr(ISP_SCWR_MIF_CTRL2, 0);
    sc_reg_wr(ISP_SCWR_MIF_CTRL3,
        wr_mif->reg_hsizem1 |
        (wr_mif->reg_vsizem1 << 16));
    sc_reg_wr(ISP_SCWR_MIF_CTRL4,
        wr_mif->reg_start_x |
        (wr_mif->reg_start_y << 16));
    sc_reg_wr(ISP_SCWR_MIF_CTRL5,
        wr_mif->reg_end_x |
        (wr_mif->reg_end_y << 16));
    sc_reg_wr(ISP_SCWR_MIF_CTRL6,
        (wr_mif->reg_canvas_strb_luma & 0xffff) |
        (wr_mif->reg_canvas_strb_chroma << 16));
    sc_reg_wr(ISP_SCWR_MIF_CTRL7,
        wr_mif->reg_canvas_strb_r);
    sc_reg_wr(ISP_SCWR_MIF_CTRL11,
        wr_mif->reg_canvas_baddr_luma_other);
    sc_reg_wr(ISP_SCWR_MIF_CTRL12,
        wr_mif->reg_canvas_baddr_chroma_other);
    sc_reg_wr(ISP_SCWR_MIF_CTRL13,
        wr_mif->reg_canvas_baddr_r_other);
    sc_reg_wr(ISP_SCWR_MIF_CTRL8,
        wr_mif->reg_canvas_baddr_luma);
    sc_reg_wr(ISP_SCWR_MIF_CTRL9,
        wr_mif->reg_canvas_baddr_chroma);
    sc_reg_wr(ISP_SCWR_MIF_CTRL10,
        wr_mif->reg_canvas_baddr_r);
}

static void isp_mif_addr(ISP_MIF_t *wr_mif)
{
    sc_reg_wr(ISP_SCWR_MIF_CTRL6,
        (wr_mif->reg_canvas_strb_luma & 0xffff) |
        (wr_mif->reg_canvas_strb_chroma << 16));
    sc_reg_wr(ISP_SCWR_MIF_CTRL7,
        wr_mif->reg_canvas_strb_r);
    sc_reg_wr(ISP_SCWR_MIF_CTRL11,
        wr_mif->reg_canvas_baddr_luma_other);
    sc_reg_wr(ISP_SCWR_MIF_CTRL12,
        wr_mif->reg_canvas_baddr_chroma_other);
    sc_reg_wr(ISP_SCWR_MIF_CTRL13,
        wr_mif->reg_canvas_baddr_r_other);
    sc_reg_wr(ISP_SCWR_MIF_CTRL8,
        wr_mif->reg_canvas_baddr_luma);
    sc_reg_wr(ISP_SCWR_MIF_CTRL9,
        wr_mif->reg_canvas_baddr_chroma);
    sc_reg_wr(ISP_SCWR_MIF_CTRL10,
        wr_mif->reg_canvas_baddr_r);
}

static void enable_isp_scale_new (
    int    initial_en,
    int    ir_source,
    int    dbg_mode,
    int    clip_mode,
    int    clip_x_st,
    int    clip_x_ed,
    int    clip_y_st,
    int    clip_y_ed,
    int    src_w,      //in_w
    int    src_h,      //in_h
    int    wr_w,      //out_w
    int    wr_h,       //out_h
    int    mux_sel,
    int    sc_en,
    int    mtx_mode,     //1:rgb->yuv,2:yuv->rgb,0:bypass
    /*uint32_t    baddr,*/
    ISP_MIF_t *wr_mif
){
    //int tmp;
    uint32_t reg_data;
    u32 val = 0;

    int clip_w = clip_mode ? clip_x_ed - clip_x_st + 1 : src_w;
    int clip_h = clip_mode ? clip_y_ed - clip_y_st + 1 : src_h;
    int  sco_w = sc_en ? wr_w : clip_w;
    int  sco_h = sc_en ? wr_h : clip_h;

    wr_mif->reg_hsizem1  = sco_w-1;
    wr_mif->reg_vsizem1  = sco_h-1;
    wr_mif->reg_start_x = 0;
    wr_mif->reg_start_y = 0;
    wr_mif->reg_end_x  = sco_w-1;
    wr_mif->reg_end_y  = sco_h-1;
    if ( stop_flag ) isp_mif_setting(wr_mif);

    sc_reg_wr(ISP_SCWR_GCLK_CTRL, (src_w<<16));
    if (clip_mode) {
        sc_reg_wr(ISP_SCWR_CLIP_CTRL1,(1<<31) | (clip_x_ed<<16) | clip_x_st);
        sc_reg_wr(ISP_SCWR_CLIP_CTRL2, (clip_y_ed<<16) | clip_y_st);
    } else
        sc_reg_wr(ISP_SCWR_CLIP_CTRL1, 0x7FFF0000);

    if ( stop_flag ) {
        if (dbg_mode) {
            sc_reg_wr(ISP_SCWR_TOP_GEN, (1<<9) | dbg_mode);
            sc_reg_wr(ISP_SCWR_MIF_CTRL14, 0x70010101);
        } else
            sc_reg_wr(ISP_SCWR_TOP_GEN, 0xD1);
    }

    if (sc_en)
        isp_sc_setting(clip_w,clip_h,wr_w,wr_h);
    else
        sc_reg_wr(ISP_SC_MISC, 0x18404);

    if ( stop_flag ) {
        sc_reg_rd(ISP_SCWR_TOP_CTRL, &reg_data);
        sc_reg_wr(ISP_SCWR_TOP_CTRL,(reg_data & 0xff0bfffc) |
                                    ((mux_sel & 0x7)<<20) |
                                    (ir_source << 18));

        sc_reg_wr(ISP_SCWR_SYNC_DELAY, 0x4020000);

        isp_mtx_setting(mtx_mode);
    }

    sc_reg_wr(ISP_SCWR_SC_CTRL1, ((clip_w & 0xfff) << 16) | (clip_h & 0xfff));

    if (initial_en && stop_flag) {
        sc_reg_rd(ISP_SCWR_TOP_DBG0, &val);
        if (val & (1 << 6))
            last_end_frame = 1;
        else
            last_end_frame = 0;

        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 19, 1);
        if ( dbg_mode != 2 ) sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 5, 13, 3);
    }

    if (!clip_mode)
        pr_info(" finished isp scale setting \n");
}

static void init_sc_mif_setting(ISP_MIF_t *mif_frame)
{
    u32 plane_size, frame_size;
    unsigned long flags;
    tframe_t *buf = NULL;
    int retval;

    if (!mif_frame)
        return;

    spin_lock_irqsave( &g_sc->sc_lock, flags );
    if (kfifo_len(&g_sc->sc_fifo_in) > 0) {
        retval = kfifo_out(&g_sc->sc_fifo_in, &buf, sizeof(tframe_t*));
        if (retval == sizeof(tframe_t*)) {
            pre_frame[0] = buf;
        } else {
            pr_info("%d, fifo out failed.\n", __LINE__);
        }
    } else {
        pr_info("%d, sc fifo is empty .\n", __LINE__);
    }
    spin_unlock_irqrestore( &g_sc->sc_lock, flags );

    if (g_sc->info.out_fmt == NV12_GREY) {
        ch_mode = 1;
    } else {
        ch_mode = 0;
    }

    memset(mif_frame, 0, sizeof(ISP_MIF_t));
    plane_size = g_sc->info.out_w * g_sc->info.out_h;
    mif_frame->reg_little_endian = 1;
    if ((g_sc->info.in_fmt == RGB24) ||
        (g_sc->info.in_fmt == AYUV)) {
        if ((g_sc->info.out_fmt == RGB24) ||
            (g_sc->info.out_fmt == AYUV)) {
            mif_frame->reg_rgb_mode = 1;
        } else if (g_sc->info.out_fmt == NV12_YUV) {
            mif_frame->reg_rgb_mode = 2;
        } else if (g_sc->info.out_fmt == NV12_YVU) {
            mif_frame->reg_rgb_mode = 2;
        } else if (g_sc->info.out_fmt == UYVY) {
            mif_frame->reg_rgb_mode = 0;
        } else if (g_sc->info.out_fmt == NV12_GREY) {
            mif_frame->reg_rgb_mode = 2;
        } else if (g_sc->info.out_fmt == RAW16) {
            mif_frame->reg_rgb_mode = 0;
        } else if (g_sc->info.out_fmt == RAW_YUY2)
            mif_frame->reg_rgb_mode = 0;
    } else if ((g_sc->info.in_fmt == NV12_YUV) ||
            (g_sc->info.in_fmt == NV12_GREY)) {
        if (g_sc->info.out_fmt == NV12_GREY) {
            mif_frame->reg_rgb_mode = 2;
        }
    }

    mif_frame->reg_bit10_mode = 0;
    mif_frame->reg_words_lim = 4;
    mif_frame->reg_burst_lim = 3;
    mif_frame->reg_hconv_mode = 2;
    mif_frame->reg_vconv_mode = 0;
    mif_frame->reg_pingpong_en = 1;
    mif_frame->reg_start_x = 0;
    mif_frame->reg_start_y = 0;
    mif_frame->reg_end_x = g_sc->info.out_w -1;
    mif_frame->reg_end_y = g_sc->info.out_h - 1;
    mif_frame->reg_hsizem1 = g_sc->info.out_w - 1;
    mif_frame->reg_vsizem1 = g_sc->info.out_h - 1;
    if (ch_mode == 1) {
        mif_frame->reg_only_1ch = 1;
        mif_frame->reg_enable_3ch = 0;
        frame_size = plane_size;
        mif_frame->reg_canvas_strb_luma =
            g_sc->info.out_w;
        mif_frame->reg_canvas_strb_chroma = 0;
        mif_frame->reg_canvas_strb_r = 0;
        mif_frame->reg_hconv_mode = 2;
        mif_frame->reg_vconv_mode = 2;
        mif_frame->reg_rgb_mode = 2;
    } else if (ch_mode == 3) {
        mif_frame->reg_only_1ch = 0;
        mif_frame->reg_enable_3ch = 1;
        frame_size = plane_size * 3;
        mif_frame->reg_canvas_strb_luma =
            g_sc->info.out_w;
        mif_frame->reg_canvas_strb_chroma =
            g_sc->info.out_w;
        mif_frame->reg_canvas_strb_r =
            g_sc->info.out_w;
        mif_frame->reg_hconv_mode = 2;
        mif_frame->reg_vconv_mode = 2;
        mif_frame->reg_rgb_mode = 2;
    } else {
        mif_frame->reg_only_1ch = 0;
        mif_frame->reg_enable_3ch = 0;
        if (mif_frame->reg_rgb_mode == 0) {
            frame_size = plane_size * 2;
            mif_frame->reg_canvas_strb_luma =
                g_sc->info.out_w * 2;
            mif_frame->reg_canvas_strb_chroma = 0;
            mif_frame->reg_canvas_strb_r = 0;
        } else if (mif_frame->reg_rgb_mode == 1) {
            frame_size = plane_size * 3;
            mif_frame->reg_canvas_strb_luma =
                g_sc->info.out_w * 3;
            mif_frame->reg_canvas_strb_chroma = 0;
            mif_frame->reg_canvas_strb_r = 0;
        } else if (mif_frame->reg_rgb_mode == 2) {
            frame_size = plane_size * 3 / 2;
            mif_frame->reg_canvas_strb_luma =
                g_sc->info.out_w;
            mif_frame->reg_canvas_strb_chroma =
                g_sc->info.out_w;
            mif_frame->reg_canvas_strb_r = 0;
        } else {
            frame_size = plane_size * 3;
            mif_frame->reg_canvas_strb_luma =
                g_sc->info.out_w * 3;
            mif_frame->reg_canvas_strb_chroma = 0;
            mif_frame->reg_canvas_strb_r = 0;
        }
    }
    mif_frame->reg_canvas_baddr_luma =
        buf->primary.address;
    mif_frame->reg_canvas_baddr_luma_other =
        mif_frame->reg_canvas_baddr_luma;

    if (mif_frame->reg_canvas_strb_chroma) {
        mif_frame->reg_canvas_baddr_chroma =
            buf->secondary.address;
        mif_frame->reg_canvas_baddr_chroma_other =
            mif_frame->reg_canvas_baddr_chroma;
    }

    if (mif_frame->reg_canvas_strb_r) {
        mif_frame->reg_canvas_baddr_r =
            buf->secondary.address;
        mif_frame->reg_canvas_baddr_r_other =
            mif_frame->reg_canvas_baddr_r;
    }

    if (!mif_frame->reg_pingpong_en) {
        mif_frame->reg_canvas_baddr_luma_other = 0;
        mif_frame->reg_canvas_baddr_chroma_other = 0;
        mif_frame->reg_canvas_baddr_r_other = 0;
    }

    pr_info("init_sc_mif_setting: %dx%d -> %dx%d, %x-%x-%x-%x-%x-%x, stride: %x-%x-%x\n",
        g_sc->info.src_w, g_sc->info.src_h,
        g_sc->info.out_w, g_sc->info.out_h,
        mif_frame->reg_canvas_baddr_luma,
        mif_frame->reg_canvas_baddr_chroma,
        mif_frame->reg_canvas_baddr_r,
        mif_frame->reg_canvas_baddr_luma_other,
        mif_frame->reg_canvas_baddr_chroma_other,
        mif_frame->reg_canvas_baddr_r_other,
        mif_frame->reg_canvas_strb_luma,
        mif_frame->reg_canvas_strb_chroma,
        mif_frame->reg_canvas_strb_r);
}

static int write_to_file(char *buf, int size)
{
    int ret = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t pos = 0;
    int nwrite = 0;

    /* change to KERNEL_DS address limit */
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    /* open file to write */
    fp = filp_open("/media/sc_img.raw", O_WRONLY|O_CREAT, 0640);
    if (!fp) {
       printk("%s: open file error\n", __FUNCTION__);
       ret = -1;
       goto exit;
    }

    /* Write buf to file */
    nwrite=vfs_write(fp, buf, size, &pos);

    if (fp) {
        filp_close(fp, NULL);
    }
exit:
    set_fs(old_fs);
    return ret;
}

static ssize_t sc2_frame_read(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    char buf1[50];

    pr_info("sc-read.\n");
    //buf = (char *)sc_cma_mem;
    write_to_file(buf, g_sc->info.out_w * g_sc->info.out_h * 3);
    return sprintf(buf1,"buffer_id:%d", buffer_id);
}

static ssize_t sc2_frame_write(struct device *dev,
    struct device_attribute *attr, char const *buf, size_t size)
{
    unsigned long write_flag = 0;
    int retval = 0;

    retval = kstrtoul(buf, 10, &write_flag);

    if (retval) {
        pr_err("Error to count strtoul\n");
        return retval;
    }
    return size;
}
static DEVICE_ATTR(sc2_frame, S_IRUGO | S_IWUSR, sc2_frame_read, sc2_frame_write);

#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
static void sc_do_tasklet( unsigned long data )
{
    tframe_t *f_buff;
    metadata_t metadata;
    memset(&metadata, 0, sizeof(metadata_t));

    while (kfifo_out(&sc_tasklet.sc_fifo_out, &f_buff, sizeof(tframe_t*))) {
        metadata.width = g_sc->info.out_w;
        metadata.height = g_sc->info.out_h;
        metadata.frame_id = frame_id;
        metadata.frame_number = frame_id;
        metadata.line_size = (((3 * g_sc->info.out_w) + 127) & (~127));
        frame_id++;
        g_sc->callback(g_sc->ctx, f_buff, &metadata );
    }
}
#endif

static void sc_config_next_buffer(tframe_t* f_buf, int ping)
{
    if (ping) {
        isp_frame.reg_canvas_baddr_luma = f_buf->primary.address;
        isp_frame.reg_canvas_baddr_chroma = f_buf->secondary.address;
        isp_frame.reg_canvas_baddr_r = f_buf->secondary.address;
    } else {
        isp_frame.reg_canvas_baddr_luma_other = f_buf->primary.address;
        isp_frame.reg_canvas_baddr_chroma_other = f_buf->secondary.address;
        isp_frame.reg_canvas_baddr_r_other = f_buf->secondary.address;
    }

    isp_mif_addr(&isp_frame);
}

static void am_sc_swap_buf(int check_last, u32 flag_ready)
{
    last_end_frame = flag_ready;

    if (pre_frame[frame_delay - 1] && pre_frame[frame_delay - 2]) {
        tframe_t* temp_frame = pre_frame[frame_delay - 1];
        pre_frame[frame_delay - 1] = pre_frame[frame_delay - 2];
        pre_frame[frame_delay - 2] = temp_frame;
    }

    if (check_last) {
        if (pre_frame[frame_delay]) {
            kfifo_in(&g_sc->sc_fifo_in, &pre_frame[frame_delay], sizeof(tframe_t*));
            pre_frame[frame_delay] = NULL;
        }
    }
}

static irqreturn_t isp_sc_isr(int irq, void *data)
{
    if (start_delay_cnt < start_delay_th) {
        start_delay_cnt++;
    } else {
        /* maybe need drop one more frame */
        if (start_delay_cnt == start_delay_th) {
            sc1_isr_count = 0;
            sc2_isr_count = 0;
            pr_info("wrmif start: %x, %x, %x\n", sc_get_reg(ISP_SCWR_TOP_DBG1), sc_get_reg(ISP_SCWR_TOP_DBG2), sc_get_reg(ISP_SCWR_TOP_CTRL));
            sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 1, 1);
            sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 0, 1);
            start_delay_cnt++;
        } else {
            u32 flag = 0;
            u32 flag_ready = 0;
            tframe_t *f_buf = NULL;
            unsigned long flags;
            int retval;

            sc2_isr_count++;

            sc_reg_rd(ISP_SCWR_TOP_DBG0, &flag);

            flag_ready = (flag & (1 << 6)) ? 1 : 0;
            if (flag_ready == last_end_frame) {
                pr_info("%d, sc last fifo no ready.\n", __LINE__);
                am_sc_swap_buf(1, flag_ready);
                return IRQ_HANDLED;
            }

            if (g_sc->crop_refresh_flag) {
                am_sc2_hw_init(0, g_sc->crop_refresh_flag);
                g_sc->crop_refresh_flag = 0;

            }

            uint32_t d_fps = g_sc->info.c_fps - g_sc->info.t_fps;
            if ((g_sc->info.c_fps > g_sc->info.t_fps) &&
                (sc1_isr_count * d_fps *1000L / g_sc->info.c_fps/1000L) != ((sc1_isr_count - 1) * d_fps * 1000L / g_sc->info.c_fps/1000L) ) {
                am_sc_swap_buf(0, flag_ready);
                return IRQ_HANDLED;
            }

            spin_lock_irqsave( &g_sc->sc_lock, flags );
            if (kfifo_len(&g_sc->sc_fifo_in) > 0) {
                retval = kfifo_out(&g_sc->sc_fifo_in, &f_buf, sizeof(tframe_t*));
                if (retval != sizeof(tframe_t*))
                    pr_info("%d, fifo out failed.\n", __LINE__);
            } else {
                //pr_info("%d, sc fifo is empty.\n", __LINE__);
                frame_id++;
                am_sc_swap_buf(0, flag_ready);
                spin_unlock_irqrestore( &g_sc->sc_lock, flags );
                return IRQ_HANDLED;
            }
            spin_unlock_irqrestore( &g_sc->sc_lock, flags );

            sc_config_next_buffer(f_buf, (flag & (1 << 8)) ? 0 : 1);

            if (pre_frame[frame_delay]) {
#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
                if (!kfifo_is_full(&sc_tasklet.sc_fifo_out)) {
                    kfifo_in(&sc_tasklet.sc_fifo_out, &(pre_frame[frame_delay]), sizeof(tframe_t*));
                }
                tasklet_schedule(&sc_tasklet.tasklet_obj);
#endif
            } else {
                pr_info("%d, sc fifo is empty.\n", __LINE__);
            }

            pre_frame[2] = pre_frame[1];
            pre_frame[1] = pre_frame[0];
            pre_frame[0] = f_buf;
            last_end_frame = flag_ready;
            buffer_id ^= 1;
        }
    }
    return IRQ_HANDLED;
}

int am_sc2_parse_dt(struct device_node *node, int port)
{
    int rtn = -1;
    int irq = -1;
    struct resource rs;
    struct am_sc2 *t_sc = NULL;

    if (node == NULL) {
        pr_err("%s: Error input param\n", __func__);
        return -1;
    }

    rtn = of_device_is_compatible(node, AM_SC_NAME);
    if (rtn == 0) {
        pr_err("%s: Error match compatible\n", __func__);
        return -1;
    }

    t_sc = kzalloc(sizeof(*t_sc), GFP_KERNEL);
    if (t_sc == NULL) {
        pr_err("%s: Failed to alloc isp-sc2\n", __func__);
        return -1;
    }

    t_sc->of_node = node;

    rtn = of_address_to_resource(node, port, &rs);
    if (rtn != 0) {
        pr_err("%s:Error get isp-sc2 reg resource\n", __func__);
        goto reg_error;
    }

    pr_info("%s: rs idx info: name: %s\n", __func__, rs.name);
    if (strcmp(rs.name, "isp_sc2") == 0) {
        t_sc->reg = rs;
        t_sc->base_addr = ioremap_nocache(
            t_sc->reg.start, resource_size(&t_sc->reg));
    }

    irq = irq_of_parse_and_map(node, port);
    if (irq <= 0) {
        pr_err("%s:Error get isp-sc2 irq\n", __func__);
        goto irq_error;
    }

    t_sc->irq = irq;
    pr_info("%s:rs info: irq: %d, ds%d\n", __func__, t_sc->irq, port);

    t_sc->p_dev = of_find_device_by_node(node);

    device_create_file(&(t_sc->p_dev->dev), &dev_attr_sc2_frame);
    t_sc->port = port;
    g_sc = t_sc;

    g_sc->crop_refresh_flag = 0;

    spin_lock_init(&g_sc->sc_lock);

    if (!kfifo_initialized(&g_sc->sc_fifo_in)) {
        rtn = kfifo_alloc(&g_sc->sc_fifo_in, PAGE_SIZE, GFP_KERNEL);
        if (rtn) {
            pr_info("alloc sc fifo failed.\n");
            return rtn;
        }
    }
#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
    if (!kfifo_initialized(&sc_tasklet.sc_fifo_out)) {
        rtn = kfifo_alloc(&sc_tasklet.sc_fifo_out, PAGE_SIZE, GFP_KERNEL);
        if (rtn) {
            pr_info("alloc sc_tasklet fifo failed.\n");
            return rtn;
        }
    }
#endif

    return 0;
irq_error:
    iounmap(t_sc->base_addr);
    t_sc->base_addr = NULL;

reg_error:
    if (t_sc != NULL)
        kfree(t_sc);
    return -1;
}

void am_sc2_deinit_parse_dt(void)
{
    if (g_sc == NULL) {
        pr_err("Error g_sc is NULL\n");
        return;
    }
    kfifo_free(&g_sc->sc_fifo_in);
#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
    // kill tasklet
    kfifo_free(&sc_tasklet.sc_fifo_out);
#endif

    if (g_sc->p_dev != NULL)
        device_remove_file(&(g_sc->p_dev->dev), &dev_attr_sc2_frame);

    if (g_sc->base_addr != NULL) {
        iounmap(g_sc->base_addr);
        g_sc->base_addr = NULL;
    }

    kfree(g_sc);
    g_sc = NULL;
}

void am_sc2_api_dma_buffer(tframe_t * data, unsigned int index)
{
    unsigned long flags;
    if (temp_buf == NULL) return;
    spin_lock_irqsave(&g_sc->sc_lock, flags);
    tframe_t *buf = temp_buf + index;
    memcpy(buf, data, sizeof(tframe_t));
    if (!kfifo_is_full(&g_sc->sc_fifo_in)) {
        kfifo_in(&g_sc->sc_fifo_in, &buf, sizeof(tframe_t*));
    } else {
        pr_info("sc fifo is full .\n");
    }
    spin_unlock_irqrestore(&g_sc->sc_lock, flags);

}

uint32_t am_sc2_get_width(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }
    return g_sc->info.out_w;
}

void am_sc2_set_width(uint32_t src_w, uint32_t out_w)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    if (g_sc->info.src_w == 0)
        g_sc->info.src_w = src_w;
    g_sc->info.out_w = out_w;
}

void am_sc2_set_fps(uint32_t c_fps, uint32_t t_fps)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.c_fps = c_fps;
    g_sc->info.t_fps = t_fps == 0 ? g_sc->info.t_fps : t_fps;;
}

uint32_t am_sc2_get_fps(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return 0;
    }
    return g_sc->info.t_fps;
}

uint32_t am_sc2_get_startx(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return 0;
    }
    return g_sc->info.startx;
}

uint32_t am_sc2_get_starty(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return 0;
    }
    return g_sc->info.starty;
}

void am_sc2_set_startx(uint32_t startx)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.startx = startx;
}

void am_sc2_set_starty(uint32_t starty)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.starty = starty;
}

uint32_t am_sc2_get_crop_width(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return 0;
    }
    return g_sc->info.c_width;
}

uint32_t am_sc2_get_crop_height(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return 0;
    }
    return g_sc->info.c_height;
}

void am_sc2_set_crop_width(uint32_t c_width)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.c_width = c_width;
}

void am_sc2_set_crop_height(uint32_t c_height)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.c_height = c_height;
}

void am_sc2_set_crop_enable()
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->crop_refresh_flag = 1;

}
void am_sc2_set_src_width(uint32_t src_w)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.src_w = src_w;
}

void am_sc2_set_src_height(uint32_t src_h)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.src_h = src_h;
}

uint32_t am_sc2_get_height(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }
    return g_sc->info.out_h;
}

uint32_t am_sc2_get_output_format(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }
    return g_sc->info.out_fmt;
}


void am_sc2_set_height(uint32_t src_h, uint32_t out_h)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    if (g_sc->info.src_h == 0)
        g_sc->info.src_h = src_h;
    g_sc->info.out_h = out_h;
}

void am_sc2_set_input_format(uint32_t value)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.in_fmt = value;
}

void am_sc2_set_output_format(uint32_t value)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->info.out_fmt = value;
}

void am_sc2_set_buf_num(uint32_t num)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return;
    }
    g_sc->req_buf_num = num;
}

int am_sc2_set_callback(acamera_context_ptr_t p_ctx, buffer_callback_t sc2_callback)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }
    g_sc->callback = sc2_callback;
    g_sc->ctx = p_ctx;
    return 0;
}

int am_sc2_system_init(void)
{
    int ret = 0;

    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }

    if (g_sc->req_buf_num == 0)
        return 0;

    start_delay_cnt = 0;
    buffer_id = 0;

    pre_frame[0] = NULL;
    pre_frame[1] = NULL;
    pre_frame[2] = NULL;

    if (!temp_buf) {
        temp_buf = (tframe_t*)kmalloc( sizeof(tframe_t) * (g_sc->req_buf_num), GFP_KERNEL | __GFP_NOFAIL);
    }

    frame_id = 0;

    return ret;
}

int am_sc2_hw_init(int is_print, int clip_mode)
{
    uint32_t flag = 0;
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }
    if (g_sc->req_buf_num == 0)
        return 0;

    if ((stop_flag == false) && !clip_mode) {
        pr_info("wrmif no stop\n");
        return 0;
    }

    sc_reg_rd(ISP_SCWR_TOP_CTRL, &flag);
    if ((flag & 0x01) && !clip_mode) {
        pr_info("wrmif Bit0 no stop\n");
        return 0;
    }

    if (sc_get_reg(ISP_SCWR_TOP_DBG1) != sc_get_reg(ISP_SCWR_TOP_DBG2)) {
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 25, 1);
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 0, 25, 1);
        pr_info("wrmif DBG force stop: %x - %x\n", sc_get_reg(ISP_SCWR_TOP_DBG1), sc_get_reg(ISP_SCWR_TOP_DBG2));
    }

    if (!clip_mode)
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 25, 1);

    if (sc_get_reg(ISP_SCWR_TOP_DBG1) != sc_get_reg(ISP_SCWR_TOP_DBG2)) {
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 25, 1);
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 0, 25, 1);
        pr_info("wrmif DBG force skip: %x - %x\n", sc_get_reg(ISP_SCWR_TOP_DBG1), sc_get_reg(ISP_SCWR_TOP_DBG2));
        return 0;
    }

    int mtx_mode = 0;
    int dbg_mode = 0;
    int mux_sel = 0;
    if ( stop_flag ) init_sc_mif_setting(&isp_frame);
    buffer_id = 0;
    if (g_sc->info.in_fmt == RGB24) {
        if (g_sc->info.out_fmt == RGB24) {
            mtx_mode = 0;
        } else if ((g_sc->info.out_fmt == NV12_YUV) ||
                (g_sc->info.out_fmt == NV12_YVU) ||
                (g_sc->info.out_fmt == UYVY) ||
                (g_sc->info.out_fmt == AYUV)) {
            mtx_mode = 1;
        } else if (g_sc->info.out_fmt == NV12_GREY) {
            mtx_mode = 1;
        } else if (g_sc->info.out_fmt == RAW16) {
            mtx_mode = 0;
            dbg_mode = 2;
            mux_sel = 4;
        } else if (g_sc->info.out_fmt == RAW_YUY2) {
            mtx_mode = 0;
            dbg_mode = 0;
            mux_sel = 4;
        }
    } else if (g_sc->info.in_fmt == AYUV) {
        if ((g_sc->info.out_fmt == NV12_YUV) ||
            (g_sc->info.out_fmt == NV12_YVU) ||
            (g_sc->info.out_fmt == UYVY) ||
            (g_sc->info.out_fmt == AYUV)) {
            mtx_mode = 0;
        } else if (g_sc->info.out_fmt == RGB24) {
            mtx_mode = 2;
        } else if (g_sc->info.out_fmt == NV12_GREY) {
            mtx_mode = 0;
        }
    } else if ((g_sc->info.in_fmt == NV12_YUV) ||
            (g_sc->info.in_fmt == NV12_GREY)) {
        if (g_sc->info.out_fmt == NV12_GREY) {
            mtx_mode = 0;
        }
    } else {
        pr_err("unSupported format");
    }

    if ( g_sc->info.c_width == 0 )
        g_sc->info.c_width = g_sc->info.src_w;

    if ( g_sc->info.c_height == 0 )
        g_sc->info.c_height = g_sc->info.src_h;

    if ( ( g_sc->info.c_width != g_sc->info.src_w ) || ( g_sc->info.c_height != g_sc->info.src_h ) )
        g_sc->info.clip_sc_mode |= CLIP_MODE;
    else
        g_sc->info.clip_sc_mode &= ~CLIP_MODE;

    if ( !clip_mode )
        g_sc->info.clip_sc_mode &= ~CLIP_MODE;

    if ( g_sc->port < 3 ) {
        if ( ( g_sc->info.c_width != g_sc->info.out_w ) || ( g_sc->info.c_height != g_sc->info.out_h ) )
            g_sc->info.clip_sc_mode |= SC_MODE;
    }

    if ( is_print ) pr_info("src_w = %d, src_h = %d, out_w = %d, out_h = %d, crop_w = %d, crop_h = %d, clip = %d, in_fmt:%d, out_fmt:%d\n",
            g_sc->info.src_w, g_sc->info.src_h, g_sc->info.out_w, g_sc->info.out_h, g_sc->info.c_width,
            g_sc->info.c_height, g_sc->info.clip_sc_mode, g_sc->info.in_fmt, g_sc->info.out_fmt);

    enable_isp_scale_new(1, 0, dbg_mode, g_sc->info.clip_sc_mode & CLIP_MODE, g_sc->info.startx,
        g_sc->info.startx + g_sc->info.c_width - 1, g_sc->info.starty,  g_sc->info.starty + g_sc->info.c_height - 1,
        g_sc->info.src_w, g_sc->info.src_h, g_sc->info.out_w, g_sc->info.out_h, mux_sel, g_sc->info.clip_sc_mode & SC_MODE, mtx_mode,
        &isp_frame);

    sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 0, 25, 1);

    return 0;
}

int am_sc2_start(void)
{
    int ret = 0;
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }
    if (stop_flag == false) return 0;
#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
    tasklet_init( &sc_tasklet.tasklet_obj, sc_do_tasklet, (unsigned long)&sc_tasklet );
#endif
    ret = request_irq(g_sc->irq, isp_sc_isr, IRQF_SHARED | IRQF_TRIGGER_RISING,
        "isp-sc2-irq", (void *)g_sc);
    pr_info("%s irq = %d, ret = %d\n",__func__,g_sc->irq, ret);

    /* switch int to sync reset for start and delay frame */
    sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 1, 3, 1);
    stop_flag = false;
    frame_id = 0;

    return 0;
}

int am_sc2_reset(void)
{
    return 0;
}

int am_sc2_stop(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }

    if (!stop_flag) {
        stop_flag = true;
        start_delay_cnt = 0;
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 0, 0, 1);
        sc_wr_reg_bits(ISP_SCWR_TOP_CTRL, 0, 3, 1);
        mdelay(66);
        pr_info("wrmif stop: %x, %x\n", sc_get_reg(ISP_SCWR_TOP_DBG1), sc_get_reg(ISP_SCWR_TOP_DBG2));

        pre_frame[0] = NULL;
        pre_frame[1] = NULL;
        pre_frame[2] = NULL;

        if (temp_buf != NULL) {
            kfree(temp_buf);
            temp_buf = NULL;
        }
        free_irq(g_sc->irq, (void *)g_sc);
#ifdef ENABLE_SC_BOTTOM_HALF_TASKLET
        tasklet_kill( &sc_tasklet.tasklet_obj );
        kfifo_reset(&sc_tasklet.sc_fifo_out);
#endif
        kfifo_reset(&g_sc->sc_fifo_in);
        frame_id = 0;

        g_sc->info.startx = 0;
        g_sc->info.starty = 0;
        g_sc->info.c_width = 0;
        g_sc->info.c_height = 0;
        g_sc->info.src_w = 0;
        g_sc->info.src_h = 0;
        g_sc->info.clip_sc_mode = 0;

    }

    return 0;
}

int am_sc2_system_deinit(void)
{
    if (!g_sc) {
        pr_info("%d, g_sc is NULL.\n", __LINE__);
        return -1;
    }

    iounmap(g_sc->base_addr);
    g_sc->base_addr = NULL;

    kfree(g_sc);
    g_sc = NULL;
    return 0;
}


int am_sc2_hw_deinit(void)
{
    return 0;
}

