#include <linux/io.h>
#include "axi_disp_ctrl.h"
#include "litex_drp.h"

static void axi_dispctrl_write_reg(void *base, unsigned int offset, unsigned int val)
{
	writel(val, base + offset);
}

static unsigned int axi_dispctrl_read_reg(void *base, unsigned int offset)
{
	volatile unsigned int* reg;
	reg = (unsigned int*)((unsigned int*)base + offset);
	return *reg;
}

void print_regs(void *base)
{
	printk("DISPCTRL_FSYNC_REG: 0x%x", axi_dispctrl_read_reg(base, DISPCTRL_FSYNC_REG));
	printk("DISPCTRL_CLK0_L_REG: 0x%x", axi_dispctrl_read_reg(base, DISPCTRL_CLK0_L_REG));
	printk("DISPCTRL_CLK1_L_REG: 0x%x", axi_dispctrl_read_reg(base, DISPCTRL_CLK1_L_REG));
	printk("DISPCTRL_FB_L_REG: 0x%x", axi_dispctrl_read_reg(base, DISPCTRL_FB_L_REG));
	printk("DISPCTRL_FB_H_CLK_H_REG: 0x%x", axi_dispctrl_read_reg(base, DISPCTRL_FB_H_CLK_H_REG));
	printk("DISPCTRL_DIV_REG: 0x%x", axi_dispctrl_read_reg(base, DISPCTRL_DIV_REG));
}

static void litex_drp_wait_drdy(struct axi_dispctrl_private *priv)
{
    int cnt = 0;
    while(!readl(priv->drp_base + DRP_DRDY) && (cnt < 1000))
        cnt++;

    if (cnt == 1000)
        printk(KERN_ERR "DRP DRDY timed out\n");
}

static unsigned int litex_drp_read_reg(struct axi_dispctrl_private *priv, unsigned int offset)
{
    writel(offset, priv->drp_base + DRP_ADR);
    writel(1, priv->drp_base + DRP_READ);
    litex_drp_wait_drdy(priv);
    return readl(priv->drp_base + DRP_DAT_R);
}

static void litex_drp_write_reg(struct axi_dispctrl_private *priv, unsigned int mask, unsigned int offset, unsigned int val)
{
    unsigned int old_val = litex_drp_read_reg(priv, offset);
    val = (old_val & mask) | val;
    writel(offset, priv->drp_base + DRP_ADR);
    writel(val, priv->drp_base + DRP_DAT_W);
    writel(1, priv->drp_base + DRP_WRITE);
    litex_drp_wait_drdy(priv);
}

void litex_drp_print_regs(struct axi_dispctrl_private *priv)
{
    printk(KERN_ERR "CLK0L 0x%04x\n", litex_drp_read_reg(priv, DRP_CLK0L));
    printk(KERN_ERR "CLK0H 0x%04x\n", litex_drp_read_reg(priv, DRP_CLK0H));
    printk(KERN_ERR "CLK1L 0x%04x\n", litex_drp_read_reg(priv, DRP_CLK1L));
    printk(KERN_ERR "CLK1H 0x%04x\n", litex_drp_read_reg(priv, DRP_CLK1H));
    printk(KERN_ERR "CLKFBL 0x%04x\n", litex_drp_read_reg(priv, DRP_CLKFBL));
    printk(KERN_ERR "CLKFBH 0x%04x\n", litex_drp_read_reg(priv, DRP_CLKFBH));
    printk(KERN_ERR "CLKDIV 0x%04x\n", litex_drp_read_reg(priv, DRP_CLKDIV));
}

#define R1920x1080

void litex_drp_setup_clocks(struct axi_dispctrl_private *priv)
{
    // we need to set this when using DRP
    litex_drp_write_reg(priv, DRP_PWR_MASK, DRP_PWR, 0xffff);

#ifdef R800x600
    litex_drp_write_reg(priv, DRP_CLK0L_MASK, DRP_CLK0L, 10 << DRP_HCOUNT | 10 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLK0H_MASK, DRP_CLK0H, 0); // divide VCO clock by 20 - 40MHz
    litex_drp_write_reg(priv, DRP_CLK1L_MASK, DRP_CLK1L, 2 << DRP_HCOUNT | 2 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLK1H_MASK, DRP_CLK1H, 0); // divide VCO clock by 4 - 200MHz
    litex_drp_write_reg(priv, DRP_CLKFBL_MASK, DRP_CLKFBL, 20 << DRP_HCOUNT | 20 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLKFBH_MASK, DRP_CLKFBH, 0); // multiply input clock by 40
#else
#  ifdef R1920x1080
    litex_drp_write_reg(priv, DRP_CLK0L_MASK, DRP_CLK0L, 2 << DRP_HCOUNT | 3 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLK0H_MASK, DRP_CLK0H, DRP_CLK_EDGE); // divide VCO clock by 5 - 148.5MHz
    litex_drp_write_reg(priv, DRP_CLK1L_MASK, DRP_CLK1L, 1 << DRP_HCOUNT | 1 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLK1H_MASK, DRP_CLK1H, DRP_CLK_NO_COUNT); // use VCO clock - 742.5MHz
#  endif
#  ifdef R1280x720
    litex_drp_write_reg(priv, DRP_CLK0L_MASK, DRP_CLK0L, 5 << DRP_HCOUNT | 5 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLK0H_MASK, DRP_CLK0H, 0); // divide VCO clock by 10 - 74.25MHz
    litex_drp_write_reg(priv, DRP_CLK1L_MASK, DRP_CLK1L, 1 << DRP_HCOUNT | 1 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLK1H_MASK, DRP_CLK1H, 0); // divide VCO clock by 2 - 371.25MHz
#  endif
	// We need to subtract 1 from desired count values when using fractional dividers
	// https://forums.xilinx.com/t5/7-Series-FPGAs/Problems-Simulating-MMCM-DRP/m-p/562280#M7953
    litex_drp_write_reg(priv, DRP_CLKFBL_MASK, DRP_CLKFBL, 17 << DRP_HCOUNT | 18 << DRP_LCOUNT);
    litex_drp_write_reg(priv, DRP_CLKFBH_MASK, DRP_CLKFBH, 1 << DRP_FRAC | DRP_CLK_FRAC_EN | DRP_CLK_EDGE); // multiply input clock by 37.125
#endif
    litex_drp_write_reg(priv, DRP_CLKDIV_MASK, DRP_CLKDIV, DRP_CLKDIV_EDGE | 2 << DRP_HCOUNT | 3 << DRP_LCOUNT); // divide input clock by 5
    // 1080P lock and filter settings should be good enough
    litex_drp_write_reg(priv, DRP_LOCK1_MASK, DRP_LOCK1, 0x00fa);
    litex_drp_write_reg(priv, DRP_LOCK2_MASK, DRP_LOCK2, 0x7c01);
    litex_drp_write_reg(priv, DRP_LOCK3_MASK, DRP_LOCK3, 0x7fe9);
    litex_drp_write_reg(priv, DRP_FLT1_MASK, DRP_FLT1, 0x0100);
    litex_drp_write_reg(priv, DRP_FLT2_MASK, DRP_FLT2, 0x8090);
}

void axi_dispctrl_init(struct axi_dispctrl_private *priv)
{
	printk("%s", __func__);

    litex_drp_setup_clocks(priv);

#ifdef R800x600
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO0_REG, 800 << 16 | 600);//Not used
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO1_REG, 40 << 16 | 168);//HSync
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO2_REG, 1 << 16 | 1055);//HSync-polarity, HMax
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO3_REG, 1 << 16 | 5);//VSync
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO4_REG, 1 << 16 | 627);//VSync-polarity, VMax
	axi_dispctrl_write_reg(priv->base, DISPCTRL_PREAM_REG, 246 << 16 | 254);//Video preamble
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VGUARD_REG, 254 << 16 | 256);//Video guard band
	axi_dispctrl_write_reg(priv->base, DISPCTRL_HDATAEN_REG, 1 << 31 | 256 << 16 | 1056);//DE polarity, Video active horizontal area
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VDATAEN_REG, 28 << 16 | 628);//Video active vertical area
	axi_dispctrl_write_reg(priv->base, DISPCTRL_FSYNC_REG, 3);//FSync will be pulsed on this line
#endif
#ifdef R1920x1080
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO0_REG, 1920 << 16 | 1080);//Not used
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO1_REG, 0 << 16 | 2100);//HSync
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO2_REG, 1 << 16 | 2199);//HSync-polarity, HMax
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO3_REG, 4 << 16 | 9);//VSync
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO4_REG, 1 << 16 | 1124);//VSync-polarity, VMax
	axi_dispctrl_write_reg(priv->base, DISPCTRL_PREAM_REG, 252 << 16 | 260);//Video preamble
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VGUARD_REG, 260 << 16 | 262);//Video guard band
	axi_dispctrl_write_reg(priv->base, DISPCTRL_HDATAEN_REG, 1 << 31 | 262 << 16 | 2182);//DE polarity, Video active horizontal area
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VDATAEN_REG, 45 << 16 | 1125);//Video active vertical area
	axi_dispctrl_write_reg(priv->base, DISPCTRL_FSYNC_REG, 4);//FSync will be pulsed on this line
#endif
#ifdef R1280x720
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO0_REG, 1280 << 16 | 720);//Not used
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO1_REG, 110 << 16 | 150);//HSync
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO2_REG, 1 << 16 | 1649);//HSync-polarity, HMax
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO3_REG, 5 << 16 | 10);//VSync
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VIDEO4_REG, 1 << 16 | 749);//VSync-polarity, VMax
	axi_dispctrl_write_reg(priv->base, DISPCTRL_PREAM_REG, 360 << 16 | 368);//Video preamble
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VGUARD_REG, 368 << 16 | 370);//Video guard band
	axi_dispctrl_write_reg(priv->base, DISPCTRL_HDATAEN_REG, 1 << 31 | 370 << 16 | 1650);//DE polarity, Video active horizontal area
	axi_dispctrl_write_reg(priv->base, DISPCTRL_VDATAEN_REG, 30 << 16 | 750);//Video active vertical area
	axi_dispctrl_write_reg(priv->base, DISPCTRL_FSYNC_REG, 3);//FSync will be pulsed on this line
#endif
}

void axi_dispctrl_enable(struct axi_dispctrl_private *priv)
{
	printk("%s", __func__);
	unsigned int reg;
	reg = axi_dispctrl_read_reg(priv->base, DISPCTRL_CTRL_REG);
	reg |= DISPCTRL_ENABLE;
	axi_dispctrl_write_reg(priv->base, DISPCTRL_CTRL_REG, reg);
}

void axi_dispctrl_disable(struct axi_dispctrl_private *priv)
{
	printk("%s", __func__);
	unsigned int reg;
	reg = axi_dispctrl_read_reg(priv->base, DISPCTRL_CTRL_REG);
	reg &= ~(DISPCTRL_ENABLE);
	axi_dispctrl_write_reg(priv->base, DISPCTRL_CTRL_REG, reg);
}
