#ifndef AXI_DISP_CTRL_H_
#define AXI_DISP_CTRL_H_

#include "axi_disp_drv.h"

#define DISPCTRL_CTRL_REG 0x0
#define DISPCTRL_STATUS_REG 0x4
#define DISPCTRL_VIDEO0_REG 0x8
#define DISPCTRL_VIDEO1_REG 0xC
#define DISPCTRL_VIDEO2_REG 0x10
#define DISPCTRL_VIDEO3_REG 0x14
#define DISPCTRL_VIDEO4_REG 0x18
#define DISPCTRL_CLK0_L_REG 0x1C
#define DISPCTRL_FB_L_REG 0x20
#define DISPCTRL_FB_H_CLK_H_REG 0x24
#define DISPCTRL_DIV_REG 0x28
#define DISPCTRL_LOCK_L_REG 0x2C
#define DISPCTRL_FLTR_LOCK_H_REG 0x30
#define DISPCTRL_CLK1_L_REG 0x34
#define DISPCTRL_PREAM_REG 0x38
#define DISPCTRL_VGUARD_REG 0x3C
#define DISPCTRL_HDATAEN_REG 0x40
#define DISPCTRL_VDATAEN_REG 0x44
#define DISPCTRL_FSYNC_REG 0x48

#define DISPCTRL_ENABLE (1<<0)

#define DISPCTRL_CLK_BYPASS_COUNT (1<<22)
#define DISPCTRL_CLK_HCOUNT_EDGE (1<<23)
#define DISPCTRL_FRAC_EN (1<<27)
#define DISPCTRL_DIV_BYPASS_COUNT (1<<12)
#define DISPCTRL_DIV_HCOUNT_EDGE (1<<13)

#define DISPCTRL_HCOUNT 6
#define DISPCTRL_LCOUNT 0
#define DISPCTRL_FRAC 28

void axi_dispctrl_init(struct axi_dispctrl_private *priv);
void axi_dispctrl_enable(struct axi_dispctrl_private *priv);
void axi_dispctrl_disable(struct axi_dispctrl_private *priv);

#endif /* AXI_DISP_CTRL_H_ */
