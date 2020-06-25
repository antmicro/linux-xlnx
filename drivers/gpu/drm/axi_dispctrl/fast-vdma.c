#include "fast-vdma.h"
#include <uapi/misc/fast-vdma-ioctl.h>

#define FAST_VDMA_CTL 0x0
#define FAST_VDMA_STA 0x4
#define FAST_VDMA_IMR 0x8
#define FAST_VDMA_ISR 0xc

#define FAST_VDMA_RD_ADDR 0x10
#define FAST_VDMA_RD_LEN 0x14
#define FAST_VDMA_RD_CNT 0x18
#define FAST_VDMA_RD_GAP 0x1c

#define FAST_VDMA_WR_ADDR 0x20
#define FAST_VDMA_WR_LEN 0x24
#define FAST_VDMA_WR_CNT 0x28
#define FAST_VDMA_WR_GAP 0x2c

#define FAST_VDMA_IRQ_MASK 0x3
#define FAST_VDMA_EN 0x3
#define FAST_VDMA_LOOP_EN 0x30
#define FAST_VDMA_OVERRIDE_SYNC 0xC

void fast_vdma_setup_chan(void* base, u32 src, u32 dst, u32 width, u32 height, u32 stride)
{

	writeReg(base, src, FAST_VDMA_RD_ADDR);
	writeReg(base, width, FAST_VDMA_RD_LEN);
	writeReg(base, height, FAST_VDMA_RD_CNT);
	writeReg(base, stride, FAST_VDMA_RD_GAP);

	writeReg(base, dst, FAST_VDMA_WR_ADDR);
	writeReg(base, width, FAST_VDMA_WR_LEN);
	writeReg(base, height, FAST_VDMA_WR_CNT);
	writeReg(base, stride, FAST_VDMA_WR_GAP);

	writeReg(base, FAST_VDMA_IRQ_MASK, FAST_VDMA_IMR);
}

void fast_vdma_setup(struct fast_vdma_priv *priv)
{
    unsigned int base = (unsigned int)priv->dev.src_buf;
	fast_vdma_setup_chan(priv->dev.base_mm2s, base, 0, priv->cfg.width, priv->cfg.height, priv->cfg.stride);
}

static void fast_vdma_start(struct fast_vdma_priv *priv)
{
	writeReg(priv->dev.base_mm2s, FAST_VDMA_EN | FAST_VDMA_LOOP_EN, FAST_VDMA_CTL);
}

static void fast_vdma_stop(struct fast_vdma_priv *priv)
{
	writeReg(priv->dev.base_mm2s, 0, FAST_VDMA_CTL);
}

static void fast_vdma_handle_irq(void *base)
{
	writeReg(base, FAST_VDMA_IRQ_MASK, FAST_VDMA_ISR);
}

static void fast_vdma_handler(struct fast_vdma_priv *priv)
{
	fast_vdma_handle_irq(priv->dev.base_mm2s);
}

struct fast_vdma_ops dma_ops = {
	.setup = fast_vdma_setup,
	.start = fast_vdma_start,
	.handler = fast_vdma_handler,
	.stop = fast_vdma_stop,
};
