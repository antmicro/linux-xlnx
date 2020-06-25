#ifndef FAST_VDMA_H
#define FAST_VDMA_H

#include <uapi/misc/fast-vdma-ioctl.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/ktime.h>
#include <linux/io.h>

#define FAST_VDMA_MEM_BASE 0x00000000
#define FAST_VDMA_MEM_CPU_BASE 0x50000000
#define FAST_VDMA_MEM_SIZE (4 * 1024 * 1024)

struct fast_vdma_dev {
	void *base_mm2s;
	void *base_s2mm;
	void *src_virt;
	void *dst_virt;
	unsigned int irq;
	dma_addr_t src_buf;
	dma_addr_t dst_buf;
	struct completion dma_done;
	ktime_t dma_done_timestamp;
};

struct fast_vdma_ops;

struct fast_vdma_priv {
	struct miscdevice mdev;
	struct platform_device *pdev;
	struct fast_vdma_dev dev;
	struct transfer_config cfg;
	struct transfer_result res;
	struct fast_vdma_ops *ops;
};

struct fast_vdma_ops {
	void (*setup) (struct fast_vdma_priv*);
	void (*start) (struct fast_vdma_priv*);
	void (*handler) (struct fast_vdma_priv*);
	void (*stop) (struct fast_vdma_priv*);
};

static void writeReg(void *base, u32 val, u32 off)
{
	writel(val, base + off);
}

static u32 readReg(void *base, u32 off)
{
	return readl(base + off);
}

extern struct fast_vdma_ops dma_ops;
extern struct fast_vdma_ops axi_vdma_ops;

#endif
