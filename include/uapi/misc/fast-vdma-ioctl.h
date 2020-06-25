#include <linux/types.h>
#ifndef FAST_VDMA_IOCTL_H
#define FAST_VDMA_IOCTL_H

#define FAST_VDMA_DIR_R 1
#define FAST_VDMA_DIR_W 2
#define FAST_VDMA_DIR_RW (FAST_VDMA_DIR_R | FAST_VDMA_DIR_W)

struct transfer_config {
	unsigned int direction;
	unsigned int width;
	unsigned int height;
	unsigned int stride;
	dma_addr_t src_buf;
	dma_addr_t dst_buf;
};

struct transfer_result {
	long long cfg_duration;
	long long xfer_duration;
};

#endif
