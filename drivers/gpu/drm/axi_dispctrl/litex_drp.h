#ifndef _LITEX_DRP_H_
#define _LITEX_DRP_H_

//CSRs

#define DRP_RESET   0x00
#define DRP_LOCKED  0x04
#define DRP_READ    0x08
#define DRP_WRITE   0x0c
#define DRP_DRDY    0x10
#define DRP_ADR     0x14
#define DRP_DAT_W   0x18
#define DRP_DAT_R   0x1c

//DRP registers

#define DRP_CLKDIV_EDGE     (1<<13)
#define DRP_CLK_FRAC_EN     (1<<11)
#define DRP_CLK_EDGE        (1<<7)
#define DRP_CLK_NO_COUNT    (1<<6)

#define DRP_LCOUNT  0
#define DRP_HCOUNT  6
#define DRP_FRAC    12

#define DRP_CLK0L   0x08
#define DRP_CLK0H   0x09

#define DRP_CLK1L   0x0a
#define DRP_CLK1H   0x0b

#define DRP_CLKFBL  0x14
#define DRP_CLKFBH  0x15

#define DRP_CLKDIV  0x16

#define DRP_LOCK1   0x18
#define DRP_LOCK2   0x19
#define DRP_LOCK3   0x1a

#define DRP_PWR     0x28

#define DRP_FLT1    0x4e
#define DRP_FLT2    0x4f

#define DRP_CLK0L_MASK  0x1000
#define DRP_CLK0H_MASK  0x8000

#define DRP_CLK1L_MASK  0x1000
#define DRP_CLK1H_MASK  0xfc00

#define DRP_CLKFBL_MASK 0x1000
#define DRP_CLKFBH_MASK 0x8000

#define DRP_CLKDIV_MASK 0xc000

#define DRP_LOCK1_MASK  0xfc00
#define DRP_LOCK2_MASK  0x8000
#define DRP_LOCK3_MASK  0x8000

#define DRP_PWR_MASK    0x0000

#define DRP_FLT1_MASK   0x66ff
#define DRP_FLT2_MASK   0x666f

#endif
