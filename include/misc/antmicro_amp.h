#ifndef ANTMICRO_AMP_H
#define ANTMICRO_AMP_H

#include <uapi/misc/antmicro_amp.h>

#define AMP_MAJOR_NUMBER 240 /*Experimental device */
#define DEVICE_NAME "Antmicro AMP loader"

DEFINE_MUTEX(device_open_lock);
DEFINE_MUTEX(cpu_run_lock);

extern int zynq_cpun_start(u32 address, int cpu);
extern void zynq_slcr_write(u32 val, u32 offset);

struct amp_private_data
{
        struct device *dev;
        bool running;
        bool file_opened;
        /* start and end of mapped memory */
        uint32_t mm_start;
        uint32_t mm_end;
        uint8_t *mmio;  /*memory for the second core*/
        uint32_t start_address; /*entry point for the second cpu*/
        uint32_t load_address; /*load address of the program for the second CPU*/
};

#endif
