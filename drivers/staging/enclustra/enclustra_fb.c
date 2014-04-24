#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/spinlock.h>           
#include <linux/device.h>           
#include <linux/cdev.h>            
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mm.h>               
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>  
#include <linux/device.h>          
#include <linux/fs.h>                          
#include <linux/pci.h>              
#include <linux/types.h>           
#include <linux/kobject.h>                           
#include <linux/delay.h>            
#include <linux/version.h>       
#include <linux/wait.h>          
#include <linux/sched.h>       
#include <linux/jiffies.h>                   
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>   
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <asm/uaccess.h>   

#include "enclustra_fb.h"

#define DRIVER_NAME                     "enclustra_fb"
#define DRIVER_DESC                     "Enclustra GmbH Frambuffer for TFT"
#define DRIVER_VERSION                  "1.00"

/*****************************************************************************/
#ifdef CONFIG_ENCLUSTRA_FB_DEBUG
unsigned int enclustra_fb_debuglevel    = 0;
#define DPRINTK(level, value...)        {if(level >= enclustra_fb_debuglevel) {printk(KERN_INFO value);}}
#define DRIVER_DEBUG                    "!!! DEBUG VERSION  !!!"
#else
#define DPRINTK(level, value...)        {if(level >= 3) {printk(KERN_INFO value);}}
static char *debug_str = "";
#endif

/*****************************************************************************/
/* Config */
#define ENCLUSTRA_FB_WIDTH              800
#define ENCLUSTRA_FB_HEIGHT             480

/*****************************************************************************/
/* General TFT */
#define BYTES_PER_PIXEL                 2
#define NUMBER_OF_BUFFERS               2
#define ENCLUSTRA_FB_CONTROL_SIZE       0x10000 


/*****************************************************************************/
/* Register Set */
#define ENCLUSTRA_TFTCTRL_REG           0x0000
#define ENCLUSTRA_TFTCTRL_PCHANGE_MSK   0x00000100
#define ENCLUSTRA_TFTCTRL_ERROR_MSK     0x00000010
#define ENCLUSTRA_TFTCTRL_DISABLE_MSK   0x00000002
#define ENCLUSTRA_TFTCTRL_ENABLE_MSK    0x00000001

#define ENCLUSTRA_PAGE_REG              0x0004

#define ENCLUSTRA_BRIGHT_REG            0x0008
#define ENCLUSTRA_BRIGHT_MAX_MSK        0x000000FF

#define ENCLUSTRA_2DCTRL_REG            0x0010

#define ENCLUSTRA_SRCADDR_REG           0x0014

#define ENCLUSTRA_DSTADDR_REG           0x0018

#define ENCLUSTRA_LEN_REG               0x001C

#define ENCLUSTRA_COLOR_REG             0x0020


/*****************************************************************************/
/* Local Defines */
#define STATE_STOPPED                   0
#define STATE_STARTED                   1

/*****************************************************************************/
/* Local Variables */
struct enclustra_fb {
    struct mutex                        lock;
    struct fb_info                      fb;
    struct cdev                         chrdev;
    atomic_t                            open_cnt;
    int                                 init_state;
    void*                               video_base;
    void*                               ctrl_base;
    u32                                 cmap[16];
    dev_t                               major_nr;
    unsigned long                       regs[8];
};

/*****************************************************************************/
/* Function Definitions */
static void enclustra_fb_write_reg(struct enclustra_fb* fb, unsigned long reg_addr, unsigned long reg_data);
static unsigned long enclustra_fb_read_reg(struct enclustra_fb* fb, unsigned long reg_addr);
static int enclustra_fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green, unsigned int blue, unsigned int transp, struct fb_info *info);
static int enclustra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int enclustra_fb_set_par(struct fb_info *info);
static int enclustra_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);
static int enclustra_fb_mmap(struct fb_info *fb, struct vm_area_struct *vma);
static int enclustra_fb_allocate_and_map_video_memory(struct enclustra_fb *fb);
static int enclustra_fb_free_and_unmap_video_memory(struct enclustra_fb *fb);
static int enclustra_fb_probe(struct platform_device *pdev);
static int enclustra_fb_remove(struct platform_device *pdev);

/*****************************************************************************/
#ifdef CONFIG_ENCLUSTRA_FB_EMULATE
unsigned long enclustra_fb_emulate_regs[8];
#endif

/*****************************************************************************/
static void enclustra_fb_write_reg(struct enclustra_fb* fb, unsigned long reg_addr, unsigned long reg_data)
{
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
#ifdef CONFIG_ENCLUSTRA_FB_EMULATE
    enclustra_fb_emulate_regs[(reg_addr/4)] = reg_data;
#else
    if(fb->ctrl_base == NULL) {
        DPRINTK(3, "ctrl_base NULL\n");
        return;
    }
    iowrite32(reg_data, (fb->ctrl_base + reg_addr)); 
#endif
}

/*****************************************************************************/
static unsigned long enclustra_fb_read_reg(struct enclustra_fb* fb, unsigned long reg_addr)
{
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
#ifdef CONFIG_ENCLUSTRA_FB_EMULATE
    return enclustra_fb_emulate_regs[(reg_addr/4)];
#else
    if(fb->ctrl_base == NULL) {
        DPRINTK(3, "ctrl_base NULL\n");
        return -1 ;
    }
    return ioread32((fb->ctrl_base + reg_addr));
#endif
}


/*****************************************************************************/
static inline u32 convert_bitfield(int val, struct fb_bitfield *bf)
{
    unsigned int mask = (1 << bf->length) - 1;

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    return (val >> (16 - bf->length) & mask) << bf->offset;
}


/*****************************************************************************/
static int enclustra_fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
         unsigned int blue, unsigned int transp, struct fb_info *info)
{
    struct enclustra_fb  *fb = container_of(info, struct enclustra_fb, fb);

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
        
    if (regno < 16) 
    {
        fb->cmap[regno] = convert_bitfield(transp, &fb->fb.var.transp) |
                  convert_bitfield(blue, &fb->fb.var.blue) |
                  convert_bitfield(green, &fb->fb.var.green) |
                  convert_bitfield(red, &fb->fb.var.red);
        return 0;
    }
    else 
    {
        return 1;
    }
}


/*****************************************************************************/
static int enclustra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    if(var->rotate & 1)
    {
        return -EINVAL;
    }
    else 
    {
        if((var->xres != info->var.xres) ||
           (var->yres != info->var.yres) ||
           (var->xres_virtual != info->var.xres) ||
           (var->yres_virtual > 
            info->var.yres * NUMBER_OF_BUFFERS) ||
           (var->yres_virtual < info->var.yres )) 
        {
            return -EINVAL;
        }
    }
    if((var->xoffset != info->var.xoffset) ||
       (var->bits_per_pixel != info->var.bits_per_pixel) ||
       (var->grayscale != info->var.grayscale)) 
    {
        return -EINVAL;
    }
    return 0;
}


/*****************************************************************************/
static int enclustra_fb_set_par(struct fb_info *info)
{
    struct enclustra_fb *fb = container_of(info, struct enclustra_fb, fb);
    struct fb_var_screeninfo *var = &(info->var);
    unsigned long y_bottom, new_page, reg;

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    if(fb->init_state != STATE_STARTED)
    {
        DPRINTK(3, "controller not initialzed\n");
        return -EFAULT;
    } 
    
    /* Set the frame buffer base to:
       fb->fb.fix.smem_start + fb->fb.var.xres * 
       BYTES_PER_PIXEL * var->yoffset
    */
    y_bottom = var->yoffset;

    if (!(var->vmode & FB_VMODE_YWRAP)) 
    {
        y_bottom += var->yres;
    }

    if (y_bottom > info->var.yres_virtual) 
    {
        return -EINVAL;
    }

    new_page = (var->yoffset * var->xres_virtual + var->xoffset);
    new_page *= (var->bits_per_pixel) / 8;
    new_page += info->fix.smem_start;

#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    /* check if switch has happenend */
    do
    {
        reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    }
    while(reg & ENCLUSTRA_TFTCTRL_PCHANGE_MSK);
#endif

    /* set new page */
    enclustra_fb_write_reg(fb, ENCLUSTRA_PAGE_REG, virt_to_phys((void*)new_page)); 

    /* tell the gpu to switch mem */
    reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    reg |= ENCLUSTRA_TFTCTRL_PCHANGE_MSK;
    enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, reg);

#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    /* check if switch has happenend */
    do
    {
        reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    }
    while(reg & ENCLUSTRA_TFTCTRL_PCHANGE_MSK);
#endif

    return 0;

}


/*****************************************************************************/
static int enclustra_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
    struct enclustra_fb *fb = container_of(info, struct enclustra_fb, fb);
    unsigned long y_bottom, new_page, reg;

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    if(fb->init_state != STATE_STARTED)
    {
        DPRINTK(3, "controller not initialzed\n");
        return -EFAULT;
    } 
    
    /* Set the frame buffer base to:
       fb->fb.fix.smem_start + fb->fb.var.xres * 
       BYTES_PER_PIXEL * var->yoffset
    */
    y_bottom = var->yoffset;

    if (!(var->vmode & FB_VMODE_YWRAP)) 
    {
        y_bottom += var->yres;
    }

    if (y_bottom > info->var.yres_virtual) 
    {
        return -EINVAL;
    }

    new_page = (var->yoffset * var->xres_virtual + var->xoffset);
    new_page *= (var->bits_per_pixel) / 8;
    new_page += info->fix.smem_start;

#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    /* check if switch has happenend */
    do
    {
        reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    }
    while(reg & ENCLUSTRA_TFTCTRL_PCHANGE_MSK);
#endif
    
    /* set new page */
    enclustra_fb_write_reg(fb, ENCLUSTRA_PAGE_REG, virt_to_phys((void*)new_page)); 
    
    /* tell the gpu to switch mem */
    reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    reg |= ENCLUSTRA_TFTCTRL_PCHANGE_MSK;
    enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, reg);

#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    /* check if switch has happenend */
    do
    {
        reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    }
    while(reg & ENCLUSTRA_TFTCTRL_PCHANGE_MSK);
#endif
    
    return 0;
}


/*****************************************************************************/
static int enclustra_fb_mmap(struct fb_info *fb, struct vm_area_struct *vma)
{
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    if ((vma->vm_end - vma->vm_start) > fb->fix.smem_len) {
        DPRINTK(3, "Memory to large to map\n");
        return -EINVAL;
    }

    /* uncached */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    if (remap_pfn_range(vma, 
                        vma->vm_start, 
                        virt_to_phys((void*)fb->fix.smem_start) >> PAGE_SHIFT,
                        (vma->vm_end - vma->vm_start), 
                        vma->vm_page_prot)) {
        DPRINTK(3, "Memory mapping failed\n");
        return -EAGAIN;
    }  
  
    return 0;;
}

/*****************************************************************************/
static long enclustra_fb_cd_ioctl(struct file* filep,
                                unsigned int cmd, 
                                unsigned long arg) 
{
    struct enclustra_fb *fb = (struct enclustra_fb*)(filep->private_data);
    
    int ret = 0;
        
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    mutex_lock(&(fb->lock));
    switch (cmd) {
    
        case ENCLUSTRA_FB_IOC_CMD_SET_BRIGHTNESS:
            if(fb->init_state != STATE_STARTED)
            {
                DPRINTK(3, "controller not initialzed\n");
                mutex_unlock(&(fb->lock));
                return -EFAULT;
            } 
    
            if(arg > ENCLUSTRA_BRIGHT_MAX_MSK) 
            {
                DPRINTK(3, "brigthess higer then max (0-255)\n");
                mutex_unlock(&(fb->lock));
                return -EFAULT;
            }
            enclustra_fb_write_reg(fb, ENCLUSTRA_BRIGHT_REG, arg);
            ret = 0;
            break;
    
        case ENCLUSTRA_FB_IOC_CMD_DEINIT:
            if(fb->init_state != STATE_STARTED) 
            {
                DPRINTK(3, "FB not started\n");
                mutex_unlock(&(fb->lock));
                return -EFAULT;
            }
            fb->init_state = STATE_STOPPED;
            
            msleep(500); // wait some time that all open things are done
            
            /* save the register values */
            fb->regs[0] = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
            fb->regs[1] = enclustra_fb_read_reg(fb, ENCLUSTRA_PAGE_REG);
            fb->regs[2] = enclustra_fb_read_reg(fb, ENCLUSTRA_BRIGHT_REG);
            fb->regs[3] = enclustra_fb_read_reg(fb, ENCLUSTRA_2DCTRL_REG);
            fb->regs[4] = enclustra_fb_read_reg(fb, ENCLUSTRA_SRCADDR_REG);
            fb->regs[5] = enclustra_fb_read_reg(fb, ENCLUSTRA_DSTADDR_REG);
            fb->regs[6] = enclustra_fb_read_reg(fb, ENCLUSTRA_LEN_REG);
            fb->regs[7] = enclustra_fb_read_reg(fb, ENCLUSTRA_COLOR_REG);
            
            msleep(500); // wait some time that all open things are done
            
            /* then disable */
            enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, 0x00000002);
            break;
    
        case ENCLUSTRA_FB_IOC_CMD_REINIT:
            if(fb->init_state != STATE_STOPPED) 
            {
                DPRINTK(3, "FB not stopped\n");
                mutex_unlock(&(fb->lock));
                return -EFAULT;
            }
            
            /* restore the register values */
            enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, fb->regs[0]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_PAGE_REG, fb->regs[1]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_BRIGHT_REG, fb->regs[2]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_2DCTRL_REG, fb->regs[3]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_SRCADDR_REG, fb->regs[4]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_DSTADDR_REG, fb->regs[5]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_LEN_REG, fb->regs[6]);
            enclustra_fb_write_reg(fb, ENCLUSTRA_COLOR_REG, fb->regs[7]);

            fb->init_state = STATE_STARTED;
            break;
    
        default:
            DPRINTK(3, "UNKNOWN CMD received. \n");
            mutex_unlock(&(fb->lock));
            return -ENOTTY;
    }
    mutex_unlock(&(fb->lock));
    return 0;
    
}/*****************************************************************************/
static int enclustra_fb_cd_open(struct inode* inode, 
                               struct file* filep)
{
    struct enclustra_fb *fb = container_of(inode->i_cdev, struct enclustra_fb, chrdev);
    
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    /* asure that the driver can only be opened once */
    if(0 != atomic_read(&(fb->open_cnt)))
    {
        DPRINTK(3, "open failed, is the device already open?\n");
        return -EACCES;
    }
    
    atomic_inc(&(fb->open_cnt));   
    
    filep->private_data = (void*)fb;
    return 0;
}

/*****************************************************************************/
static int enclustra_fb_cd_close(struct inode* inode, 
                                 struct file* filep) 
{
    struct enclustra_fb *fb = container_of(inode->i_cdev, struct enclustra_fb, chrdev);

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    /* asure that the driver can only be closed once open */
    if(0 == atomic_read(&(fb->open_cnt)))
    {
        DPRINTK(3, "close failed, was the device open?\n");
        return -EACCES;
    }
    
    atomic_dec(&(fb->open_cnt));
    
    return 0;
}

/*****************************************************************************/
static struct fb_ops enclustra_fb_ops = {
    .owner          = THIS_MODULE,
    .fb_check_var   = enclustra_fb_check_var,
    .fb_set_par     = enclustra_fb_set_par,
    .fb_setcolreg   = enclustra_fb_setcolreg,
    .fb_pan_display = enclustra_fb_pan_display,
    .fb_mmap        = enclustra_fb_mmap,
    
    /* These are generic software based fb functions */
    .fb_fillrect    = cfb_fillrect,
    .fb_copyarea    = cfb_copyarea,
    .fb_imageblit   = cfb_imageblit,
};

/*****************************************************************************/
static struct file_operations enclustra_fb_cd_fops = 
{ 
    .open = enclustra_fb_cd_open,
    .release = enclustra_fb_cd_close, 
    .unlocked_ioctl = enclustra_fb_cd_ioctl,
};


/*****************************************************************************/
static int enclustra_fb_allocate_and_map_video_memory(struct enclustra_fb *fb)
{
    int ret;
    unsigned long nr_of_pages;

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    // allocate memory
    fb->fb.fix.smem_len = fb->fb.var.yres_virtual * fb->fb.fix.line_length;

    nr_of_pages = (fb->fb.fix.smem_len / PAGE_SIZE) + 2;
    fb->video_base = kmalloc((nr_of_pages * PAGE_SIZE), GFP_KERNEL | GFP_DMA);
    if(fb->video_base == NULL){
        DPRINTK(3, "Unable to allocate framebuffer memory\n");
        ret = -ENOMEM;
        goto err_alloc_mem_fail;
    }
   
    /* align with a page */
    fb->fb.screen_base = (void*)((((unsigned long)fb->video_base) + PAGE_SIZE - 1) & PAGE_MASK);
    fb->fb.fix.smem_start = (int)fb->fb.screen_base;

    printk(KERN_ERR "%s video_mem@0x%08X - 0x%08X (phys_start: 0x%08X) \n", DRIVER_NAME, (unsigned int) fb->fb.fix.smem_start, (unsigned int) (fb->fb.fix.smem_start + fb->fb.fix.smem_len), virt_to_phys((void*)(fb->fb.fix.smem_start)));

    fb->fb.screen_size = fb->fb.fix.smem_len;

    return 0;

err_alloc_mem_fail:
    fb->fb.fix.smem_len = 0;
    fb->fb.fix.smem_start = 0;
    fb->fb.screen_base = NULL;
    fb->fb.screen_size = 0;
    return ret;
}


/*****************************************************************************/
static int enclustra_fb_free_and_unmap_video_memory(struct enclustra_fb *fb)
{
    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    kfree(fb->video_base);
    
    fb->fb.screen_base = NULL;
    fb->fb.screen_size = 0;
    fb->fb.fix.smem_start = 0;
    fb->fb.fix.smem_len = 0;
    return 0;
}


/*****************************************************************************/
static int enclustra_fb_probe(struct platform_device *pdev)
{
    int ret;
    struct class *dev_class;
	struct device *dev;
    struct enclustra_fb *fb;
    size_t framesize;
    unsigned long width, height;
    unsigned long reg;
    struct resource *mem;

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    DPRINTK(3, "************************************************************\n");
    DPRINTK(3, "Enclustra FB\n");
    DPRINTK(3, "************************************************************\n");
    
    fb = kzalloc(sizeof(*fb), GFP_KERNEL);
    if(fb == NULL) {
        DPRINTK(3, "kzalloc failed\n");
        ret = -ENOMEM;
        goto err_fb_alloc_failed;
    }

    /* create locks */
    mutex_init(&(fb->lock));
    atomic_set(&(fb->open_cnt), 0);
    
    // map the private structure to the device device
    dev_set_drvdata(&pdev->dev, fb);
    cdev_init(&(fb->chrdev), &enclustra_fb_cd_fops);
    
    fb->chrdev.owner = THIS_MODULE;
    fb->chrdev.ops = &enclustra_fb_cd_fops;
    
    fb->init_state = STATE_STOPPED;
    
    /* register the charcter device */
    ret = alloc_chrdev_region(&fb->major_nr, 0, 1, DRIVER_NAME);
    if (ret != 0) {
        DPRINTK(3, "fb alloc_chrdev_region failed\n");
        goto err_alloc_chrdev_region_failed;
    } else {
        DPRINTK(3, "%s allocated major = %d minor = %d", DRIVER_NAME, MAJOR(fb->major_nr), MINOR(fb->major_nr));
    }
    
	dev_class = class_create(THIS_MODULE, DRIVER_NAME);
	dev = device_create(dev_class, NULL, fb->major_nr, NULL, DRIVER_NAME);
	if (dev == NULL) {
		ret = -ENOMEM;
        DPRINTK(3, "fb device_create failed\n");
        goto err_device_create_failed;
	}

    ret = cdev_add(&(fb->chrdev), fb->major_nr, 1);
    if (ret != 0) {
        DPRINTK(3, "fb cdev_add failed\n");
        goto err_cdev_add_failed;
    }    

    /* ioremap control memory */
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (mem == NULL) {
        DPRINTK(3, "platform_get_resource failed\n");
        ret = -ENOENT;
        goto err_platform_get_resource_failed;
    }

#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    printk(KERN_ERR "%s ctrl_mem@0x%08X - 0x%08X\n", DRIVER_NAME, mem->start, (mem->start + ENCLUSTRA_FB_CONTROL_SIZE));
    
    if(NULL == request_mem_region(mem->start, ENCLUSTRA_FB_CONTROL_SIZE, DRIVER_NAME)) {
        DPRINTK(3, "request_mem_region failed\n");
        ret = -ENOMEM;
        goto err_request_mem_region_failed;
    }

    fb->ctrl_base = ioremap_nocache(mem->start, ENCLUSTRA_FB_CONTROL_SIZE);
    if(fb->ctrl_base == NULL) {
        DPRINTK(3, "ioremap failed\n");
        ret = -ENOMEM;
        goto err_ioremap_failed;
    }
#endif

    /* disable first */
    enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, ENCLUSTRA_TFTCTRL_DISABLE_MSK);
    
    width = ENCLUSTRA_FB_WIDTH;
    height = ENCLUSTRA_FB_HEIGHT;

    fb->fb.fbops                = &enclustra_fb_ops;

    fb->fb.flags                = FBINFO_FLAG_DEFAULT;
    fb->fb.pseudo_palette       = fb->cmap;
    fb->fb.fix.type             = FB_TYPE_PACKED_PIXELS;
    fb->fb.fix.visual           = FB_VISUAL_TRUECOLOR;
    fb->fb.fix.line_length      = width * BYTES_PER_PIXEL;
    fb->fb.fix.accel            = FB_ACCEL_NONE;
    fb->fb.fix.ypanstep         = 1;

    fb->fb.var.xres             = width;
    fb->fb.var.yres             = height;
    fb->fb.var.xres_virtual     = width;
    fb->fb.var.yres_virtual     = height * NUMBER_OF_BUFFERS;
    fb->fb.var.bits_per_pixel   = BYTES_PER_PIXEL * 8;
    fb->fb.var.activate         = FB_ACTIVATE_NOW;
    fb->fb.var.height           = height;
    fb->fb.var.width            = width;

    fb->fb.var.red.offset       = 11;
    fb->fb.var.red.length       = 5;
    fb->fb.var.green.offset     = 5;
    fb->fb.var.green.length     = 6;
    fb->fb.var.blue.offset      = 0;
    fb->fb.var.blue.length      = 5;
    fb->fb.var.blue.length      = 5;

    framesize = width * height * BYTES_PER_PIXEL * NUMBER_OF_BUFFERS;
    
    /* allocate VIDEO RAM */
    ret = enclustra_fb_allocate_and_map_video_memory(fb);
    if(ret) {
        DPRINTK(3, "alloc_video_ram failed\n");
        goto err_fb_memory_alloc_failed;
    }

    ret = fb_set_var(&fb->fb, &fb->fb.var);
    if(ret) {
        DPRINTK(3, "fb_set_var failed\n");
        goto err_fb_set_var_failed;
    }
        
    /* blank display to white*/
    memset(fb->fb.screen_base, 0xFF, framesize);
    
    /* set initial base */
    enclustra_fb_write_reg(fb, ENCLUSTRA_PAGE_REG, virt_to_phys((void*)(fb->fb.fix.smem_start)));

    /* set max brightness */
    enclustra_fb_write_reg(fb, ENCLUSTRA_BRIGHT_REG, ENCLUSTRA_BRIGHT_MAX_MSK);

    /* enable and set page*/
    reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    reg &= ~ENCLUSTRA_TFTCTRL_DISABLE_MSK;
    reg |= ENCLUSTRA_TFTCTRL_PCHANGE_MSK | ENCLUSTRA_TFTCTRL_ENABLE_MSK;
    enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, reg);
    
#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    /* check if page has been switched */
    do
    {
        reg = enclustra_fb_read_reg(fb, ENCLUSTRA_TFTCTRL_REG);
    }
    while(reg & ENCLUSTRA_TFTCTRL_PCHANGE_MSK);
#endif
    
    fb->init_state = STATE_STARTED;
    
    ret = register_framebuffer(&fb->fb);
    if(ret) {
        DPRINTK(3, "register_framebuffer failed\n");
        goto err_register_framebuffer_failed;
    }
    
    return 0;

err_register_framebuffer_failed:
err_fb_set_var_failed:
    enclustra_fb_free_and_unmap_video_memory(fb);
err_fb_memory_alloc_failed:

#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    iounmap(fb->ctrl_base);
err_ioremap_failed:
    release_mem_region((int)fb->ctrl_base, ENCLUSTRA_FB_CONTROL_SIZE); 
err_request_mem_region_failed:
#endif

err_platform_get_resource_failed:
    cdev_del(&(fb->chrdev));

err_cdev_add_failed:    
    device_destroy(dev_class, fb->major_nr);    

err_device_create_failed:
    class_destroy(dev_class);
    unregister_chrdev_region(fb->major_nr, 1);   
    
err_alloc_chrdev_region_failed:
    kfree(fb);
    
err_fb_alloc_failed:

    return ret;
}


/*****************************************************************************/
static int enclustra_fb_remove(struct platform_device *pdev)
{
    struct enclustra_fb *fb = dev_get_drvdata(&pdev->dev);

    DPRINTK(0, "%s:%s entering function", DRIVER_NAME, __FUNCTION__);
    
    /* disable first */
    unregister_framebuffer(&fb->fb);

    enclustra_fb_write_reg(fb, ENCLUSTRA_TFTCTRL_REG, ENCLUSTRA_TFTCTRL_DISABLE_MSK);
#ifndef CONFIG_ENCLUSTRA_FB_EMULATE
    iounmap(fb->ctrl_base);
    release_mem_region((int)fb->ctrl_base, ENCLUSTRA_FB_CONTROL_SIZE);
#endif

    enclustra_fb_free_and_unmap_video_memory(fb);
    
    kfree(fb);
    return 0;
}


/*****************************************************************************/
static struct of_device_id enclustra_fb_of_match[] = {
    { .compatible = "enclustra,enclustra-fb", },
    {},
};

MODULE_DEVICE_TABLE(of, enclustra_fb_of_match);


/*****************************************************************************/
static struct platform_driver enclustra_fb_of_driver = {
    .probe = enclustra_fb_probe,
    .remove = enclustra_fb_remove,
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = enclustra_fb_of_match,
    },
};

module_platform_driver(enclustra_fb_of_driver);


/*****************************************************************************/
MODULE_AUTHOR("Enclustra GmbH, Sven Meier <sven.meier@enclustra.com>");
MODULE_DESCRIPTION("Enclustra TFT frame buffer driver");

