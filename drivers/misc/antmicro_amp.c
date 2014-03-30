/*
 *  Driver for AMP execution on Zynq CPUs control
 *  Copyright (C) 2014 Antmicro Ltd.
 *  Author Karol Gugala <kgugala@antmicro.com>
 *  Some of the code is based on the AMP implementation by Petalogix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <misc/antmicro_amp.h>

static struct amp_private_data amp_priv;

static int amp_start_second_cpu(uint32_t address)
{
	uint32_t ret;
	
	mutex_lock(&cpu_run_lock);
	if(amp_priv.running)
	{
		dev_info(amp_priv.dev, "Tried to start already running cpu \n");
		return -EINVAL;
	}	

	ret = zynq_cpun_start(address, 1);
	if(!ret)
		amp_priv.running = true;
	mutex_unlock(&cpu_run_lock);
	return ret;
}

static int amp_stop_second_cpu(void)
{
	mutex_lock(&cpu_run_lock);
	if(!amp_priv.running)
	{
		dev_info(amp_priv.dev, "Tried to stop already stopped cpu \n");
		return -EINVAL;
	}	
	amp_priv.running = false;
	zynq_slcr_write(0x22, 0x244);
	mutex_unlock(&cpu_run_lock);
	return 0;
}

static int amp_open(struct inode *inode, struct file *file)
{
	file->private_data = &amp_priv;
	mutex_lock(&device_open_lock);
	if(amp_priv.file_opened){
		return -EBUSY;
	}
	amp_priv.file_opened = true;
	mutex_unlock(&device_open_lock);
	return 0;
}

static int amp_close(struct inode *inode, struct file *file)
{
	struct amp_private_data *priv = file->private_data;
	mutex_lock(&device_open_lock);
	priv->file_opened = false;
	file->private_data = NULL;
	mutex_unlock(&device_open_lock);
	return 0;
}

static ssize_t amp_read(struct file *file, char __user *buffer, size_t length, loff_t *offset)
{
	return 0;
}

//TODO: use offset
static ssize_t amp_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset)
{
	struct amp_private_data *priv = file->private_data;
	uint32_t *buf;

/*if second CPU is running we should not change it's program*/
	if(priv->running) {
		dev_err(priv->dev, "Changing program while CPU is runnig may cause undefined behaviour. Please stop the CPU before loading new program\n");
		return -EBUSY;
	}

/* We are about to load program binary for the second CPU, as we cannot check if it is actually 
 * proper program, we check if it is aligned to 4 bytes although */

	if(length % 4) {
		dev_err(priv->dev, "Binary is not aligned to 4 bytes, will not attemp to load it.\n");
		return -EINVAL;
	} 
/* Check if the binary fits the memory mapped for the second CPU */
	if(length > (priv->mm_end - priv->load_address)){
		dev_err(priv->dev, "Size of the binary exceeds the memory provided for the second CPU\n");
		return -ENOMEM;
	}

	buf = (uint32_t*)kmalloc(length, GFP_KERNEL);

	if(!buf) {
		dev_err(priv->dev,"Error allocating memory.\n");
		return -ENOMEM;
	}
	if (copy_from_user(buf, buffer, length) != 0) {
		kfree(buf);
		return -EFAULT;
	}
	memcpy_toio( (void*)(priv->mmio + priv->load_address), buf, length);
	kfree(buf);
	return length;
}

static long amp_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
	struct amp_private_data *priv = file->private_data;
	struct amp_ioctl_data ioctl_data;
	
	switch(ioctl_num) {
		case AMP_SET_START:
			if(copy_from_user(&ioctl_data, (unsigned int*)ioctl_param, sizeof(unsigned int)))
				return -EACCES;
			if((ioctl_data.data < priv->mm_start)|| (ioctl_data.data > priv->mm_end)){
				dev_err(priv->dev,"Cannot set entry point from outside of the processor memory \n");
				return -EINVAL;
			}
			priv->start_address = ioctl_data.data;
			break;
		case AMP_SET_LOAD:
			if(copy_from_user(&ioctl_data, (unsigned int*)ioctl_param, sizeof(unsigned int)))
                                return -EACCES;
			if((ioctl_data.data < priv->mm_start)|| (ioctl_data.data > priv->mm_end)){
                                dev_err(priv->dev,"Cannot set load address outside the processor memory \n");
                                return -EINVAL;
			}
                        priv->load_address = ioctl_data.data;
			break;
		case AMP_START:
			amp_start_second_cpu(priv->start_address);
			break;
		case AMP_STOP:
			amp_stop_second_cpu();
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

struct file_operations fops = {
	.read = amp_read,
	.write = amp_write,
	.unlocked_ioctl = amp_ioctl,
	.open = amp_open,
	.release = amp_close,};

/* platform driver api */

static int amp_probe(struct platform_device *pdev) {
	int32_t ret;
	struct resource *res;
	uint32_t dev_space = 0;

	res = platform_get_resource(pdev,IORESOURCE_MEM, 0);

	if(!res) {
		dev_err(&pdev->dev,"Failed to get device reource \n");
		return -ENODEV;
	}

	/* fill private data */
	amp_priv.dev = &pdev->dev;
	amp_priv.mm_start = res->start;
	amp_priv.mm_end = res->end;
	amp_priv.file_opened = false;

	/* default entry point and load address for the second CPU is 0 */
	amp_priv.start_address = 0;
	amp_priv.load_address = 0;

	dev_space = (res->end - res->start) + 1;
	amp_priv.mmio = ioremap_nocache(res->start, dev_space);
	dev_info(amp_priv.dev, "Mapped 0x%08x - 0x%08x [0x%08x]\n", res->start, res->end, dev_space);
	
	ret = register_chrdev(AMP_MAJOR_NUMBER, DEVICE_NAME, &fops);

	if (ret < 0) {
		dev_err(amp_priv.dev, "Char device registration failed\n");
		iounmap(amp_priv.mmio);
		return -ENODEV;
	}
	
	/* assume that second cpu is running and stop it */
	amp_priv.running = true;
	amp_stop_second_cpu();
	return 0;
}

static int amp_remove(struct platform_device *pdev)
{
	/* if program is running on the second cpu - stop it */
	if(amp_priv.running) {
		amp_stop_second_cpu();
	}
	iounmap(amp_priv.mmio);
	unregister_chrdev(AMP_MAJOR_NUMBER, DEVICE_NAME);
	return 0;
}

/* Match table for of_platform binding */
static struct of_device_id amp_of_match[] = {
         { .compatible = "antmicro,amp-loader", },
         {}
};
MODULE_DEVICE_TABLE(of, amp_of_match);
 
static struct platform_driver amp_platform_driver = {
         .probe = amp_probe, /* Probe method */
         .remove = amp_remove, /* Detach method */
         .driver = {
                 .owner = THIS_MODULE,
                 .name = DEVICE_NAME, /* Driver name */
                 .of_match_table = amp_of_match,
                 },
};

/* module api */

static int __init amp_init(void)
{
	return platform_driver_register(&amp_platform_driver);
}

static void __exit amp_exit(void)
{
	platform_driver_unregister(&amp_platform_driver);
}

module_init(amp_init);
module_exit(amp_exit);

MODULE_AUTHOR("Karol Gugala <kgugala@antmicro.com>");
MODULE_DESCRIPTION("Zynq AMP loader");
MODULE_LICENSE("GPL v2");
