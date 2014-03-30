#ifndef __ANTMICRO_AMP_H__
#define __ANTMICOR_AMP_H__

struct amp_ioctl_data {
        unsigned int data;
};

/* ioctls */
#define AMP_IOCTL_MAGIC         'A'

#define AMP_SET_START           _IOW(AMP_IOCTL_MAGIC, 1, struct amp_ioctl_data)
#define AMP_SET_LOAD            _IOW(AMP_IOCTL_MAGIC, 2, struct amp_ioctl_data)
#define AMP_START               _IO(AMP_IOCTL_MAGIC, 3)
#define AMP_STOP                _IO(AMP_IOCTL_MAGIC, 4)

#endif
