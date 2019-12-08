/*
 * ten file: vchar_driver.c
 * tac gia : dat.a3cbq91@gmail.com
 * ngay tao: 9/12/2018
 * mo ta   : char driver cho thiet bi gia lap vchar_device.
 *           vchar_device la mot thiet bi nam tren RAM.
 */

#include <linux/module.h> /* thu vien nay dinh nghia cac macro nhu module_init va module_exit */
#include <linux/fs.h> /* thu vien nay dinh nghia cac ham cap phat/giai phong device number */
#include <linux/device.h> /* thu vien nay chua cac ham phuc vu viec tao device file */
#include <linux/slab.h> /* thu vien nay chua cac ham kmalloc va kfree */
#include <linux/gfp.h> /* thu vien nay chua ham get_zeroed_page */
#include <linux/cdev.h> /* thu vien nay chua cac ham lam viec voi cdev */
#include <linux/uaccess.h> /* thu vien nay chua cac ham trao doi du lieu giua user va kernel */
#include <linux/ioctl.h> /* thu vien nay chua cac ham phuc vu ioctl */

#include "vchar_driver.h" /* thu vien mo ta cac thanh ghi cua vchar device */

#define DRIVER_AUTHOR "Nguyen Tien Dat <dat.a3cbq91@gmail.com>"
#define DRIVER_DESC   "A sample character device driver"
#define DRIVER_VERSION "5.1"
#define MAGICAL_NUMBER 243
#define VCHAR_CLR_DATA_REGS _IO(MAGICAL_NUMBER, 0)
#define VCHAR_GET_STS_REGS  _IOR(MAGICAL_NUMBER, 1, sts_regs_t *)
#define VCHAR_SET_RD_DATA_REGS _IOW(MAGICAL_NUMBER, 2, unsigned char *)
#define VCHAR_SET_WR_DATA_REGS _IOW(MAGICAL_NUMBER, 3, unsigned char *)

typedef struct {
        unsigned char read_count_h_reg;
        unsigned char read_count_l_reg;
        unsigned char write_count_h_reg;
        unsigned char write_count_l_reg;
        unsigned char device_status_reg;
} sts_regs_t;

typedef struct vchar_dev {
	unsigned char * control_regs;
	unsigned char * status_regs;
	unsigned char * data_regs;
} vchar_dev_t;

struct _vchar_drv {
	dev_t dev_num;
	struct class *dev_class;
	struct device *dev;
	vchar_dev_t * vchar_hw;
	struct cdev *vcdev;
	unsigned int open_cnt;
} vchar_drv;

/****************************** device specific - START *****************************/
/* ham khoi tao thiet bi */
int vchar_hw_init(vchar_dev_t *hw)
{
	char * buf;
	buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!buf) {
		return -ENOMEM;
	}

	hw->control_regs = buf;
	hw->status_regs = hw->control_regs + NUM_CTRL_REGS;
	hw->data_regs = hw->status_regs + NUM_STS_REGS;

	//khoi tao gia tri ban dau cho cac thanh ghi
	hw->control_regs[CONTROL_ACCESS_REG] = 0x03;
	hw->status_regs[DEVICE_STATUS_REG] = 0x03;

	return 0;
}

/* ham giai phong thiet bi */
void vchar_hw_exit(vchar_dev_t *hw)
{
	free_page((unsigned long)hw->control_regs);
}


/* ham doc tu cac thanh ghi du lieu cua thiet bi */
int vchar_hw_read_data(vchar_dev_t *hw, int start_reg, int num_regs, char* kbuf)
{
	int read_bytes = num_regs;

	//kiem tra xem co quyen doc du lieu khong
	if ((hw->control_regs[CONTROL_ACCESS_REG] & CTRL_READ_DATA_BIT) == DISABLE)
		return -1;
	//kiem tra xem dia chi cua kernel buffer co hop le khong
	if(kbuf == NULL)
		return -1;
	//kiem tra xem vi tri cua cac thanh ghi can doc co hop ly khong
	if(start_reg > NUM_DATA_REGS)
		return -1;
	//dieu chinh lai so luong thanh ghi du lieu can doc (neu can thiet)
	if(num_regs > (NUM_DATA_REGS - start_reg))
		read_bytes = NUM_DATA_REGS - start_reg;
	//ghi du lieu tu kernel buffer vao cac thanh ghi du lieu
	memcpy(kbuf, hw->data_regs + start_reg, read_bytes);

	//cap nhat so lan doc tu cac thanh ghi du lieu
	hw->status_regs[READ_COUNT_L_REG] += 1;
	if(hw->status_regs[READ_COUNT_L_REG] == 0)
		hw->status_regs[READ_COUNT_H_REG] += 1;
	//tra ve so byte da doc duoc tu cac thanh ghi du lieu
	return read_bytes;
}

/* ham ghi vao cac thanh ghi du lieu cua thiet bi */
int vchar_hw_write_data(vchar_dev_t *hw, int start_reg, int num_regs, char* kbuf)
{
	int write_bytes = num_regs;

	//kiem tra xem co quyen ghi du lieu khong
	if ((hw->control_regs[CONTROL_ACCESS_REG] & CTRL_WRITE_DATA_BIT) == DISABLE)
		return -1;
	//kiem tra xem dia chi cua kernel buffer co hop le khong
	if(kbuf == NULL)
		return -1;
	//kiem tra xem vi tri cua cac thanh ghi can ghi co hop ly khong
	if(start_reg > NUM_DATA_REGS)
		return -1;
	//dieu chinh lai so luong thanh ghi du lieu can ghi (neu can thiet)
	if(num_regs > (NUM_DATA_REGS - start_reg)) {
		write_bytes = NUM_DATA_REGS - start_reg;
		hw->status_regs[DEVICE_STATUS_REG] |= STS_DATAREGS_OVERFLOW_BIT;
	}
	//doc du lieu tu cac thanh ghi du lieu vao kernel buffer
	memcpy(hw->data_regs + start_reg, kbuf, write_bytes);

	//cap nhat so lan ghi vao cac thanh ghi du lieu
	hw->status_regs[WRITE_COUNT_L_REG] += 1;
	if(hw->status_regs[WRITE_COUNT_L_REG] == 0)
		hw->status_regs[WRITE_COUNT_H_REG] += 1;
	//tra ve so byte da ghi vao cac thanh ghi du lieu
	return write_bytes;
}

int vchar_hw_clear_data(vchar_dev_t *hw)
{
	if ((hw->control_regs[CONTROL_ACCESS_REG] & CTRL_WRITE_DATA_BIT) == DISABLE)
		return -1;

	memset(hw->data_regs, 0, NUM_DATA_REGS * REG_SIZE);
	hw->status_regs[DEVICE_STATUS_REG] &= ~STS_DATAREGS_OVERFLOW_BIT;
	return 0;
}

/* ham doc tu cac thanh ghi trang thai cua thiet bi */
void vchar_hw_get_status(vchar_dev_t *hw, sts_regs_t *status)
{
	memcpy(status, hw->status_regs, NUM_STS_REGS * REG_SIZE);
}

/* ham ghi vao cac thanh ghi dieu khien cua thiet bi */
// ham cho phep doc tu cac thanh ghi du lieu cua thiet bi
void vchar_hw_enable_read(vchar_dev_t *hw, unsigned char isEnable)
{
	if(isEnable == ENABLE) {
		//dieu khien cho phep doc tu cac thanh ghi du lieu
		hw->control_regs[CONTROL_ACCESS_REG] |= CTRL_READ_DATA_BIT;
		//cap nhat trang thai "co the doc"
		hw->status_regs[DEVICE_STATUS_REG] |= STS_READ_ACCESS_BIT;
	} else {
		//dieu khen khong cho phep doc tu cac thanh ghi du lieu
		hw->control_regs[CONTROL_ACCESS_REG] &= ~CTRL_READ_DATA_BIT;
		//cap nhat trang thai "khong the doc"
		hw->status_regs[DEVICE_STATUS_REG] &= ~STS_READ_ACCESS_BIT;
	}
}

// ham cho phep ghi vao cac thanh ghi du lieu cua thiet bi
void vchar_hw_enable_write(vchar_dev_t *hw, unsigned char isEnable)
{
	if(isEnable == ENABLE) {
		//dieu khien cho phep ghi vao cac thanh ghi du lieu
		hw->control_regs[CONTROL_ACCESS_REG] |= CTRL_WRITE_DATA_BIT;
		//cap nhat trang thai "co the ghi"
		hw->status_regs[DEVICE_STATUS_REG] |= STS_WRITE_ACCESS_BIT;
	} else {
		//dieu khien khong cho ghi vao cac thanh ghi du lieu
		hw->control_regs[CONTROL_ACCESS_REG] &= ~CTRL_WRITE_DATA_BIT;
		//cap nhat trang thai "khong the ghi"
		hw->status_regs[DEVICE_STATUS_REG] &= ~STS_WRITE_ACCESS_BIT;
	}
}

/* ham xu ly tin hieu ngat gui tu thiet bi */

/******************************* device specific - END *****************************/

/******************************** OS specific - START *******************************/
/* cac ham entry points */
static int vchar_driver_open(struct inode *inode, struct file *filp)
{
	vchar_drv.open_cnt++;
	printk("Handle opened event (%d)\n", vchar_drv.open_cnt);
	return 0;
}

static int vchar_driver_release(struct inode *inode, struct file *filp)
{
	printk("Handle closed event\n");
	return 0;
}

static ssize_t vchar_driver_read(struct file *filp, char __user *user_buf, size_t len, loff_t *off)
{
	char *kernel_buf = NULL;
	int num_bytes = 0;

	printk("Handle read event start from %lld, %zu bytes\n", *off, len);

	kernel_buf = kzalloc(len, GFP_KERNEL);
	if(kernel_buf == NULL)
		return 0;

	num_bytes = vchar_hw_read_data(vchar_drv.vchar_hw, *off, len, kernel_buf);
	printk("read %d bytes from HW\n", num_bytes);

	if(num_bytes < 0)
		return -EFAULT;
	if(copy_to_user(user_buf, kernel_buf, num_bytes))
		return -EFAULT;

	*off += num_bytes;
	return num_bytes;
}

static ssize_t vchar_driver_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	char *kernel_buf = NULL;
	int num_bytes = 0;
	printk("Handle write event start from %lld, %zu bytes\n", *off, len);

	kernel_buf = kzalloc(len, GFP_KERNEL);
	if(copy_from_user(kernel_buf, user_buf, len))
		return -EFAULT;

	num_bytes = vchar_hw_write_data(vchar_drv.vchar_hw, *off, len, kernel_buf);
	printk("writes %d bytes to HW\n", num_bytes);

	if(num_bytes < 0)
		return -EFAULT;

	*off += num_bytes;
	return num_bytes;
}

static long vchar_driver_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	printk("Handle ioctl event (cmd: %u)\n", cmd);

	switch(cmd) {
		case VCHAR_CLR_DATA_REGS:
		{
			ret = vchar_hw_clear_data(vchar_drv.vchar_hw);
			if (ret < 0)
				printk("Can not clear data registers\n");
			else
				printk("Data registers have been cleared\n");
		}
			break;
		case VCHAR_SET_RD_DATA_REGS:
		{
			unsigned char isReadEnable;
			copy_from_user(&isReadEnable, (unsigned char*)arg, sizeof(isReadEnable));
			vchar_hw_enable_read(vchar_drv.vchar_hw, isReadEnable);
			printk("Data registers have been %s to read\n", (isReadEnable == ENABLE)?"enabled":"disabled");
		}
			break;
		case VCHAR_SET_WR_DATA_REGS:
		{
			unsigned char isWriteEnable;
			copy_from_user(&isWriteEnable, (unsigned char*)arg, sizeof(isWriteEnable));
			vchar_hw_enable_write(vchar_drv.vchar_hw, isWriteEnable);
			printk("Data registers have been %s to write\n", (isWriteEnable == ENABLE)?"enabled":"disabled");
		}
			break;
		case VCHAR_GET_STS_REGS:
		{
			sts_regs_t status;
			vchar_hw_get_status(vchar_drv.vchar_hw, &status);
			copy_to_user((sts_regs_t*)arg, &status, sizeof(status));
			printk("Got information from status registers\n");
		}
			break;
	}
	return ret;
}

static struct file_operations fops =
{
	.owner   = THIS_MODULE,
	.open    = vchar_driver_open,
	.release = vchar_driver_release,
	.read    = vchar_driver_read,
	.write   = vchar_driver_write,
	.unlocked_ioctl = vchar_driver_ioctl,
};

/* ham khoi tao driver */
static int __init vchar_driver_init(void)
{
	int ret = 0;

	/* cap phat device number */
	vchar_drv.dev_num = 0;
	ret = alloc_chrdev_region(&vchar_drv.dev_num, 0, 1, "vchar_device");
	if (ret < 0) {
		printk("failed to register device number dynamically\n");
		goto failed_register_devnum;
	}
	printk("allocated device number (%d,%d)\n", MAJOR(vchar_drv.dev_num), MINOR(vchar_drv.dev_num));

	/* tao device file */
	vchar_drv.dev_class = class_create(THIS_MODULE, "class_vchar_dev");
	if(vchar_drv.dev_class == NULL) {
		printk("failed to create a device class\n");
		goto failed_create_class;
	}
	vchar_drv.dev = device_create(vchar_drv.dev_class, NULL, vchar_drv.dev_num, NULL, "vchar_dev");
	if(IS_ERR(vchar_drv.dev)) {
		printk("failed to create a device\n");
		goto failed_create_device;
	}

	/* cap phat bo nho cho cac cau truc du lieu cua driver va khoi tao */
	vchar_drv.vchar_hw = kzalloc(sizeof(vchar_dev_t), GFP_KERNEL);
	if(!vchar_drv.vchar_hw) {
		printk("failed to allocate data structure of the driver\n");
		ret = -ENOMEM;
		goto failed_allocate_structure;
	}

	/* khoi tao thiet bi vat ly */
	ret = vchar_hw_init(vchar_drv.vchar_hw);
	if(ret < 0) {
		printk("failed to initialize a virtual character device\n");
		goto failed_init_hw;
	}

	/* dang ky cac entry point voi kernel */
	vchar_drv.vcdev = cdev_alloc();
	if(vchar_drv.vcdev == NULL) {
		printk("failed to allocate cdev structure\n");
		goto failed_allocate_cdev;
	}
	cdev_init(vchar_drv.vcdev, &fops);
	ret = cdev_add(vchar_drv.vcdev, vchar_drv.dev_num, 1);
	if(ret < 0) {
		printk("failed to add a char device to the system\n");
		goto failed_allocate_cdev;
	}

	/* dang ky ham xu ly ngat */

	printk("Initialize vchar driver successfully\n");
	return 0;

failed_allocate_cdev:
	vchar_hw_exit(vchar_drv.vchar_hw);
failed_init_hw:
	kfree(vchar_drv.vchar_hw);
failed_allocate_structure:
	device_destroy(vchar_drv.dev_class, vchar_drv.dev_num);
failed_create_device:
	class_destroy(vchar_drv.dev_class);
failed_create_class:
	unregister_chrdev_region(vchar_drv.dev_num, 1);
failed_register_devnum:
	return ret;
}

/* ham ket thuc driver */
static void __exit vchar_driver_exit(void)
{
	/* huy dang ky xu ly ngat */

	/* huy dang ky entry point voi kernel */
	cdev_del(vchar_drv.vcdev);

	/* giai phong thiet bi vat ly */
	vchar_hw_exit(vchar_drv.vchar_hw);

	/* giai phong bo nho da cap phat cau truc du lieu cua driver */
	kfree(vchar_drv.vchar_hw);

	/* xoa bo device file */
	device_destroy(vchar_drv.dev_class, vchar_drv.dev_num);
	class_destroy(vchar_drv.dev_class);

	/* giai phong device number */
	unregister_chrdev_region(vchar_drv.dev_num, 1);

	printk("Exit vchar driver\n");
}
/********************************* OS specific - END ********************************/

module_init(vchar_driver_init);
module_exit(vchar_driver_exit);

MODULE_LICENSE("GPL"); /* giay phep su dung cua module */
MODULE_AUTHOR(DRIVER_AUTHOR); /* tac gia cua module */
MODULE_DESCRIPTION(DRIVER_DESC); /* mo ta chuc nang cua module */
MODULE_VERSION(DRIVER_VERSION); /* mo ta phien ban cuar module */
MODULE_SUPPORTED_DEVICE("testdevice"); /* kieu device ma module ho tro */
