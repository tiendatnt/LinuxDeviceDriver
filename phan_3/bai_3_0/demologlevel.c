#include <linux/module.h> /* thu vien nay dinh nghia cac macro nhu module_init va module_exit */
#include <linux/kernel.h> /* thu vien nay cung cap cac message log level */

#define DRIVER_AUTHOR "Nguyen Tien Dat <dat.a3cbq91@gmail.com>"
#define DRIVER_DESC   "A demonstration about message log levels"


static int __init init_demo_loglevel(void)
{
    printk("default level\n");
    printk(KERN_EMERG "emergency level\n");
    printk(KERN_ALERT "alert level\n");
    printk(KERN_CRIT "critical level\n");
    printk(KERN_ERR "error level\n");
    printk(KERN_WARNING "warning level\n");
    printk(KERN_NOTICE "notice level\n");
    printk(KERN_INFO "information level\n");
    printk(KERN_DEBUG "debug level\n");
    return 0;
}

static void __exit exit_demo_loglevel(void)
{
    printk(KERN_INFO "end demo log level\n");
}

module_init(init_demo_loglevel);
module_exit(exit_demo_loglevel);

MODULE_LICENSE("GPL"); /* giay phep su dung cua module */
MODULE_AUTHOR(DRIVER_AUTHOR); /* tac gia cua module */
MODULE_DESCRIPTION(DRIVER_DESC); /* mo ta chuc nang cua module */
MODULE_SUPPORTED_DEVICE("testdevice"); /* kieu device ma module ho tro */
