#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define DEVICE_NODE "/dev/vchar_dev"
#define MAGICAL_NUM 243
#define SHOW_THEN_RESET_CRITICAL_RESOURCE _IO(MAGICAL_NUM, 6)

int main()
{
	int fd;

	fd = open(DEVICE_NODE, O_RDWR);
	if(fd < 0) {
		printf("Cannot open device file...\n");
		return -1;
	}

	ioctl(fd, SHOW_THEN_RESET_CRITICAL_RESOURCE);

	close(fd);
	return 0;
}
