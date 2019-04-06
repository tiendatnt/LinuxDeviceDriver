#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

int8_t read_buf[1024];
int8_t write_buf[1024]="Uoc mo ve mot Viet Nam hung cuong\n";

int main()
{
	int fd = open("/dev/vchar_dev", O_RDWR);

	if (fd < 0) {
		printf("Cannot open device file...\n");
		return 0;
	}

	/* Ghi du lieu vao vi tri dau tien cua device file */
	pwrite(fd, write_buf, strlen(write_buf)+1, 0);

	/* Doc du lieu tu vi tri dau tien cua device file */
	pread(fd, read_buf, 1024, 0);
	printf("Data = %s", read_buf);

	close(fd);

	return 0;
}
