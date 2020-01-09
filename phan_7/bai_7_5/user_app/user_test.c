/*
 * ten file: user_test.c
 * tac gia : dat.a3cbq91@gmail.com
 * ngay tao: 9/12/2018
 * mo ta   : day la chuong trinh tren user space tuong tac voi vchar_device
 *           vchar_device la mot thiet bi nam tren RAM.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

typedef struct {
        unsigned char read_count_h;
        unsigned char read_count_l;
        unsigned char write_count_h;
        unsigned char write_count_l;
        unsigned char device_status;
} status_t;

#define BUFFER_SIZE 1024
#define DEVICE_NODE "/dev/vchar_dev"
#define MAGICAL_NUM 243
#define CLEAR_DATA_CHARDEV _IO(MAGICAL_NUM, 0)
#define GET_STATUS_CHARDEV _IOR(MAGICAL_NUM, 1, status_t *)
#define CTRL_READ_CHARDEV  _IOW(MAGICAL_NUM, 2, unsigned char *)
#define CTRL_WRITE_CHARDEV _IOW(MAGICAL_NUM, 3, unsigned char *)

/* ham kiem tra entry point open cua vchar driver */
int open_chardev() {
	int fd = open(DEVICE_NODE, O_RDWR);
	if(fd < 0) {
		printf("Can not open the device file\n");
		exit(1);
	}
	return fd;
}

/* ham kiem tra entry point release cua vchar driver */
void close_chardev(int fd) {
	close(fd);
}

/* ham kiem tra entry point read cua vchar driver */
void read_data_chardev() {
	int ret = 0;
	char user_buf[BUFFER_SIZE];

	int fd = open_chardev();
	ret = read(fd, user_buf, BUFFER_SIZE);
	close_chardev(fd);

	if (ret < 0)
		printf("Could not read a message from %s\n", DEVICE_NODE);
	else
		printf("Read a message from %s: %s\n", DEVICE_NODE, user_buf);
}

/* ham kiem tra entry point write cua vchar driver */
void write_data_chardev() {
	int ret = 0;
	char user_buf[BUFFER_SIZE];
	printf("Enter your message: ");
	scanf(" %[^\n]s", user_buf);

	int fd = open_chardev();
	ret = write(fd, user_buf, strlen(user_buf) + 1); //ghi chuoi ky tu, bao gom ca NULL
	close_chardev(fd);

	if (ret < 0)
		printf("Could not write the message to %s\n", DEVICE_NODE);
	else
		printf("Wrote the message to %s\n", DEVICE_NODE);
}

/* cac ham kiem tra entry point ioctl cua vchar driver */
void clear_data_chardev() {
	int fd = open_chardev();
	int ret = ioctl(fd, CLEAR_DATA_CHARDEV);
	close_chardev(fd);
	printf("%s data registers in char device\n", (ret < 0)?"Couldn't clear":"Cleared");
}

void get_status_chardev() {
	status_t status;
	unsigned int read_cnt, write_cnt;

	int fd = open_chardev();
	ioctl(fd, GET_STATUS_CHARDEV, (status_t*)&status);
	close_chardev(fd);

	read_cnt = status.read_count_h << 8 | status.read_count_l;
	write_cnt = status.write_count_h << 8 | status.write_count_l;
	printf("Statistic: number of reading(%u), number of writing (%u)\n", read_cnt, write_cnt);
}

void control_read_chardev() {
	unsigned char isReadable = 0;
	status_t status;
	char c = 'n';
	printf("Do you want to enable reading from data registers (y/n)? ");
	scanf(" %c", &c);
	if(c == 'y')
		isReadable = 1;
	else if(c == 'n')
		isReadable = 0;
	else
		return;

	int fd = open_chardev();
	ioctl(fd, CTRL_READ_CHARDEV, (unsigned char*)&isReadable);
	ioctl(fd, GET_STATUS_CHARDEV, (status_t*)&status);
	close_chardev(fd);
	if(status.device_status & 0x01)
		printf("Enable to read from data registers successful\n");
	else
		printf("Disable to read from data registers successful\n");
}

void control_write_chardev() {
	unsigned char isWriteable = 0;
	status_t status;
	char c = 'n';
	printf("Do you want to enable writing to data registers (y/n)? ");
	scanf(" %c", &c);
	if(c == 'y')
		isWriteable = 1;
	else if(c == 'n')
		isWriteable = 0;
	else
		return;

	int fd = open_chardev();
	ioctl(fd, CTRL_WRITE_CHARDEV, (unsigned char*)&isWriteable);
	ioctl(fd, GET_STATUS_CHARDEV, (status_t*)&status);
	close_chardev(fd);
	if(status.device_status & 0x02)
		printf("Enable to write to data registers successful\n");
	else
		printf("Disable to write to data registers successful\n");
}

void mem_map_chardev()
{
	//anh xa kernel buffer vao vung mapped area tren user space
	int fd = open_chardev();
	off_t offset = 0;
	size_t mapped_size = getpagesize();
	char* mapped_area = mmap(NULL, mapped_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
	if(mapped_area == MAP_FAILED) {
		printf("memory mapping failed\n");
		return;
	}
	close_chardev(fd);

	//ghi du lieu vao vung mapped area
	printf("Enter your message: ");
	scanf(" %[^\n]s", mapped_area);
	printf("Wrote the message to mapped area\n");

	//doc du lieu tu vung mapped area
	printf("Read a message from mapped area: %s\n", mapped_area);

	//huy vung mapped area
	munmap((void*)mapped_area, mapped_size);
}

int main() {
	int ret = 0;
	char option = 'q';
	int fd = -1;
	printf("Select below options:\n");
	printf("\to (to open a device node)\n");
	printf("\tc (to close the device node)\n");
	printf("\tr (to read data from device node\n");
	printf("\tw (to write data to device node\n");
	printf("\tm (to map a kernel buffer into user space)\n");
	printf("\tC (to clear data registers)\n");
	printf("\tR (to enable/disable to read from data registers)\n");
	printf("\tW (to enable/disable to write to data registers)\n");
	printf("\ts (to get status of device)\n");
	printf("\tq (to quit the application)\n");
	while (1) {
		printf("Enter your option: ");
		scanf(" %c", &option);

		switch (option) {
			case 'o':
				if (fd < 0)
					fd = open_chardev();
				else
					printf("%s has already opened\n", DEVICE_NODE);
				break;
			case 'c':
				if (fd > -1)
					close_chardev(fd);
				else
					printf("%s has not opened yet! Can not close\n", DEVICE_NODE);
				fd = -1;
				break;
			case 'm':
				mem_map_chardev();
				break;
			case 'r':
				read_data_chardev();
				break;
			case 'w':
				write_data_chardev();
				break;
			case 'C':
				clear_data_chardev();
				break;
			case 'R':
				control_read_chardev();
				break;
			case 'W':
				control_write_chardev();
				break;
			case 's':
				get_status_chardev();
				break;
			case 'q':
				if (fd > -1)
					close_chardev(fd);
				printf("Quit the application. Good bye!\n");
				return 0;
			default:
				printf("invalid option %c\n", option);
				break;
		}
	};
}
