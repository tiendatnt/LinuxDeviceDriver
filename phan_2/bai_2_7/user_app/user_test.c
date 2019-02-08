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

#define BUFFER_SIZE 1024
#define DEVICE_NODE "/dev/vchar_dev"

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

int main() {
	int ret = 0;
	char option = 'q';
	int fd = -1;
	printf("Select below options:\n");
	printf("\to (to open a device node)\n");
	printf("\tc (to close the device node)\n");
	printf("\tr (to read data from device node\n");
	printf("\tw (to write data to device node\n");
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
			case 'r':
				read_data_chardev();
				break;
			case 'w':
				write_data_chardev();
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
