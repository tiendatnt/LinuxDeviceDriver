/*
 * ten file: vchar_driver.h
 * tac gia : dat.a3cbq91@gmail.com
 * ngay tao: 9/12/2018
 * mo ta   : chua cac dinh nghia mo ta thiet bi gia lap vchar_dev
 *           vchar_device la mot thiet bi nam tren RAM.
 */

#define REG_SIZE 1 //kich thuoc cua 1 thang ghi la 1 byte (8 bits)
#define NUM_CTRL_REGS 1 //so thanh ghi dieu khien cua thiet bi
#define NUM_STS_REGS 5 //so thanh ghi trang thai cua thiet bi
#define NUM_DATA_REGS 256 //so thanh ghi du lieu cua thiet bi
#define NUM_DEV_REGS (NUM_CTRL_REGS + NUM_STS_REGS + NUM_DATA_REGS) //tong so thanh ghi cua thiet bi

/****************** Mo ta cac thanh ghi trang thai: START ******************/
/*
 * cap thanh ghi [READ_COUNT_H_REG:READ_COUNT_L_REG]:
 * - gia tri luc khoi tao: 0x0000
 * - moi lan doc thanh cong tu cac thanh ghi du lieu, tang them 1 don vi
 */
#define READ_COUNT_H_REG 0
#define READ_COUNT_L_REG 1

/*
 * cap thanh ghi [WRITE_COUNT_H_REG:WRITE_COUNT_L_REG]:
 * - gia tri luc khoi tao: 0x0000
 * - moi lan ghi thanh cong vao cac thanh ghi du lieu, tang them 1 don vi
 */
#define WRITE_COUNT_H_REG 2
#define WRITE_COUNT_L_REG 3

/*
 * thanh ghi DEVICE_STATUS_REG:
 * - gia tri luc khoi tao: 0x03
 * - y nghia cua cac bit:
 *   bit0:
 *       0: cho biet cac thanh ghi du lieu dang khong san sang de doc
 *       1: cho biet cac thanh ghi du lieu da san sang cho viec doc
 *   bit1:
 *       0: cho biet cac thanh ghi du lieu dang khong san sang de ghi
 *       1: cho biet cac thanh ghi du lieu da san sang cho viec ghi
 *   bit2:
 *       0: khi cac thanh ghi du lieu bi xoa, bit nay se duoc thiet lap bang 0
 *       1: khi toan bo cac thanh ghi du lieu da bi ghi, bit nay se duoc thiet lap bang 1
 *   bit3~7: chua dung toi
 */
#define DEVICE_STATUS_REG 4

#define STS_READ_ACCESS_BIT (1 << 0)
#define STS_WRITE_ACCESS_BIT (1 << 1)
#define STS_DATAREGS_OVERFLOW_BIT (1 << 2)

#define READY 1
#define NOT_READY 0
#define OVERFLOW 1
#define NOT_OVERFLOW 0
/****************** Mo ta cac thanh ghi trang thai: END ******************/


/****************** Mo ta cac thanh ghi dieu khien: START ******************/
/*
 * thanh ghi CONTROL_ACCESS_REG:
 * - vai tro: chua cac bit dieu khien kha nang doc/ghi cac thanh ghi du lieu
 * - gia tri luc khoi tao: 0x03
 * - y nghia:
 *   bit0:
 *       0: khong cho phep doc tu cac thanh ghi du lieu
 *       1: cho phep doc tu cac thanh ghi du lieu
 *   bit1:
 *       0: khong cho phep ghi vao cac thanh ghi du lieu
 *       1: cho phep ghi vao cac thanh ghi du lieu
 *   bit2~7: chua dung toi
 */
#define CONTROL_ACCESS_REG 0

#define CTRL_READ_DATA_BIT (1 << 0)
#define CTRL_WRITE_DATA_BIT (1 << 1)

#define ENABLE 1
#define DISABLE 0
/****************** Mo ta cac thanh ghi dieu khien: END ******************/

