#include<iostream>
using namespace std;

#include <inttypes.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <unistd.h>
//#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <syslog.h>		/* Syslog functionality */

#define BUFFER_SIZE 0x04  //4 byte buffer

namespace tr1cpp
{

	class I2C {
		public:
			I2C(int, int);
			virtual ~I2C();
			uint8_t dataBuffer[BUFFER_SIZE];
			uint8_t read_byte(uint8_t);
			uint8_t write_byte(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
		private:
			int _i2caddr;
			int _i2cbus;
			void openfd();
			char busfile[64];
			int fd;
	};

	I2C::I2C(int bus, int address) {
		_i2cbus = bus;
		_i2caddr = address;
		snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
		openfd();
	}

	I2C::~I2C() {
		close(fd);
	}
	//! Read a single byte from I2C Bus
	/*!
	 \param address register address to read from
	 */
	uint8_t I2C::read_byte(uint8_t address) {
		if (fd != -1) {
			uint8_t buff[BUFFER_SIZE];
			buff[0] = address;

			if (write(fd, buff, BUFFER_SIZE) != BUFFER_SIZE) {
				cout << "I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]" << _i2caddr << address << errno;
				return (-1);
			} else {
				if (read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {
					cout << "Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]" << _i2caddr << address << errno;
					return (-1);
				} else {
					return dataBuffer[0];
				}
			}
		} else {
			cout << "Device File not available. Aborting read";
			return (-1);
		}

	}
	//! Write a single byte from a I2C Device
	/*!
	 \param address register address to write to
	 \param data 8 bit data to write
	 */
	uint8_t I2C::write_byte(uint8_t address, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
		syslog(LOG_ERR,"write_byte");
		if (fd != -1) {
			uint8_t buff[5];
			buff[0] = address;
			buff[1] = b1;
			buff[2] = b2;
			buff[3] = b3;
			buff[4] = b4;
			
			int result = write(fd, buff, sizeof(buff));
			if (result != 2) {
				cout << "Failed to write to I2C Slave 0x" << _i2caddr << " @ register 0x" << address << " [write_byte():write " << errno << "] \n";
				return (-1);
			} else {
				cout << "Wrote to I2C Slave 0x" << _i2caddr << " @ register 0x" << address << " [0x" << b1 << "] \n" ;
				return (-1);
			}
		} else {
			cout << "Device File not available. Aborting write \n";
			return (-1);
		}
		return 0;
	}
	//! Open device file for I2C Device
	void I2C::openfd() {
		if ((fd = open(busfile, O_RDWR)) < 0) {
			cout << "Couldn't open I2C Bus " << _i2cbus << " [openfd():open " << errno << "] \n";
		}
		if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
			cout << "I2C slave " << _i2caddr << " failed [openfd():ioctl " << errno << "] \n";
		}
	}
}

int main(int argc, char** argv)
{
	tr1cpp::I2C i2c(1, 0x72);
	uint8_t address = 0x72;
	uint8_t result = i2c.write_byte(address, 1, 100, 1, 255);
	return 0;
}
