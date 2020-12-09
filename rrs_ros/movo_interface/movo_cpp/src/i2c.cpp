#include "ros/ros.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <unistd.h>
//#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <syslog.h>		/* Syslog functionality */
#include <tr1cpp/i2c.h>

namespace tr1cpp
{
	I2C::I2C(int bus, int address) {
		_i2cbus = bus;
		_i2caddr = address;
		snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
		openfd();
	}

	I2C::~I2C() {
		close(fd);
	}

	uint8_t I2C::readBytes(uint8_t registerNumber, uint8_t bufferSize, uint16_t &position) {
		if (fd != -1) {
			uint8_t buff[bufferSize];

			uint8_t writeBufferSize = 1;
			uint8_t writeBuffer[writeBufferSize];
			writeBuffer[0] = registerNumber;

			if (write(fd, writeBuffer, writeBufferSize) != writeBufferSize) {
				ROS_ERROR("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", _i2caddr, registerNumber, errno);
				return (-1);
			} else {
				if (read(fd, buff, bufferSize) != bufferSize) {
					ROS_ERROR("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", _i2caddr, registerNumber, errno);
					return (-1);
				} else {
					position = 1;
					for (int i = 0; i < bufferSize; i++) {
						int shift = pow(256, abs(i + 1 - bufferSize));
						position = position + (buff[i] * shift);
						if (registerNumber == 2) {
							//ROS_INFO("%i: %i", i, buff[i]);
						}
					}
					uint32_t excessK = pow(256, bufferSize)/2;
					position -= excessK;
					return (1);
				}
			}
		} else {
			ROS_ERROR("Device File not available. Aborting read");
			return (-1);
		}
	}

	uint8_t I2C::writeData(uint8_t registerNumber, uint8_t data[4]) {
		if (fd != -1) {
			uint8_t buff[5];
			buff[0] = registerNumber;
			buff[1] = data[0];
			buff[2] = data[1];
			buff[3] = data[2];
			buff[4] = data[3];

			int result = write(fd, buff, sizeof(buff));
			if (result != sizeof(buff)) {
				ROS_ERROR("%s. Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", strerror(errno), _i2caddr, registerNumber, errno);
				return (-1);
			} else {
				//ROS_INFO("Wrote to I2C Slave 0x%x @ register 0x%x", _i2caddr, address);
				return (-1);
			}
		} else {
			ROS_ERROR("Device File not available. Aborting write");
			return (-1);
		}
		return 0;
	}

	void I2C::openfd() {
		if ((fd = open(busfile, O_RDWR)) < 0) {
			ROS_ERROR("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus, errno);
		}
		if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
			ROS_ERROR("I2C slave %d failed [openfd():ioctl %d]", _i2caddr, errno);
		}
	}
}
