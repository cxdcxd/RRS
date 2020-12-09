#ifndef I2C_H
#define I2C_H
#include <inttypes.h>

#define BUFFER_SIZE 1

namespace tr1cpp
{
	class I2C
	{
		public:
			I2C(int, int);
			virtual ~I2C();
			uint8_t dataBuffer[BUFFER_SIZE];
			uint8_t readBytes(uint8_t registerNumber, uint8_t bufferSize, uint16_t &position);
			uint8_t writeData(uint8_t registerNumber, uint8_t data[4]);
		private:
			int _i2caddr;
			int _i2cbus;
			void openfd();
			char busfile[64];
			int fd;
	};
}

#endif
