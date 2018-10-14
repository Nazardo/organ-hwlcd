#ifndef I2C_H_
#define I2C_H_

void i2c_write_byte(unsigned char busAddress, unsigned char dataByte);
unsigned char i2c_read_byte(unsigned char busAddress);

#endif /* I2C_H_ */
