#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <stdlib.h>    // Standard C library


void i2c_init(void);

int i2c_read_byte(uint8_t address, uint8_t regAdd, uint8_t *data);

int i2c_write_byte(uint8_t address, uint8_t regAdd, uint8_t data);
