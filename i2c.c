#include "i2c.h"

void i2c_init(void) {
    TWBR = 3;
    TWSR = 1;
}

int i2c_read_byte(uint8_t address, uint8_t regAdd, uint8_t *data) {
    // send start condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // wait to see of start condition has been transmitted
    while (!(TWCR & (1 << TWINT))) {
    }
    // check if status is no "start"
    if ((TWSR & 0xF8) != 0x08) {
        return -1;
    }

    // slave address to write to
    TWDR = (address << 1) | TW_WRITE;
    // send device address
    TWCR = (1 << TWINT) | (1 << TWEN);
    //Wait for ACK/NACK
    while (!(TWCR & (1 << TWINT))) {
    }

    // check for aknowledgement
    if ((TWSR & 0xF8) != 0x18) {
        if ((TWSR & 0xF8) == 0x20) {
            return -3;
        }
        return -2;
    }

    // set device register address
    TWDR = regAdd;
    //send device register address
    TWCR = (1 << TWINT) | (1 << TWEN);

    // wait to see of start condition has been transmitted
    while (!(TWCR & (1 << TWINT))) {
    }

    // check for aknowledgement
    if (TW_STATUS != TW_MT_DATA_ACK) {
        return -3;
    }

    // repeated start
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    // wait to see of start condition has been transmitted
    while (!(TWCR & (1 << TWINT))) {
    }

    // check if status is no "repeated start"
    if (TW_STATUS != TW_REP_START) {
        return -4;
    }

    // slave address to read from
    TWDR = (0x68 << 1) | TW_READ;
    TWCR = (1 << TWINT) | (1 << TWEN);

    // wait for address to be sent
    while (!(TWCR & (1 << TWINT))) {
    }
    // check for ack
    if (TW_STATUS != TW_MR_SLA_ACK) {
        return -5;
    }

    //Send address, NACK after data recieved to end transmission
    TWCR = (1 << TWINT) | (1 << TWEN);
    // wait for data to be sent
    while (!(TWCR & (1 << TWINT))) {}
    // read data
    *data = TWDR;

    // transmit STOP
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    return 0;
}


//writes data value to register with address regAdd
//return data value written on success, -1 on failure
int i2c_write_byte(uint8_t address, uint8_t regAdd, uint8_t data) {

    // send start condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // wait to see of start condition has been transmitted
    while (!(TWCR & (1 << TWINT))) {
    }

    // check if status is no "start"
    if ((TWSR & 0xF8) != TW_START) {
        return -1; // error has occured
    }

    // slave address to write to
    TWDR = (address << 1) | TW_WRITE;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {
    }

    if ((TWSR & 0xF8) != TW_MT_SLA_ACK) {
        return -1;
    }

    // register to write to
    TWDR = regAdd;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {
    }
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) {
        return -1;
    }


    // write data to register
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {
    }
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK) {
        return -1;
    }

    // transmit STOP
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

    return data;
}
