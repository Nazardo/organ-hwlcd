#include "i2c.h"
#include <xc.h>

void inline ssp1if_wait_and_clear() {
    while (!PIR1bits.SSP1IF);
    PIR1bits.SSP1IF = 0;
}

void i2c_write_byte(unsigned char busAddress, unsigned char dataByte) {
    SSP1CON2bits.SEN = 1;
    ssp1if_wait_and_clear();
    // TODO Is a wait needed here?
    SSP1BUF = busAddress & 0xFEu;
    ssp1if_wait_and_clear();
    SSP1BUF = dataByte;
    ssp1if_wait_and_clear();
    SSP1CON2bits.PEN = 1;
    ssp1if_wait_and_clear();
}

unsigned char i2c_read_byte(unsigned char busAddress) {
    SSP1CON2bits.SEN = 1;
    ssp1if_wait_and_clear();
    SSP1BUF = busAddress | 0x01u;
    // ignore ACK
    ssp1if_wait_and_clear();
    SSP1CON2bits.RCEN = 1;
    ssp1if_wait_and_clear();
    unsigned char data = SSP1BUF;
    SSP1STATbits.BF = 0;
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
    ssp1if_wait_and_clear();
    SSP1CON2bits.PEN = 1;
    ssp1if_wait_and_clear();
    return data;
}