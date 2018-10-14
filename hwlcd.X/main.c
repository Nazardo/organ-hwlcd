// PIC16F18346 Configuration Bit Settings

// CONFIG1
#pragma config FEXTOSC = HS     // FEXTOSC External Oscillator mode Selection bits (HS (crystal oscillator) above 4 MHz)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

#include <xc.h>
#include "delay.h"
#include "lcd_pcf8574.h"

#define _XTAL_FREQ 32000000

void init();

#define BUFFER_SIZE 200u

#define NUM_LCD_LINES 4
#define NUM_LCD_STORED_LINES 10

#define MIDI_STATUS_SYSEX 0xF0
#define HAUPTWERK_SYSEX_MANUFACTURER_ID 0x7D
#define HAUPTWERK_SYSEX_MESSAGE_TYPE_LCD 0x01
#define MIDI_SYSEX_END_BYTE 0xF7

#define AdvancePointer(PTR) PTR = ((PTR+1u) % BUFFER_SIZE)

unsigned char buffer[BUFFER_SIZE];
bit readingFERR = 0;
bit startOfSysexFound = 0;
unsigned char ptrBuffer_ToRead = 0;
unsigned char ptrBuffer_ConfirmedData = 0;
unsigned char ptrBuffer_ReceivingData = 0;
unsigned char lcdCurrentOffset = 0;

unsigned char lcdLines[NUM_LCD_STORED_LINES][21];

void interrupt isr() {
    while (PIR1bits.RCIF) {
        if (RC1STAbits.OERR) {
            RB7 = 1;
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;
            return;
        }
        readingFERR = RC1STAbits.FERR;
        unsigned char data = RC1REG; // reading RC1REG clears FERR and RCIF
        if (readingFERR) {
            RB7 = 1;
            startOfSysexFound = 0;
            ptrBuffer_ReceivingData = ptrBuffer_ConfirmedData;
        } else {
            //buffer[ptrBuffer_ReceivingData] = data;
            //AdvancePointer(ptrBuffer_ReceivingData);
            //ptrBuffer_ConfirmedData = ptrBuffer_ReceivingData;
            if (data == MIDI_STATUS_SYSEX) {
                startOfSysexFound = 1;
                ptrBuffer_ReceivingData = ptrBuffer_ConfirmedData;
            } else if (startOfSysexFound) {
                buffer[ptrBuffer_ReceivingData] = data;
                AdvancePointer(ptrBuffer_ReceivingData);
            }
            if (data == MIDI_SYSEX_END_BYTE && startOfSysexFound) {
                startOfSysexFound = 0;
                ptrBuffer_ConfirmedData = ptrBuffer_ReceivingData;
            }
        }
    }
}

void scrollLcdDown() {
    unsigned char maxOffset = NUM_LCD_STORED_LINES - NUM_LCD_LINES;
    if (lcdCurrentOffset < maxOffset) {
        lcdCurrentOffset++;
    }
}
void scrollLcdUp() {
    if (lcdCurrentOffset > 0) {
        lcdCurrentOffset--;
    }
}
void updateLcd();
void copyFromReadBuffer(unsigned char* output, unsigned char index, unsigned char size);

void main(void) {
    init();

    LCDclear();

    LCDsetCursor(0, 3);
    LCD_Write_Str("Test with MIDI");

    LCDsetCursor(0, 0);

    while (1) {
        /*while (ptrBuffer_ToRead != ptrBuffer_ConfirmedData) {
            unsigned char data = buffer[ptrBuffer_ToRead];
            unsigned char toPrint = data >> 4;
            if (toPrint < 10) {
                toPrint += '0';
            } else {
                toPrint = toPrint - 10 + 'A';
            }
            LCD_Write_Char(toPrint);
            toPrint = data & 0x0F;
            if (toPrint < 10) {
                toPrint += '0';
            } else {
                toPrint = toPrint - 10 + 'A';
            }
            LCD_Write_Char(toPrint);
            AdvancePointer(ptrBuffer_ToRead);
        }*/
        if (ptrBuffer_ToRead != ptrBuffer_ConfirmedData) {
            // New data available
            if (buffer[ptrBuffer_ToRead] == HAUPTWERK_SYSEX_MANUFACTURER_ID) {
                AdvancePointer(ptrBuffer_ToRead);
                if (buffer[ptrBuffer_ToRead] == HAUPTWERK_SYSEX_MESSAGE_TYPE_LCD) {
                    AdvancePointer(ptrBuffer_ToRead);
                    unsigned char id_lsb = buffer[ptrBuffer_ToRead];
                    AdvancePointer(ptrBuffer_ToRead);
                    unsigned char id_msb = buffer[ptrBuffer_ToRead];
                    AdvancePointer(ptrBuffer_ToRead);
                    AdvancePointer(ptrBuffer_ToRead);
                    unsigned char offset = id_lsb * 2u;
                    copyFromReadBuffer(lcdLines[offset], 2, 16);
                    copyFromReadBuffer(lcdLines[offset + 1u], 2, 16);
                    lcdLines[offset][20] = 1;
                    lcdLines[offset + 1u][20] = 1;
                }
            }
            // Read until end of sysex
            while (buffer[ptrBuffer_ToRead] != MIDI_SYSEX_END_BYTE &&
                   ptrBuffer_ToRead != ptrBuffer_ConfirmedData) {
                AdvancePointer(ptrBuffer_ToRead);
            }
            if (buffer[ptrBuffer_ToRead] == MIDI_SYSEX_END_BYTE) {
                AdvancePointer(ptrBuffer_ToRead);
            }
        }
        updateLcd();
    }
    return;
}

void init() {
    LATA = 0;
    LATB = 0;
    LATC = 0;
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    TRISA = 0;
    TRISB = 0b01110000;
    WPUC = 0xFF;
    TRISC = 0;

    RXPPS = 0x0D; // RB5 is already default PPS for RX input

    // I2C INTERFACE CONFIGURATION
    SSP1DATPPS = 0b00001100; // SDA1 input is RB4
    RB4PPS = 0b00011001; // SDA1 output is RB4
    SSP1CLKPPS = 0b000001110; // SCL1 input is RB6
    RB6PPS = 0b00011000; // SCL1 output is RB6

    SSP1ADD = 0x4F; // 100kHz Baud rate
    SSP1CON1bits.SSPM = 0x08; // I2C Master Mode
    SSP1CON1bits.SSPEN = 1; // Enable SSP1

    delay_init_32Mhz();
    LCD_Init();

    // Clear buffers
    for (unsigned char i = 0; i < 4; ++i) {
        for (unsigned char j = 0; j < 20; ++j) {
            lcdLines[i][j] = ' ';
        }
        lcdLines[i][20] = 0;
    }

    // INPUT MIDI INTERFACE CONFIGURATION (EUSART)
    // Set baud-rate = fosc / [16 * (n+1)] = 32M / 16*40 = 31.25k
    SP1BRGH = 0;
    SP1BRGL = 63;
    TX1STAbits.BRGH = 1;
    BAUD1CONbits.ABDEN = 0;
    BAUD1CONbits.BRG16 = 0;
    // BAUD1CON is OK with default

    TX1STAbits.SYNC = 0;
    RC1STAbits.SPEN = 1;
    PIE1bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    RC1STAbits.CREN = 1;
}

void copyFromReadBuffer(unsigned char* output, unsigned char index, unsigned char size) {
    for (unsigned char i = 0; i < size; ++i) {
        output[i + index] = buffer[ptrBuffer_ToRead];
        AdvancePointer(ptrBuffer_ToRead);
    }
}

void updateLcd() {
    for (unsigned char i = 0; i < 4; ++i) {
        unsigned char row = (unsigned char)lcdCurrentOffset + i;
        if (lcdLines[row][20]) {
            lcdLines[row][20] = 0;
            LCDsetCursor(0, i);
            for (unsigned char j = 0; j < 20; ++j) {
                LCDdataWrite(lcdLines[row][j]);
            }
        }
    }
}
