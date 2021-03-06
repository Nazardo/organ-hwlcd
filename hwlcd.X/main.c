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

// The LCD module is a 20x4, but Hauptwerk only handles strings
// of 16 characters, so we center them in the middle of each 20 characters
// LCD line.

unsigned char lcdLines[NUM_LCD_STORED_LINES][20] = {
    { ' ',' ','H','a','u','p','t','w','e','r','k',' ',' ',' ',' ','L','C','D',' ',' ' },
    { ' ',' ','v',' ','1','.','0',' ',' ',' ',' ','2','0','1','9',' ','M','L',' ',' ' },
    { ' ',' ','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-',' ',' ' },
    { ' ',' ',' ','O','r','g','a','n','o',' ','d','i',' ','N','o','l','e',' ',' ',' ' },
    { ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' },
    { ' ',' ',' ',' ',' ','2','0','1','9','-','0','4','-','2','7',' ',' ',' ',' ',' ' },
    { ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' },
    { ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' },
    { ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' },
    { ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' }
};

#define DEBOUNCE_COUNTER_RESET 3

unsigned char stableInputStatus = 3;
unsigned char previousInputStatus = 3;
unsigned char debounceCounter = DEBOUNCE_COUNTER_RESET;

void scrollLcdDown();
void scrollLcdUp();

void interrupt isr() {
    if (PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        unsigned char currentInputStatus = PORTC & 0x03u;
        if (currentInputStatus != previousInputStatus) {
            previousInputStatus = currentInputStatus;
            debounceCounter = DEBOUNCE_COUNTER_RESET;
        } else if (debounceCounter > 0) {
            debounceCounter--;
            if (debounceCounter == 0) {
                // Detect falling edges (stable[i] = 1 becomes current[i] = 0)
                unsigned char activated = stableInputStatus & (currentInputStatus ^ 0xFFu);
                if (activated & 1) {
                    scrollLcdDown();
                } else if (activated & 2) {
                    scrollLcdUp();
                }
                stableInputStatus = currentInputStatus; // persist into stable
            }
        }
    }
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

void updateLcd();
void copyFromReadBuffer(unsigned char* output, unsigned char index, unsigned char size);

void main(void) {
    init();

    LCDclear();

    while (1) {
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
    TRISC = 0b00000011;

    RXPPS = 0x0D; // RB5 is already default PPS for RX input

    // TIMER0 CONFIGURATION
    // 8bit timer with no pre/post-scaler
    // 256 * 0,125 = 32 ms
    T0CON1 = 0b01000000; // Clock source Fosc /4
    TMR0H = 0xFF;
    TMR0L = 0;
    T0CON0bits.T0EN = 1; // Enable
    PIE0bits.TMR0IE = 1;

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
        unsigned char row = (unsigned char)(lcdCurrentOffset + i);
        LCDsetCursor(0, i);
        for (unsigned char j = 0; j < 20; ++j) {
            LCDdataWrite(lcdLines[row][j]);
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
