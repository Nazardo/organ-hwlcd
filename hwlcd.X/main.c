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

void init();

#define BUFFER_SIZE 256u

#define MIDI_STATUS_SYSEX 0xF0
#define HAUPTWERK_SYSEX_MANUFACTURER_ID 0x7D
#define HAUPTWERK_SYSEX_MESSAGE_TYPE_LCD 0x01
#define MIDI_SYSEX_END_BYTE 0xF7

#define AdvancePointer(PTR) PTR = ((PTR+1u) % BUFFER_SIZE)

unsigned char buffer[BUFFER_SIZE];
bit readingFERR = 0;
bit startOfSysexFound = 0;
unsigned char updateLcdMessage = 0;
unsigned char ptrBuffer_ToRead = 0;
unsigned char ptrBuffer_ConfirmedData = 0;
unsigned char ptrBuffer_ReceivingData = 0;

unsigned char lcdUniqueID;
unsigned char lcdLine1[16];
unsigned char lcdLine2[16];

void __interrupt isr() {
    while (PIR1bits.RCIF) {
        readingFERR = RC1STAbits.FERR;
        unsigned char data = RC1REG; // reading RC1REG clears FERR and RCIF
        if (readingFERR) {
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

void writeMessagesToLcd();
void copyFromReadBuffer(unsigned char* output, unsigned char size);

void main(void) {
    init();

    while (1) {
        if (ptrBuffer_ToRead != ptrBuffer_ConfirmedData) {
            // New data available
            if (buffer[ptrBuffer_ToRead] == HAUPTWERK_SYSEX_MANUFACTURER_ID) {
                AdvancePointer(ptrBuffer_ToRead);
                if (buffer[ptrBuffer_ToRead] == HAUPTWERK_SYSEX_MESSAGE_TYPE_LCD) {
                    AdvancePointer(ptrBuffer_ToRead);
                    if (buffer[ptrBuffer_ToRead] == lcdUniqueID) {
                        AdvancePointer(ptrBuffer_ToRead);
                        AdvancePointer(ptrBuffer_ToRead);
                        AdvancePointer(ptrBuffer_ToRead);
                        copyFromReadBuffer(lcdLine1, 16);
                        copyFromReadBuffer(lcdLine2, 16);
                        updateLcdMessage++;
                    }
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
        if (updateLcdMessage) {
            writeMessagesToLcd();
            updateLcdMessage--;
        }
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
    TRISB = 0b00100000;
    TRISC = 1;

    lcdUniqueID = PORTC & 0x7Fu;

    // RXPPS = 0x0D; // RB5 is already default PPS for RX input

    // Set baud-rate = fosc / [64 * (n+1)] = 32M / 64*16 = 31.25k
    SPBRGH = 0;
    SPBRGL = 15;
    // BAUD1CON is OK with default

    TX1STAbits.SYNC = 0;
    RC1STAbits.SPEN = 1;
    PIE1bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    RC1STAbits.CREN = 1;
}

void copyFromReadBuffer(unsigned char* output, unsigned char size) {
    for (unsigned char i = 0; i < size; ++i) {
        output[i] = buffer[ptrBuffer_ToRead];
        AdvancePointer(ptrBuffer_ToRead);
    }
}

void writeMessagesToLcd() {
    // TODO
}
