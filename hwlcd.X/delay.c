#include "delay.h"
#include <xc.h>

static unsigned short multiplier = 1;

#ifdef XTAL_20_MHZ
void delay_with_timer_init(void)
{
    T1CONbits.TMR1CS = 1; // instruction clock fosc = 20 MHz
    T1CONbits.T1CKPS = 1; // 1:2 prescaler : timer clock is 10 MHz => 0.1 us
    T1CONbits.TMR1ON = 0;
    multiplier = 10;
}
#else
void delay_with_timer_init(void)
{
    T1CONbits.TMR1CS = 0; // instruction clock (fosc / 4) = 8MHz
    T1CONbits.T1CKPS = 3; // 1:8 prescaler : timer clock is 1Mhz => 1 us
    T1CONbits.TMR1ON = 0;
}
#endif

void delay_with_timer_us(unsigned short useconds) {
    for (int i = 0; i < multiplier; ++i) {
        T1CONbits.TMR1ON = 0;
        TMR1 = 0xFFFFu - useconds;
        PIR1bits.TMR1IF = 0;
        T1CONbits.TMR1ON = 1;
        while (!PIR1bits.TMR1IF);
        T1CONbits.TMR1ON = 0;
    }
}
