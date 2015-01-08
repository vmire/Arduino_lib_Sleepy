#include "Sleepy.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/atomic.h>

/// @fn static void Sleepy::powerDown ();
/// Take the ATmega into the deepest possible power down state. Getting out of
/// this state requires setting up the watchdog beforehand, or making sure that
/// suitable interrupts will occur once powered down.
/// Disables the Brown Out Detector (BOD), the A/D converter (ADC), and other
/// peripheral functions such as TWI, SPI, and UART before sleeping, and
/// restores their previous state when back up.

static volatile byte watchdogCounter;

void Sleepy::watchdogInterrupts (char mode) {
    // correct for the fact that WDP3 is *not* in bit position 3!
    if (mode & bit(3))
        mode ^= bit(3) | bit(WDP3);
    // pre-calculate the WDTCSR value, can't do it inside the timed sequence
    // we only generate interrupts, no reset
    byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : 0;
    /* Clear the reset flag. */
	MCUSR &= ~(1<<WDRF);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
		/* In order to change WDE or the prescaler, we need to
		 * set WDCE (This will allow updates for 4 clock cycles).
		 */
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif
        WDTCSR |= (1<<WDCE) | (1<<WDE); // timed sequence
        WDTCSR = wdtcsr;
    }
}
/// @see http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
void Sleepy::powerDown () {
    //byte adcsraSave = ADCSRA;
    //byte prrSave = PRR;
    
    //ADCSRA &= ~ bit(ADEN); // disable the ADC
    //power_all_disable();
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        sleep_enable();
        //sleep_bod_disable(); // can't use this - not in my avr-libc version!
//#ifdef BODSE
        MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
        MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
//#endif
    }
    sleep_cpu();
    sleep_disable();
    
    // re-enable what we disabled
    //PRR = prrSave;
    //ADCSRA = adcsraSave;
}
byte Sleepy::loseSomeTime (word msecs) {
    byte ok = 1;
    word msleft = msecs;
    // only slow down for periods longer than the watchdog granularity
    while (msleft >= 16) {
        char wdp = 0;
		// wdp:
		//   0 :  16ms
		//   1 :  32ms
		//   2 :  64ms
		//   3 : 125ms
		//   4 : 250ms
		//   5 : 0.5s
		//   6 :   1s
		//   7 :   2s
		//   8 :   4s
		//   9 :   8s
        // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
		//
		//On divise par 2 le temps restant, jusqu'à la plus petite granularité, et 9 fois max
		//On incrémente autant de fois le prescaler
		// -> pour les temps élevés, wdp=9
		// -> en dessous de 8s, le nb d'incréments baisse
        for (word m = msleft; m >= 32; m >>= 1){
            if (++wdp >= 9)
                break;
		}
        watchdogCounter = 0;
        watchdogInterrupts(wdp);
        powerDown();
        watchdogInterrupts(-1); // off
        // when interrupted, our best guess is that half the time has passed
        word halfms = 8 << wdp;
        msleft -= halfms;
        if (watchdogCounter == 0) {
            ok = 0; // lost some time, but got interrupted
            break;
        }
        msleft -= halfms;
    }
    // adjust the milli ticks, since we will have missed several
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined (__AVR_ATtiny44__) || defined (__AVR_ATtiny45__)
    extern volatile unsigned long millis_timer_millis;
    millis_timer_millis += msecs - msleft;
#else
    extern volatile unsigned long timer0_millis;
    timer0_millis += msecs - msleft;
#endif
    return ok; // true if we lost approx the time planned
}
void Sleepy::watchdogEvent() {
    ++watchdogCounter;
}
