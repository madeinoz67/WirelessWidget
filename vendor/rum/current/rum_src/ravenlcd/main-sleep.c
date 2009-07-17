#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


/*
 * Advanced Electrical Signal Forwarding over Cu-Based Interconnect System
 *
 * Allows remote control of silicon-based photon generation system, as well
 * as routing physical user interaction back to the radio-control microprocessor.
 *
 */


/** \name Port defininition for the joystick Enter button. */
/** \{ */
#define ENTER_PORT      PINE
#define ENTER_PUR       PORTE
#define ENTER_DDR       DDRE
#define ENTER_PIN       PINE2
/** \} */

int main(void)
{
    
    /* Enable LED */
    DDRB |= 1<<PB7;
    
    /* User input */
    ENTER_DDR &= ~(1<<ENTER_PIN);
    
    /* Pull-up on */
    ENTER_PUR |= (1<<ENTER_PIN);	
    
    /* Turn on output to M1284p */
    DDRE |= 1<<1;
    
    
    while(1) {
	
        /* If user is pressing button - put port pin high */
        if(ENTER_PORT & (1<<ENTER_PIN)) {
            PORTE &= ~(1<<1); //Button not pushed
        } else {
            PORTE |= 1<<1; //Button pushed
        }	
	
	
        /* If port pin high, turn on LED */
        if (PINE & 1<<0) {
            PORTB &= ~(1<<PB7); //Led ON
         } else {
            PORTB |= 1<<PB7; //Led OFF
        }
	

        // Shut down everything for sleep
        
        // Pull all unused port pins high
        DDRA  = 0x00;
        PORTA = 0xff;
        // PB0 = CS for eeprom (out hi)
        // PB1 = SCK (out lo)
        // PB2 = SI (out lo
        // PB3 = SO (input, pullup)
        // PB4 = reset for eeprom (out hi)
        // PB5 = audio out (output, low)
        // PB6 = relay (out low)
        // PB7 = LED (out hi)
        DDRB  = 0xf7;
        PORTB = 0x89;
        DDRC  = 0x00;
        PORTC = 0xff;
        DDRD  = 0x00;
        PORTD = 0xff;
        DDRE  = 0x00;
        PORTE = 0xff;
        // PF0 = sig in (out lo)
        // PF1 = button (in hi-z)
        // PF2 = ext supply sig (in hi-z)
        // PF3 = volt sig (in hi-z)
        // PF4 = temp (in hi-z)
        // PF5 = nuthin (in hi-z)
        // PF6 = temp pwr (out lo)
        // PF7 = nuthin (in hi-z)
        DDRF  = 0x41;
        PORTF = 0x00;
        DDRG  = 0x00;
        PORTG = 0xff;
        DDRH  = 0x00;
        PORTH = 0xff;
        DDRJ  = 0x00;
        PORTJ = 0xff;

        // Take EEPROM out of reset        
        PORTB = 0x99;

        
/*         MCUCR |= (1 << JTD);              // Turn off JTAG interface */
/*         MCUCR |= (1 << JTD); */
/*         MCUCR |= (1 << JTD); */
/*         MCUCR |= (1 << JTD); */
/*         MCUCR |= (1 << JTD); */
/*         MCUCR |= (1 << JTD); */
/*         MCUCR |= (1 << JTD); */
        

        // Shutdown all peripherals
        PRR = 0xff;
       
        for(;;)
        {
            // Go to hell, I mean permanent sleep
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            sleep_enable();
            cli();
            sleep_cpu();
            sleep_disable();
        }
    }
    
    
    return 0;	
}


