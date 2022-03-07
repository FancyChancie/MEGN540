#include "Battery_Monitor.h"

/*
NOT SURE WHAT TO DO HERE!!!!
*/
static const float BITS_TO_BATTERY_VOLTS = 1023;

/**
 * Function Battery_Monitor_Init initializes the Battery Monitor to record the current battery voltages.
 */
void Battery_Monitor_Init()
{
    //// VBAT pin PF6/ADC6 (aka A1) ////
    // Enable ADC (Sec. 24.9.2)
    ADCSRA |= (1 << ADEN);

    // Set prescalor to 128 (Sec 24.9.2)
    // Need within 50kHz-200kHz range: 16MHz/128 = 125kHz
    ADCSRA |= (1 << ADPS2);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS0);

    // Set internal 2.56V reference with external capacitor on AREF pin (Sec.24.9.1)
    ADMUX |= (1 << REFS1);
    ADMUX |= (1 << REFS0);

    // Enable ADC6 as input (Sec.24.9.1)
    ADMUX |= (1 << MUX2);
    ADMUX |= (1 << MUX1);
}

/**
 * Function Battery_Voltage initiates the A/D measurement and returns the result for the battery voltage.
 */
float Battery_Voltage()
{
    // A Union to assist with reading the LSB and MSB in the 16 bit register
    union { struct {uint8_t LSB; uint8_t MSB; } split; uint16_t value;} data;
    
    // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON) (Sec. 14.2)
    char SREG_copy = SREG;
        // Disable global interrupts
        // __disable_interrupt();
        cli();
        // Start ADC conversion (resets itself to 0 after conversion complete) (Sec. 24.9.2)
        ADCSRA |= (1 << ADSC);
        
        while(ADCSRA & (1<<ADSC)){
            
        }

        // Save ADC Low byte into union stuct (Sec. 24.9.3)
            data.split.LSB = ADCL;
            // Save ADC high byte into union struct
            data.split.MSB = ADCH;
    // Restore interrupt settings
    SREG = SREG_copy;

    return (float) data.value * BITS_TO_BATTERY_VOLTS;
}
