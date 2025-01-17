#include "Encoder.h"

/**
* Internal counters for the Interrupts to increment or decrement as necessary.
*/
static volatile bool _last_right_A;  // Static limits it's use to this file
static volatile bool _last_right_B;  // Static limits it's use to this file

static volatile bool _last_left_A;   // Static limits it's use to this file
static volatile bool _last_left_B;   // Static limits it's use to this file
static volatile bool _last_left_XOR; // Necessary to check if PB4 triggered the ISR or not

static volatile int32_t _left_counts;   // Static limits it's use to this file
static volatile int32_t _right_counts;  // Static limits it's use to this file

/** Helper Funcions for Accessing Bit Information */
// Hint, use avr's bit_is_set function to help (found in avr/sfr_degs.h)
static inline bool Right_XOR() { return bit_is_set(PINE,PE6); } // (Port E Input Pins Address, INT6 <- Sec. 10.4.13)
static inline bool Right_B()   { return bit_is_set(PINF,PF0); } // (Port F Input Pins Address, ADC0 <- Sec. 10.4.16)
static inline bool Right_A()   { return (Right_XOR()^Right_B()); }

static inline bool Left_XOR() { return bit_is_set(PINB,PB4); } // (Port B Input Pins Address, PCINT4 <- Sec. 10.4.4)
static inline bool Left_B()   { return bit_is_set(PINE,PE2); } // (Port E Input Pins Address, PE2 (HWB) <- Sec. 10.4.13)
static inline bool Left_A()   { return (Left_XOR()^Left_B()); }
/**
 * Function Encoders_Init initializes the encoders, sets up the pin change interrupts, and zeros the initial encoder
 * counts.
 */
void Encoders_Init()
{
    /* 
    Left encoder uses PB4 and PE2 pins as digital inputs. External interrupt PCINT4 is necessary to detect
    the change in XOR flag. You'll need to see Section 11.1.5 - 11.1.7 for setup and use.
    Note that the PCINT interrupt is trigered by any PCINT pin. In the ISR you should check to make sure
    the interrupt triggered is the one you intend on processing.

    Right encoder uses PE6 adn PF0 as digital inputs. External interrupt INT6 is necessary to detect
    the changes in XOR flag. You'll need to see Sections 11.1.2-11.1.4 for setup and use.
    You'll use the INT6_vect ISR flag.
    */

    // Initialize static file variables. 
    _last_right_A  = 0;
    _last_right_B  = 0;

    _last_left_A   = 0;
    _last_left_B   = 0;
    _last_left_XOR = 0;

    _left_counts   = 0;
    _right_counts  = 0;


    //// Left encoder (PCINT4, pins PB4 & PE2) ////
    // Set pins PB4 & PE2 as digital inputs (0=input, 1=output) (Sec. 10.2.1)
    DDRB   &= (0 << DDB4);
    DDRE   &= (0 << DDE2);
    // Set PB4 & PE2 pins as input with pull-up resistor deactivated (0=deactivated, 1=activated) (Sec. 10.2.1)
    PORTB  &= (1 << PORTB4);
    PORTE  &= (1 << PORTE2);
    // Enable interrupt trigger request for PCINT4 (Sec. 11.1.5)
    PCICR  |= (1 << PCIE0);
    // Enable external interrupt on any change of PCINT4 (Sec. 11.1.7)
    PCMSK0 |= (1 << PCINT4);

    //// Right Encoder (INT6, pins PE6 & PF0) ////
    // Set pins PE6 & PF0 as digial inputs (0=input, 1=output) (Sec. 10.2.1)
    DDRE  &= (1 << DDE6);
    DDRF  &= (1 << DDF0);
    // Set PE6 & PF0 pins as input with pull-up resistor deactivated (0=deactivated, 1=activated) (Sec. 10.2.1)
    PORTE &= (1 << PORTE6);
    PORTF &= (1 << PORTF0);
    
    // Need to clear Interrupt Enable bit of EIMSK register when setting ISC161/ISC160 (Note in Sec 11.1.2)
    EIMSK &= (1 << INT6);

    // Enable external interrupt on any logical change of INT6 (Sec. 11.1.2)
    EICRB |= (1 << ISC60);
    //EICRB &= (1 << ISC61);

    // Enable interrupt trigger request for INT6 (Sec. 11.1.3)
    EIMSK |= (1 << INT6);
}


/**
 * Function Counts_Left returns the number of counts from the left encoder.
 * @return [int32_t] The count number.
 */
int32_t Counts_Left()
{
    // Note: Interrupts can trigger during a function call and an int32 requires
    // multiple clock cycles to read/save. You may want to stop interrupts, copy the value,
    // and re-enable interrupts to prevent this from corrupting your read/write.
    
    // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON)) (Sec. 14.2)
    char SREG_copy = SREG;
        // Disable global interrupts
        // __disable_interrupt();
        cli();
        // store left encoder counts
        int32_t L_Count = _left_counts;
    // Restore interrupt settings
    SREG = SREG_copy;

    return L_Count;
}

/**
 * Function Counts_Right returns the number of counts from the right encoder.
 * @return [int32_t] The count number.
 */
int32_t Counts_Right()
{
    // Note: Interrupts can trigger during a function call and an int32 requires
    // multiple clock cycles to read/save. You may want to stop interrupts, copy the value,
    // and re-enable interrupts to prevent this from corrupting your read/write.

    // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON) (Sec. 14.2)
    char SREG_copy = SREG;
        // Disable global interrupts
        // __disable_interrupt();
        cli();
        // store left encoder counts
        int32_t R_Count = _right_counts;
    // Restore interrupt settings
    SREG = SREG_copy;

    return R_Count;
}

/**
 * Function Rad_Left returns the number of radians for the left encoder.
 * @return [float] Encoder angle in radians
 */
float Rad_Left()
{
    // Store counts per revolution (Sec. 3.4 of Zumo 32U4 datasheet)
    float CPR = 909.7;
    // Convert to radians and return value
    float Rad_Left = (float) Counts_Left() * ((3.14159265359 * 4)/CPR);
    return Rad_Left;
}

/**
 * Function Rad_Right returns the number of radians for the left encoder.
 * @return [float] Encoder angle in radians
 */
float Rad_Right()
{
    // Store counts per revolution (Sec. 3.4 of Zumo 32U4 datasheet)
    float CPR = 909.7;
    // Convert to radians and return value
    float Rad_Right = (float) Counts_Right() * ((3.14159265359 * 4)/CPR);
    return Rad_Right;
}

/**
 * Interrupt Service Routine for the left Encoder. Note: May need to check that it is actually PCINT4 that triggered, as
 * the Pin Change Interrupts can trigger for multiple pins.
 * @param found in /usr/lib/avr/include/avr/iom32u4.h
 * @return
 */
ISR(PCINT0_vect)
{
    // Check to see if movement has happened on left side
    if(Left_XOR() != _last_left_A){
        // Get counts on left (see Lecture 14 class notes)
        _left_counts += (Left_A() ^ _last_left_B) - (_last_left_A ^ Left_B());

        // Update previous values for left encoder tracking
        _last_left_A   = Left_A();
        _last_left_B   = Left_B();
        _last_left_XOR = Left_XOR();

        return;
    }
    return;
}


/**
 * Interrupt Service Routine for the right Encoder.
 * @param found in /usr/lib/avr/include/avr/iom32u4.h
 * @return
 */
ISR(INT6_vect)
{
    // Check to see if movement has happened on right side
    if(Right_XOR() != _last_right_A){
        // Get counts on right (see Lecture 14 class notes)
        _right_counts += (Right_A() ^ _last_right_B) - (_last_right_A ^ Right_B());

        // Update previous values for left encoder tracking
        _last_right_A   = Right_A();
        _last_right_B   = Right_B();
        //_last_right_XOR = Right_XOR();
        
        return;
    }
    return;
}
