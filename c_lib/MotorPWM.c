#include "MotorPWM.h"

/**
 * Function MotorPWM_Init initializes the motor PWM on Timer 1 for PWM based voltage control of the motors.
 * The Motor PWM system shall initialize in the disabled state for safety reasons. You should specifically enable
 * Motor PWM outputs only as necessary.
 * @param [uint16_t] MAX_PWM is the maximum PWM value to use. This controls the PWM frequency.
 */
void Motor_PWM_Init( uint16_t MAX_PWM )
{
    // Enable OC1A & OCA1B as outputs (left and right motors)
    // Clear on Timer1 compare match (Channel A & B) (Sec. 14.10.1)
    TCCR1A &= (1 << COM1A1);
    TCCR1A &= (1 << COM1B1);
    // Set PWM, Phase and Frequency correct mode (9) (Tabe 14-4)
    TCCR1A &= (1 << WGM11);
    // Set waveform generation mode to 8 to use ICR as TOP value
    TCCR1B &= (1 << WGM13);
    // Enable clock source with no prescaler
    TCCR1B &= (1 << CS10);
    // Disable motors
    Motor_PWM_Enable(0);
    // Set motor max PWM
    Set_MAX_Motor_PWM(MAX_PWM);
    // As an extra safety set motor PWMs to zero
    Motor_PWM_Left(0);
    Motor_PWM_Right(0);
}

/**
 * Function MotorPWM_Enable enables or disables the motor PWM outputs.
 * @param [bool] enable (true set enable, false set disable)
 */
void Motor_PWM_Enable( bool enable )
{
    // Set PB5 (right motor speed) and PB6 (left motor speed) (Sec. 3.3 of Zumo 32U4)
    if(ebable){
        // Enable drivers by setting data direction (See Sec. 14.10.2)
        DDRB |= (1 << DDB5);
        DDRB |= (1 << DDB6);
    }else{
        DDRB |= (0 << DDB5);
        DDRB |= (0 << DDB6);
    }
}

/**
 * Function Is_Motor_PWM_Enabled returns if the motor PWM is enabled for output.
 * @param [bool] true if enabled, false if disabled
 */
bool Is_Motor_PWM_Enabled()
{
    if(bit_is_set(DDRB, DDB5) && bit_is_set(DDRB, DDB6)){
        return true;
    }else{
        return false;
    }
}

/**
 * Function Motor_PWM_Left sets the PWM duty cycle for the left motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Left( int16_t pwm )
{
    // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON)) (Sec. 14.2)
    char SREG_copy = SREG;
        // Disable global interrupts
        cli();
        if(pwm > ICR1){ // Handle case if PWM command is faster than max PWM
            OCR1B = ICR1;
        }else{
            OCR1B = pwm;
        }
    // Restore interrupt settings
    SREG = SREG_copy;
}

/**
 * Function Motor_PWM_Right sets the PWM duty cycle for the right motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Right( int16_t pwm )
{
    // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON)) (Sec. 14.2)
    char SREG_copy = SREG;
        // Disable global interrupts
        cli();
        if(pwm > ICR1){ // Handle case if PWM command is faster than max PWM
            OCR1A = ICR1;
        }else{
            OCR1A = pwm;
        }
        
    // Restore interrupt settings
    SREG = SREG_copy;
}

/**
 * Function Get_Motor_PWM_Left returns the current PWM duty cycle for the left motor. If disabled it returns what the
 * PWM duty cycle would be.
 * @return [int16_t] duty-cycle for the left motor's pwm
 */
int16_t Get_Motor_PWM_Left()
{
    // // **** DON'T KNOW IF WE NEE TO DISABLE INTERRUPTS HERE OR NOT **** //
    // // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON)) (Sec. 14.2)
    // char SREG_copy = SREG;
    //     // Disable global interrupts
    //     cli();
    //     int16t pwm_val = OCR1B;
    // // Restore interrupt settings
    // SREG = SREG_copy;

    // return (pwm_val/ICR1)*100;
    // Return duty cycle as percent
    return (OCR1B/ICR1) * 100;
}

/**
 * Function Get_Motor_PWM_Right returns the current PWM duty cycle for the right motor. If disabled it returns what the
 * PWM duty cycle would be.
 * @return [int16_t] duty-cycle for the right motor's pwm
 */
int16_t Get_Motor_PWM_Right()
{
    // // **** DON'T KNOW IF WE NEE TO DISABLE INTERRUPTS HERE OR NOT **** //
    // // Store interrupt settings (this is like ATOMIC_BLOCK(ATOMIC_FORCEON)) (Sec. 14.2)
    // char SREG_copy = SREG;
    //     // Disable global interrupts
    //     cli();
    //     int16t pwm_val = OCR1A;
    // // Restore interrupt settings
    // SREG = SREG_copy;

    // return (pwm_val/ICR1)*100;
    // Return duty cycle as percent
    return (OCR1A/ICR1) * 100;
}

/**
 * Function Get_MAX_Motor_PWM() returns the PWM count that corresponds to 100 percent duty cycle (all on), this is the
 * same as the value written into ICR1 as (TOP).
 */
uint16_t Get_MAX_Motor_PWM()
{
    return ICR1;
}

/**
 * Function Set_MAX_Motor_PWM sets the maximum pwm count. This function sets the timer counts to zero because
 * the ICR1 can cause undesired behaviors if change dynamically below the current counts.  See page 128 of the
 * atmega32U4 datasheat.
 */
void Set_MAX_Motor_PWM( uint16_t MAX_PWM )
{
    ICR1 = MAX_PWM;
}