#include "MotorPWM.h"

static bool motor_enabled = false;

/**
 * Function MotorPWM_Init initializes the motor PWM on Timer 1 for PWM based voltage control of the motors.
 * The Motor PWM system shall initialize in the disabled state for safety reasons. You should specifically enable
 * Motor PWM outputs only as necessary.
 * @param [uint16_t] MAX_PWM is the maximum PWM value to use. This controls the PWM frequency.
 */
void Motor_PWM_Init( uint16_t MAX_PWM ) {
    // Disable motors by default
    Motor_PWM_Enable(false);

    // Set waveform generation mode to 8 to use ICR as TOP value
    TCCR1B |= (1 << WGM13);
    // Enable clock source with no prescaler
    TCCR1B |= (1 << CS10);

    PORTB &= ~(1 << PB5);
    PORTB &= ~(1 << PB6);

    // Set ICRI register to Max_PWM
    Set_MAX_Motor_PWM(MAX_PWM);

    Motor_PWM_Left(0);
    Motor_PWM_Right(0);
}

/**
 * Function MotorPWM_Enable enables or disables the motor PWM outputs.
 * @param [bool] enable (true set enable, false set disable)
 */
void Motor_PWM_Enable( bool enable ) {
    if(enable){
        // Enable driver pins as output for PWM speed (Sec. 14.10.2)
        DDRB |= (1 << DDB5); 
        DDRB |= (1 << DDB6); 

        // Enable driver pins as output for motor direction (Sec. 14.10.2)
        DDRB |= (1 << DDB1); 
        DDRB |= (1 << DDB2); 

        // Set Timer1 to clear on compare match (Channel A & B) (Sec. 14.10.1)
        TCCR1A |= (1 << COM1A1);
        TCCR1A |= (1 << COM1B1);

        // Set motor enabled flag
        motor_enabled = true;
    }else{
        DDRB &= ~(1 << DDB5); 
        DDRB &= ~(1 << DDB6); 

        DDRB &= ~(1 << DDB1); 
        DDRB &= ~(1 << DDB2); 

        TCCR1A &= ~(1 << COM1A1);
        TCCR1A &= ~(1 << COM1B1);

        motor_enabled = false;
    }
}

/**
 * Function Is_Motor_PWM_Enabled returns if the motor PWM is enabled for output.
 * @param [bool] true if enabled, false if disabled
 */
bool Is_Motor_PWM_Enabled() {
    return motor_enabled;
}

/**
 * Function Motor_PWM_Left sets the PWM duty cycle for the left motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Left( int16_t pwm ) {
    // (Sec. 14.10.10)
    OCR1BL = ((uint16_t) pwm) & 0xFF;
    OCR1BH = (((uint16_t) pwm) >> 8) & 0xFF;
}

/**
 * Function Motor_PWM_Right sets the PWM duty cycle for the right motor.
 * @return [int32_t] The count number.
 */
void Motor_PWM_Right( int16_t pwm ) {
    // (Sec. 14.10.9)
    OCR1AL = ((uint16_t) pwm) & 0xFF;
    OCR1AH = (((uint16_t) pwm) >> 8) & 0xFF;
}

/**
 * Function Get_Motor_PWM_Left returns the current PWM duty cycle for the left motor. If disabled it returns what the
 * PWM duty cycle would be.
 * @return [int16_t] duty-cycle for the left motor's pwm
 */
int16_t Get_Motor_PWM_Left() {
    int16_t duty_cycle;
    duty_cycle = (OCR1BH << 8) | OCR1BL;
    return duty_cycle;
}

/**
 * Function Get_Motor_PWM_Right returns the current PWM duty cycle for the right motor. If disabled it returns what the
 * PWM duty cycle would be.
 * @return [int16_t] duty-cycle for the right motor's pwm
 */
int16_t Get_Motor_PWM_Right() {
    int16_t duty_cycle;
    duty_cycle = (OCR1AH << 8) | OCR1AL;
    return duty_cycle;
}

/**
 * Function Get_MAX_Motor_PWM() returns the PWM count that corresponds to 100 percent duty cycle (all on), this is the
 * same as the value written into ICR1 as (TOP).
 */
uint16_t Get_MAX_Motor_PWM() {
    uint16_t max_motor;
    max_motor = (ICR1H << 8) | ICR1L;
    return max_motor;
}

/**
 * Function Set_MAX_Motor_PWM sets the maximum pwm count. This function sets the timer counts to zero because
 * the ICR1 can cause undesired behaviors if change dynamically below the current counts.  See page 128 of the
 * atmega32U4 datasheat.
 */
void Set_MAX_Motor_PWM( uint16_t MAX_PWM ) {
    // Reset timer1 counter
    TCNT1 = 0;
    // (Sec. 14.10.15)
    ICR1L = MAX_PWM & 0xFF;
    ICR1H = (MAX_PWM >> 8) & 0xFF;
}
