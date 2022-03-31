/*
         MEGN540 Mechatronics Lab
    Copyright (C) Andrew Petruska, 2021.
       apetruska [at] mines [dot] edu
          www.mechanical.mines.edu
*/

/*
    Copyright (c) 2021 Andrew Petruska at Colorado School of Mines
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "../c_lib/SerialIO.h"
#include "../c_lib/Timing.h"
#include "../c_lib/MEGN540_MessageHandeling.h"
#include "../c_lib/Encoder.h"
#include "../c_lib/Battery_Monitor.h"
#include "../c_lib/Filter.h"
#include "../c_lib/MotorPWM.h"

void Debug()
{
    char debug[5] = "debug";
    usb_send_msg("cc",'!', &debug, sizeof(debug)); // send response
}

/** 
 * Function to re/initialize states
 */
void Initialize()
{
    USB_SetupHardware();     // Initialize USB
    GlobalInterruptEnable(); // Enable Global Interrupts for USB and Timer etc.
    Message_Handling_Init(); // Initialize message handing and all associated flags
    SetupTimer0();           // Initialize timer zero functionality
    Encoders_Init();         // Initalize encoders
    Battery_Monitor_Init();  // Initalize battery monitor
    Motor_PWM_Init(380);     // Initialize motors at TOP PWM of 400
    usb_flush_input_buffer();// Flush buffer
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
    Initialize();

    // Tracking variable for timers
    bool firstLoop  = true;
    bool firstLoopV = true;
    bool firstLoopSysData = true;
    
    // Variable for storing user command
    char command;
    // Build a meaningful structure for storing data about timing
    struct __attribute__((__packed__)) { uint8_t B; float f; } timeData;

    //// Battery voltage stuff ////
    // Low battery message
    struct __attribute__((__packed__)) { char let[7]; float volt; } low_batt_msg = {.let = {'B','A','T',' ','L','O','W'},.volt = 0.1};
    // Power off message
    struct __attribute__((__packed__)) { char let[9]; } pwr_off_msg = {.let = {'P','O','W','E','R',' ','O','F','F'}};
    // Battery check interval (every X seconds)
    float batUpdateInterval = 0.002;
    // Time structure for getting voltage and filtering at intervals
    Time_t batVoltageFilter = GetTime();
    // Time structure for sending battery/power warnings every X seconds
    Time_t battPwrWarnTimer = GetTime();
    // Send warning every X seconds rather than constantly
    float battPwrWarnInterval = 3;
    // Minimum battery voltage (min NiMh batt voltage * num batteries)
    float minBatVoltage = 1.1875 * 4;
    // Lower voltage threshold to warn if power is off
    float offBattVoltage = 0.5;
    // Order & coefficients for Butterworth filter from homework (cut off = 3750Hz (15), sampling = 125000Hz (200), order 4)
    int   order = 4;
    float numerator_coeffs[5]   = {0.00178260999192539,0.00713043996770157,0.0106956599515524,0.00713043996770157,0.00178260999192539}; // Matlab B values
    float denominator_coeffs[5] = {1,-2.77368231754887,3.01903869942386,-1.50476505142532,0.287930429421141}; // Matlab A values
    // Create instance of filter stucture for battery voltage
    Filter_Data_t voltage_Filter;
    // Initalize filter (might be good to add an if to the Initalize() call to reinitalize this too, if needed)
    Filter_Init(&voltage_Filter, numerator_coeffs, denominator_coeffs, order);
    // Initialize variables for saving unfltered & filtered voltage
    float filtered_voltage   = 0;
    float unfiltered_voltage = 0;

    //// System info stuff ////
    // Build a meaningful structure for storing data about timing for sending system info
    struct __attribute__((__packed__)) { float interval; Time_t startTime; Time_t last_trigger_time;} systemDataTime;
    // Build a meaningful structure for storing data about system info
    // struct __attribute__((__packed__)) { float time; int16_t PWM_L; int16_t PWM_R; int16_t Encoder_L; int16_t Encoder_R;} systemData;
    // struct __attribute__((__packed__)) { float time; int16_t PWM_L; int16_t PWM_R;} systemData;
    struct __attribute__((__packed__)) { float time; int16_t Encoder_L; int16_t Encoder_R;} systemData;

    for (;;){
        // USB_Echo_Task();
        USB_Upkeep_Task();
        Message_Handling_Task();

        // [State-machine flag] Restart
        if(MSG_FLAG_Execute(&mf_restart)){
            // Reinitialize everything
            Initialize();
        }
        
        // [State-machine flag] Send time
        if(MSG_FLAG_Execute(&mf_send_time)){
            command = mf_send_time.command;
            timeData.B = mf_send_time.subcommand;
            timeData.f = GetTimeSec(); 

            usb_flush_input_buffer();

            if(mf_send_time.duration <= 0){ 
                usb_send_msg("cBf", command, &timeData, sizeof(timeData)); // send response
                mf_send_time.active = false;
            }else if(SecondsSince(&mf_send_time.last_trigger_time) >= mf_send_time.duration){
                    usb_send_msg("cBf", command, &timeData, sizeof(timeData)); // send response
                    mf_send_time.last_trigger_time = GetTime();
                }
        }

        // [State-machine flag] Time to complete loop
        if(MSG_FLAG_Execute(&mf_loop_timer)){
            static Time_t loopTimeStart;    // struct to store loop time (static so it doesn't get deleted after this inner loop ends)
            
            if(firstLoop){
                loopTimeStart = GetTime();   // fill loopTime struct
            }else{
                command = mf_loop_timer.command;
                timeData.B = mf_loop_timer.subcommand;
                timeData.f = SecondsSince(&loopTimeStart); 

                usb_flush_input_buffer();

                if(mf_loop_timer.duration <= 0){ 
                    usb_send_msg("cBf", command, &timeData, sizeof(timeData)); // send response
                    mf_loop_timer.active = false;
                }else if(SecondsSince(&mf_loop_timer.last_trigger_time) >= mf_loop_timer.duration){
                        usb_send_msg("cBf", command, &timeData, sizeof(timeData)); // send response
                        mf_loop_timer.last_trigger_time = GetTime();
                    }
            }
            firstLoop = !firstLoop; // flip boolean since it is only checking the time of one loop
        }

        // [State-machine flag] Time to send float
        if(MSG_FLAG_Execute(&mf_time_float_send)){
            float gravity = 9.81; // float to send

            char command = usb_msg_get(); // store main command

            // Build a meaningful structure to put subcommand and time in.
            struct __attribute__((__packed__)) { uint8_t B; float f; } data;
            
            data.B = usb_msg_get();  // store subcommand
            
            Time_t floatSendStart = GetTime();   // struct for time to send float
            usb_send_msg("cf", 'g', &gravity, sizeof(gravity)); // send float
            USB_Upkeep_Task(); // wait for send (won't send without this)
            data.f = SecondsSince(&floatSendStart);   // get time since send

            if(mf_time_float_send.duration <= 0){
                usb_send_msg("cBf", command, &data, sizeof(data)); // send response
                mf_time_float_send.active = false;
            }
        }

        // [State-machine flag] Send encoder counts
        if(MSG_FLAG_Execute(&mf_send_encoder)){
            // Build a meaningful structure to put encoder radians in into.
            struct __attribute__((packed)) { float L_Rad; float R_Rad; } encoderData;
            encoderData.L_Rad = Rad_Left();
            encoderData.R_Rad = Rad_Right();

            usb_flush_input_buffer();

            if(mf_send_encoder.duration <= 0){
                usb_send_msg("cff", 'e', &encoderData, sizeof(encoderData)); // send response
                mf_send_encoder.active = false;
            }else if(SecondsSince(&mf_send_encoder.last_trigger_time) >= mf_send_encoder.duration){
                usb_send_msg("cff", 'E', &encoderData, sizeof(encoderData)); // send response
                mf_send_encoder.last_trigger_time = GetTime();
            }
        }

        // Battery voltage measurement/monitor every 2 ms.
        if(SecondsSince(&batVoltageFilter) >= batUpdateInterval){
            // Get unfiltered battery voltage to help the filter smooth out quicker than sending it 0 to begin with
            unfiltered_voltage = Battery_Voltage();
            // Set time battery voltage was retreived
            batVoltageFilter = GetTime();

            if(firstLoopV){
                // Initialize filter with unfiltered_voltage values
                Filter_SetTo(&voltage_Filter,unfiltered_voltage);
                firstLoopV = !firstLoopV; // flip boolean after first battery voltage read
            }
            // Set filtered voltage value
            filtered_voltage = 2.0 * Filter_Value(&voltage_Filter,unfiltered_voltage);

            // Send warning only every X seconds
            if(SecondsSince(&battPwrWarnTimer) >= battPwrWarnInterval){
                // Reset battery warning timer
                battPwrWarnTimer = GetTime();
                // Send warning if battery voltage below minimum voltage but power is NOT off
                if(filtered_voltage <= minBatVoltage && filtered_voltage > offBattVoltage){
                    low_batt_msg.volt = filtered_voltage;
                    usb_send_msg("cf", 'b', &filtered_voltage, sizeof(filtered_voltage));
                    // usb_send_msg("c7sf",'!',&low_batt_msg,sizeof(low_batt_msg));
                    // Disable motors if battery too low
                    Motor_PWM_Enable(false);
                }
                // Send warning of power IS off
                if(filtered_voltage <= offBattVoltage){
                    usb_send_msg("c9s",'!',&pwr_off_msg,sizeof(pwr_off_msg));
                }
            }
        }
        
        // [State-machine flag] Send battery voltage
        if(MSG_FLAG_Execute(&mf_send_voltage)){
            if(mf_send_voltage.duration <= 0){
                usb_send_msg("cf", 'b', &filtered_voltage, sizeof(filtered_voltage));
                mf_send_voltage.active = false;
            }else if(SecondsSince(&mf_send_voltage.last_trigger_time) >= mf_send_voltage.duration){
                usb_send_msg("cf", 'B', &filtered_voltage, sizeof(filtered_voltage));
                mf_send_voltage.last_trigger_time = GetTime();
            }
        }

        // [State-machine flag] Set the motors
        if(MSG_FLAG_Execute(&mf_set_PWM)){
            if(mf_set_PWM.duration <= 0){
                Motor_PWM_Enable(true);

                if(PWM_data.right_PWM < 0){
                    PORTB &= (1 << PB1);
                }
                else{
                    PORTB &= ~(1 << PB1);
                }

                if(PWM_data.left_PWM < 0){
                    PORTB &= (1 << PB2);
                }
                else{
                    PORTB &= ~(1 << PB2);
                }

                Motor_PWM_Left(PWM_data.left_PWM);
                Motor_PWM_Right(PWM_data.right_PWM);

                mf_set_PWM.active = false;

            //// STILL NEEDS WORK ////
            }else if(SecondsSince(&mf_set_PWM.last_trigger_time) >= mf_set_PWM.duration){
                usb_send_msg("cf", 'B', &filtered_voltage, sizeof(filtered_voltage));
                mf_set_PWM.last_trigger_time = GetTime();
            }
        }

        // [State-machine flag] Stop the motors
        if(MSG_FLAG_Execute(&mf_stop_PWM)){
            Motor_PWM_Left(0);
            Motor_PWM_Right(0);
            Motor_PWM_Enable(false);
            mf_stop_PWM.active = false;
        }

        // [State-machine flag] Send system information
        if(MSG_FLAG_Execute(&mf_send_sys_info)){
            if(firstLoopSysData){
                systemDataTime.startTime = GetTime();
                firstLoopSysData = !firstLoopSysData;
            }
            

            if(mf_send_sys_info.duration <= 0){
                systemData.time      = SecondsSince(&systemDataTime.startTime);
                // systemData.PWM_L     = Get_Motor_PWM_Left();
                // systemData.PWM_R     = Get_Motor_PWM_Right();
                systemData.Encoder_L = Counts_Left();
                systemData.Encoder_R = Counts_Right();

                usb_send_msg("cf2h",'q',&systemData,sizeof(systemData));

                mf_send_sys_info.active = false;

                firstLoopSysData = !firstLoopSysData;
                
            }else if(SecondsSince(&systemDataTime.last_trigger_time) >= mf_send_sys_info.duration){
                systemDataTime.last_trigger_time = GetTime();

                systemData.time      = SecondsSince(&systemDataTime.startTime);
                // systemData.PWM_L     = Get_Motor_PWM_Left();
                // systemData.PWM_R     = Get_Motor_PWM_Right();
                systemData.Encoder_L = Counts_Left();
                systemData.Encoder_R = Counts_Right();

                usb_send_msg("cf4h",'Q',&systemData,sizeof(systemData));
            }
        }
    }
}
