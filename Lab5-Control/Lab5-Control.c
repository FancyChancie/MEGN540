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
#include <stdlib.h>
#include "../c_lib/MEGN540_MessageHandeling.h"
#include "../c_lib/Timing.h"
#include "../c_lib/Encoder.h"
#include "../c_lib/MotorPWM.h"
#include "../c_lib/Filter.h"
#include "../c_lib/Battery_Monitor.h"
#include "../c_lib/Controller.h"

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
    Motor_PWM_Init(400);     // Initialize motors at TOP PWM of 400
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
    bool firstLoopDist = true;
    bool secondLoopDist = true;
    bool firstLoopVeloc = true;

    //////////////////////////////
    //// Program timing stuff ////
    //////////////////////////////
    // Variable for storing user command
    char command;
    // Build a meaningful structure for storing data about timing
    struct __attribute__((__packed__)) { uint8_t B; float f; } timeData;

    ///////////////////////////////
    //// Battery voltage stuff ////
    ///////////////////////////////
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
    float offBattVoltage = 3.0;
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

    ///////////////////////////
    //// System info stuff ////
    ///////////////////////////
    // Build a meaningful structure for storing data about timing for sending system info
    struct __attribute__((__packed__)) { float interval; Time_t startTime; Time_t last_trigger_time;} systemDataTime;
    // Build a meaningful structure for storing data about system info
    struct __attribute__((__packed__)) { float time; int16_t PWM_L; int16_t PWM_R; int16_t Encoder_L; int16_t Encoder_R;} systemData;

    //////////////////////////
    //// Controller stuff ////
    //////////////////////////
    struct __attribute__((__packed__)) { Time_t startTime; Time_t last_trigger_time;} controlTime;
    float distanceTraveled_Last_L;
    float distanceTraveled_Last_R;
    float angleTraveled_Last_L;
    float angleTraveled_Last_R;
    float velocity_L;
    float velocity_R;
    float velocity_T;
    float angular_T;
    float startRad_L;
    float startRad_R;
    float update_period = 0.005;
    // Left track controller values
    uint8_t order_L = 1;
    float Kp_L = 138.6274;
    float numerator_coeffs_L[2] = {1,-0.925};
    float denominator_coeffs_L[2] = {8.7776,-8.7026};
    Controller_t control_Filter_L;
    Controller_Init(&control_Filter_L,Kp_L,numerator_coeffs_L,denominator_coeffs_L,order_L,update_period);
    // Right track controller values
    uint8_t order_R = order_L;
    float Kp_R = 138.2969;
    float numerator_coeffs_R[2] = {1,-0.9249};
    float denominator_coeffs_R[2] = {8.8115,-8.7364};
    Controller_t control_Filter_R;
    Controller_Init(&control_Filter_R,Kp_R,numerator_coeffs_R,denominator_coeffs_R,order_R,update_period);

    /////////////////////////////////
    //// Zumo car physical stuff ////
    /////////////////////////////////
    float trackWheelDiameter = 0.035;          // [m] (aka 35 mm)
    float trackWheelRadius = trackWheelDiameter/2;
    float trackSeparationDistance = 0.084;    // [m] (aka 84 mm)


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
                    usb_send_msg("c7sf",'!',&low_batt_msg,sizeof(low_batt_msg));
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
            Motor_PWM_Enable(true);

            if(PWM_data.right_PWM < 0){
                PORTB |= (1 << PB1);
                PWM_data.right_PWM = -PWM_data.right_PWM;
            }else{
                PORTB &= ~(1 << PB1);
            }

            if(PWM_data.left_PWM < 0){
                PORTB |= (1 << PB2);
                PWM_data.left_PWM = -PWM_data.left_PWM;
            }else{
                PORTB &= ~(1 << PB2);
            }

            Motor_PWM_Left(PWM_data.left_PWM);
            Motor_PWM_Right(PWM_data.right_PWM);

            if(PWM_data.timed == false){
                mf_set_PWM.active = false;
            }

            else if(PWM_data.timed && SecondsSince(&mf_set_PWM.last_trigger_time) >= mf_set_PWM.duration){
                mf_set_PWM.active = false;
                mf_stop_PWM.active = true;
            }
        }

        // [State-machine flag] Stop the motors
        if(MSG_FLAG_Execute(&mf_stop_PWM)){
            Motor_PWM_Left(0);
            Motor_PWM_Right(0);
            Motor_PWM_Enable(false);
            mf_stop_PWM.active = false;
            mf_velocity_mode.active = false;
            mf_distance_mode.active = false;
            usb_flush_input_buffer();
        }

        // [State-machine flag] Send system information
        if(MSG_FLAG_Execute(&mf_send_sys_info)){
            if(firstLoopSysData){
                systemDataTime.startTime = GetTime();
                firstLoopSysData = !firstLoopSysData;
            }

            if(mf_send_sys_info.duration <= 0){
                systemData.time      = SecondsSince(&systemDataTime.startTime);
                systemData.PWM_L     = Get_Motor_PWM_Left();
                systemData.PWM_R     = Get_Motor_PWM_Right();
                systemData.Encoder_L = Rad_Left();
                systemData.Encoder_R = Rad_Right();

                usb_send_msg("cf4h",'q',&systemData,sizeof(systemData));

                mf_send_sys_info.active = false;

                firstLoopSysData = !firstLoopSysData;

            }else if(SecondsSince(&systemDataTime.last_trigger_time) >= mf_send_sys_info.duration){
                systemDataTime.last_trigger_time = GetTime();

                systemData.time      = SecondsSince(&systemDataTime.startTime);
                systemData.PWM_L     = Get_Motor_PWM_Left();
                systemData.PWM_R     = Get_Motor_PWM_Right();
                systemData.Encoder_L = Rad_Left();
                systemData.Encoder_R = Rad_Right();

                usb_send_msg("cf4h",'Q',&systemData,sizeof(systemData));
            }
        }

        // [State-machine flag] Distance mode
        if(MSG_FLAG_Execute(&mf_distance_mode)){
            if(firstLoopDist){
                Filter_Init(&control_Filter_L, numerator_coeffs_L, denominator_coeffs_L, order_L);
                Filter_Init(&control_Filter_R, numerator_coeffs_R, denominator_coeffs_R, order_R);
                // usb_send_msg("cf", 'B', &filtered_voltage, sizeof(filtered_voltage));
                distanceTraveled_Last_L = 0;
                distanceTraveled_Last_R = 0;
                float distanceTraveled_L = 0;
                
                float distanceTraveled_R = 0;
                float angleTraveled_L = 0;
                float angleTraveled_R = 0;
                angleTraveled_Last_L = 0;
                angleTraveled_Last_R = 0;
                startRad_L = Rad_Left();
                startRad_R = Rad_Right();
                controlTime.startTime = GetTime();
                controlTime.last_trigger_time = GetTime();
                firstLoopDist = false;
                Motor_PWM_Enable(true);
            }

            if(mf_distance_mode.duration < 0 || SecondsSince(&controlTime.startTime) >= mf_distance_mode.duration){
                mf_stop_PWM.active = true;
                firstLoopDist = true;
            }else{
                struct __attribute__((packed)) { float L; float R; float T;} trackData;
                struct __attribute__((packed)) { float L; float R; } PWData;
                // Linear
                float distanceTraveled_L = (Rad_Left() - startRad_L) * trackWheelRadius;
                float distanceTraveled_R = (Rad_Right() - startRad_R) * trackWheelRadius;
                float distanceTraveled_Total = (distanceTraveled_L + distanceTraveled_R)/2;
                // Angular
                float angleTraveled_L = distanceTraveled_L / trackWheelRadius;
                float angleTraveled_R = distanceTraveled_R / trackWheelRadius;
                float angleTraveled_Total = (angleTraveled_R - angleTraveled_L)/2;

                trackData.L = angleTraveled_L;
                trackData.R = angleTraveled_R;
                trackData.T = angleTraveled_Total;

                // Move distance
                if(angleTraveled_Total < Dist_data.angular && Dist_data.angular != 0){
                    if(SecondsSince(&controlTime.last_trigger_time) >= update_period){
                        control_Filter_L.target_pos = -Dist_data.angular;
                        control_Filter_R.target_pos = Dist_data.angular;
                        // Motor_PWM_Left(Controller_Update(&control_Filter_L, angleTraveled_L - angleTraveled_Last_L, SecondsSince(&controlTime.last_trigger_time)));
                        // Motor_PWM_Right(Controller_Update(&control_Filter_R, angleTraveled_R - angleTraveled_Last_R, SecondsSince(&controlTime.last_trigger_time)));

                        PWData.L = Controller_Update(&control_Filter_L, angleTraveled_L, SecondsSince(&controlTime.last_trigger_time));
                        PWData.R = Controller_Update(&control_Filter_R, angleTraveled_R, SecondsSince(&controlTime.last_trigger_time));

                        usb_send_msg("cff", 'p', &PWData, sizeof(PWData));

                        if(PWData.R < 0){
                            PORTB |= (1 << PB1);
                            PWData.R = -PWData.R;
                        }else{
                            PORTB &= ~(1 << PB1);
                        }

                        if(PWData.L < 0){
                            PORTB |= (1 << PB2);
                            PWData.L = -PWData.L;
                        }else{
                            PORTB &= ~(1 << PB2);
                        }

                        Motor_PWM_Left(PWData.L);
                        Motor_PWM_Right(PWData.R);

                        // Save travel distance
                        angleTraveled_Last_L = angleTraveled_L - angleTraveled_Last_L;
                        angleTraveled_Last_R = angleTraveled_R - angleTraveled_Last_R;

                        controlTime.last_trigger_time = GetTime();

                        // usb_send_msg("cfff", 'I', &trackData, sizeof(trackData));
                    }
                // }else if(secondLoopDist && angleTraveled_Total > Dist_data.angular){
                //     distanceTraveled_Last_L = 0;
                //     distanceTraveled_Last_R = 0;
                //     startRad_L = Rad_Left();
                //     startRad_R = Rad_Right();
                //     secondLoopDist = false;
                }else if(distanceTraveled_Total < Dist_data.linear){
                    if(SecondsSince(&controlTime.last_trigger_time) >= update_period){
                        control_Filter_L.target_pos = Dist_data.linear;
                        control_Filter_R.target_pos = Dist_data.linear;

                        PWData.L = Controller_Update(&control_Filter_L, distanceTraveled_L, SecondsSince(&controlTime.last_trigger_time));
                        PWData.R = Controller_Update(&control_Filter_R, distanceTraveled_R, SecondsSince(&controlTime.last_trigger_time));

                        usb_send_msg("cff", 'p', &PWData, sizeof(PWData));
                        // usb_send_msg("cf",'k',&distanceTraveled_Total,sizeof(distanceTraveled_Total));

                        if(PWData.R < 0){
                            PORTB |= (1 << PB1);
                            PWData.R = -PWData.R;
                        }else{
                            PORTB &= ~(1 << PB1);
                        }

                        if(PWData.L < 0){
                            PORTB |= (1 << PB2);
                            PWData.L = -PWData.L;
                        }else{
                            PORTB &= ~(1 << PB2);
                        }

                        Motor_PWM_Left(PWData.L);
                        Motor_PWM_Right(PWData.R);

                        // Save travel distance
                        distanceTraveled_Last_L = distanceTraveled_L - distanceTraveled_Last_L;
                        distanceTraveled_Last_R = distanceTraveled_R - distanceTraveled_Last_R;

                        controlTime.last_trigger_time = GetTime();

                        // usb_send_msg("cff", 'I', &trackData, sizeof(trackData));
                    }
                }else{
                    mf_send_encoder.active = true;
	                firstLoopDist = true;
                    mf_stop_PWM.active = true;
                }
            }
        }

        // [State-machine flag] Velocity mode
        if(MSG_FLAG_Execute(&mf_velocity_mode)){
            if(firstLoopVeloc){
                Filter_Init(&voltage_Filter, numerator_coeffs, denominator_coeffs, order);
                startRad_L = Rad_Left();
                startRad_R = Rad_Right();
                controlTime.startTime = GetTime();
                controlTime.last_trigger_time = GetTime();
                firstLoopVeloc = !firstLoopVeloc;
                Motor_PWM_Enable(true);

                velocity_L = Veloc_data.linear - (trackSeparationDistance*Veloc_data.angular)/2;
                velocity_R = (trackSeparationDistance*Veloc_data.angular) + velocity_L;

                control_Filter_L.target_vel = velocity_L;
                control_Filter_R.target_vel = velocity_R;
            }

            if(mf_velocity_mode.duration < 0 || SecondsSince(&controlTime.startTime) >= mf_velocity_mode.duration){
                mf_stop_PWM.active = true;
                firstLoopVeloc = !firstLoopVeloc;
                mf_velocity_mode.active = false;
            }else{
                struct __attribute__((packed)) { float L; float R; } PWData;

                if(SecondsSince(&controlTime.last_trigger_time) >= update_period){
                    float distanceTraveled_L = (Rad_Left() - startRad_L) * trackWheelRadius;
                    float distanceTraveled_R = (Rad_Right() - startRad_R) * trackWheelRadius;

                    distanceTraveled_L = distanceTraveled_L / SecondsSince(&controlTime.last_trigger_time);
                    distanceTraveled_R = distanceTraveled_R / SecondsSince(&controlTime.last_trigger_time);

                    PWData.L = Controller_Update(&control_Filter_L, distanceTraveled_L, SecondsSince(&controlTime.last_trigger_time));
                    PWData.R = Controller_Update(&control_Filter_R, distanceTraveled_R, SecondsSince(&controlTime.last_trigger_time));

                        if(PWData.R < 0){
                            PORTB |= (1 << PB1);
                            PWData.R = -PWData.R;
                        }else{
                            PORTB &= ~(1 << PB1);
                        }

                        if(PWData.L < 0){
                            PORTB |= (1 << PB2);
                            PWData.L = -PWData.L;
                        }else{
                            PORTB &= ~(1 << PB2);
                        }

                        Motor_PWM_Left(PWData.L);
                        Motor_PWM_Right(PWData.R);

                    controlTime.last_trigger_time = GetTime();

                    // usb_send_msg("cff", 'P', &PWData, sizeof(PWData));
                }
            }
        }
    }
}
