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
    usb_flush_input_buffer();// Flush buffer
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
    Initialize();

    // Tracking variable for mf_loop_timer
    bool firstLoop = true;

    // Variable for storing user command
    char command;
    // Build a meaningful structure for storing data about timing
    struct __attribute__((__packed__)) { uint8_t B; float f; } timeData;
    // Build a meaningful structure for storing low battery message
    struct __attribute__((__packed__)) { char let[7]; float volt; } msg = {
        .let = {'B','A','T',' ','L','O','W'},
        .volt = bat_volt
    };

    // Battery voltage stuff
    // Get unfiltered battery voltage to help the filter smooth out quicker than sending it 0 to begin with
    float unfiltered_voltage = Battery_Voltage();
    // Order & coefficients for Butterworth filter from homework
    int   order = 4;
    float numerator_coeffs[5]   = {8.063598650370949e-04,0.003225439460148,0.004838159190223,0.003225439460148,8.063598650370949e-04}; // Matlab B values
    float denominator_coeffs[5] = {1,-3.017555238686491,3.507193724716209,-1.847550944118580,0.370814215929455}; // Matlab A values
    // Create instance of filter stucture for battery voltage
    Filter_Data_t voltage_Filter
    // Initalize filter (might be good to add an if to the Initalize() call to reinitalize this too, if needed)
    Filter_Init(&voltage_Filter, &numerator_coeffs, &denominator_coeffs, order);

    for (;;){
        //USB_Echo_Task();
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
        if(MSG_FLAG_Execute(&mf_encoder_count)){
            // Build a meaningful structure to put encoder counts in.
            struct __attribute__((packed)) { float L_Count; float R_Count; } encoderData;
            encoderData.L_Count = Counts_Left();
            encoderData.R_Count = Counts_Right();
            usb_send_msg("cff", 'e', &encoderData, sizeof(encoderData));
        }

        // get & filter voltage every iteration (can we do this every X iterations or X seconds?)
        unfiltered_voltage = Battery_Voltage();
        float filtered_voltage = Filter_Value(&voltage_Filter,unfiltered_voltage);

        // [State-machine flag] Send battery voltage
        if(MSG_FLAG_Execute(&mf_send_voltage)){
            
            usb_send_msg("cf", 'b', filtered_voltage, sizeof(filtered_voltage));
        }
    }
}
