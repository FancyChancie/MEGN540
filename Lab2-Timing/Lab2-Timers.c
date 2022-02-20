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

/** 
 * Function to re/initialize states
 */
void Initialize()
{
    USB_SetupHardware();     // initialize USB
    GlobalInterruptEnable(); // enable Global Interrupts for USB and Timer etc.
    Message_Handling_Init(); // initialize message handing and all associated flags
    SetupTimer0();           // initialize timer zero functionality
    usb_flush_input_buffer();// flush buffer
}

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
    Initialize();

    bool firstLoop = true; //tracking variable for mf_loop_timer

    for (;;){
        //USB_Echo_Task();
        USB_Upkeep_Task();
        Message_Handling_Task();

        // [State-machine flag] Restart
        if(MSG_FLAG_Execute(&mf_restart)){
            Initialize(); // reinitialize everything
        }
        
        // [State-machine flag] Send time
        if(MSG_FLAG_Execute(&mf_send_time)){
            char command = usb_msg_get(); // store main command

            // Build a meaningful structure to put subcommand and time in.
            struct __attribute__((__packed__)) { uint8_t B; float f; } data;
            
            data.B = usb_msg_get();  // store subcommand
            data.f = GetTimeSec();   // get current time

            mf_send_time.last_trigger_time = GetTime();

            if(mf_send_time.duration <= 0){
                // send response
                usb_send_msg("cBf", command, &data, sizeof(data));
                mf_send_time.active = false;
            }else{
                // mf_send_time.last_trigger_time = GetTime();
                // struct __attribute__((__packed__)) { float duration; float time; } data;
                // data.duration = mf_send_time.duration;
                // data.time = currentTime;
                // usb_send_msg("cBf", 'T', &data, sizeof(data));
            }
        }

        // [State-machine flag] Time to complete loop
        if(MSG_FLAG_Execute(&mf_loop_timer)){
            static Time_t loopTimeStart;    // struct to store loop time (static so it doesn't get deleted after this inner loop ends)
            
            if(firstLoop){
                loopTimeStart = GetTime();   // fill loopTime struct
            }else{
                char command = usb_msg_get(); // store main command

                // Build a meaningful structure to put subcommand and time in.
                struct __attribute__((__packed__)) { uint8_t B; float f; } data;
            
                data.B = usb_msg_get();  // store subcommand
                data.f = SecondsSince(&loopTimeStart);;   // get loop time

                //float loopTimeEnd = SecondsSince(&loopTimeStart);
                if(mf_loop_timer.duration <= 0){
                    usb_send_msg("cBf", command, &data, sizeof(data));
                    mf_loop_timer.active = false;
                }else{
                //     struct __attribute__((__packed__)) { float duration; float time; } data;
                //     data.duration = mf_loop_timer.duration;
                //     data.time = loopTimeEnd;
                //     usb_send_msg("cBf", 'T', &data, sizeof(data));
                //     mf_loop_timer.last_trigger_time = GetTime();
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
            USB_Upkeep_Task(); // wait for send
            //float floatSendTime = SecondsSince(&floatSendStart); // get time it took to send float
            data.f = SecondsSince(&floatSendStart);   // get current sincr time

            if(mf_time_float_send.duration <= 0){
                usb_send_msg("cBf", command, &data, sizeof(data));
                mf_time_float_send.active = false;
            }else{
                // struct __attribute__((__packed__)) { float duration; float time; } data;
                // data.duration = mf_time_float_send.duration;
                // data.time = floatSendTime;
                // usb_send_msg("cBf", 'T', &data, sizeof(data));
                // mf_time_float_send.last_trigger_time = GetTime();
            }
        }
    }
}