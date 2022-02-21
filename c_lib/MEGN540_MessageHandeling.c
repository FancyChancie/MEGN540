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

#include "MEGN540_MessageHandeling.h"


static inline void MSG_FLAG_Init(MSG_FLAG_t* p_flag)
{
    p_flag->active = false;
    p_flag->duration = -1;
    p_flag->last_trigger_time.microsec = 0;
    p_flag->last_trigger_time.millisec = 0;
}


/**
 * Function MSG_FLAG_Execute indicates if the action associated with the message flag should be executed
 * in the main loop both because its active and because its time.
 * @return [bool] True for execute action, False for skip action
 */
bool MSG_FLAG_Execute(MSG_FLAG_t* p_flag)
{
    // What is the logic to indicate an action should be executed?
    
    // if active and duration is less than (or equal to) the last trigger time, return true,
    // otherwise, return false
    return p_flag->active && (p_flag->duration <= SecondsSince(&p_flag->last_trigger_time));
}


/**
 * Function Message_Handling_Init initializes the message handling and all associated state flags and data to their default
 * conditions.
 */
void Message_Handling_Init()
{
    // Initialize any state machine flags to control your main-loop state machine
    MSG_FLAG_Init(&mf_restart);
    MSG_FLAG_Init(&mf_send_time);
    MSG_FLAG_Init(&mf_loop_timer);
    MSG_FLAG_Init(&mf_time_float_send);

}

/**
 * Function Message_Handler processes USB messages as necessary and sets status flags to control the flow of the program.
 * It returns true unless the program receives a reset message.
 * @return
 */
void Message_Handling_Task()
{
    // I suggest you use your peak function and a switch interface
    // Either do the simple stuff strait up, set flags to have it done later.
    // If it just is a USB thing, do it here, if it requires other hardware, do it in the main and
    // set a flag to have it done here.

    // Check to see if their is data in waiting
    if(usb_msg_length() == 0) return; // nothing to process...

    // Get Your command designator without removal so if their are not enough bytes yet, the command persists
    char command = usb_msg_peek();

    // process command
    switch(command){
        case '*':
            if(usb_msg_length() >= MEGN540_Message_Len('*')){
                // then process your times...
                // remove the command from the usb recieved buffer using the usb_msg_get() function
                usb_msg_get(); // removes the first character from the received buffer, we already know it was a * so no need to save it as a variable

                // Build a meaningful structure to put your data in. Here we want two floats.
                struct __attribute__((__packed__)) { float v1; float v2; } data;

                // Copy the bytes from the usb receive buffer into our structure so we can use the information
                usb_msg_read_into( &data, sizeof(data) );

                // Do the thing you need to do. Here we want to multiply.
                float ret_val = data.v1 * data.v2;

                // send response right here if appropriate.
                usb_send_msg("cf", command, &ret_val, sizeof(ret_val));
            }
            break;
        case '/':
            if(usb_msg_length() >= MEGN540_Message_Len('/')){
                // then process your divide...
                // remove the command from the usb recieved buffer using the usb_msg_get() function
                usb_msg_get(); // removes the first character from the received buffer, we already know it was a / so no need to save it as a variable

                // Build a meaningful structure to put your data in. Here we want two floats.
                struct __attribute__((__packed__)) { float v1; float v2; } data;

                // Copy the bytes from the usb receive buffer into our structure so we can use the information
                usb_msg_read_into(&data, sizeof(data) );

                // Do the thing you need to do. Here we want to divide.
                float ret_val = data.v1 / data.v2;

                // send response right here if appropriate.
                usb_send_msg("cf", command, &ret_val, sizeof(ret_val));
            }
            break;
        case '+':
            if(usb_msg_length() >= MEGN540_Message_Len('+')){
                // then process your plus...
                // remove the command from the usb recieved buffer using the usb_msg_get() function
                usb_msg_get(); // removes the first character from the received buffer, we already know it was a + so no need to save it as a variable

                // Build a meaningful structure to put your data in. Here we want two floats.
                struct __attribute__((__packed__)) { float v1; float v2; } data;

                // Copy the bytes from the usb receive buffer into our structure so we can use the information
                usb_msg_read_into( &data, sizeof(data) );

                // Do the thing you need to do. Here we want to add.
                float ret_val = data.v1 + data.v2;

                // send response right here if appropriate.
                usb_send_msg("cf", command, &ret_val, sizeof(ret_val));
            }
            break;
        case '-':
            if(usb_msg_length() >= MEGN540_Message_Len('-')){
                // then process your minus...
                // remove the command from the usb recieved buffer using the usb_msg_get() function
                usb_msg_get(); // removes the first character from the received buffer, we already know it was a - so no need to save it as a variable

                // Build a meaningful structure to put your data in. Here we want two floats.
                struct __attribute__((__packed__)) { float v1; float v2; } data;

                // Copy the bytes from the usb receive buffer into our structure so we can use the information
                usb_msg_read_into( &data, sizeof(data) );

                // Do the thing you need to do. Here we want to subtract.
                float ret_val = data.v1 - data.v2;

                // send response right here if appropriate.
                usb_send_msg("cf", command, &ret_val, sizeof(ret_val));
            }
            break;
        case 't':
            // case 't' returns the time it requested followed by the time to complete the action specified by the second input char. 
            if(usb_msg_length() >= MEGN540_Message_Len('t')){
                // then process your t...
                uint8_t subcommand = usb_msg_peek_ahead(1);
                if(subcommand == 0){    // send time now
                    mf_send_time.active = true; // set flag to true so it knows to send time
                }else if(subcommand == 1){  // send time to complete one full loop iteration
                    mf_loop_timer.active = true;
                    mf_loop_timer.last_trigger_time = GetTime();
                    mf_loop_timer.duration = -1;
                }else if(subcommand == 2){  // send time to send float
                    mf_time_float_send.active = true;
                    mf_time_float_send.last_trigger_time = GetTime();
                    mf_time_float_send.duration = -1;
                }else{
                    usb_send_msg("cc", '?', &subcommand, sizeof(subcommand));
                }
            }
            break;
        case 'T':
            // case 'T' returns the time it requested followed by the time to complete the action specified by the second input char
            // and returns the time every X milliseconds. If the time is zero or negative it cancels the request without response.
            if(usb_msg_length() >= MEGN540_Message_Len('T')){
                // then process your T...
                uint8_t subcommand = usb_msg_peek_ahead(1);
                uint8_t durcommand = usb_msg_peek_ahead(2);

                if(subcommand <= 0){   // cancel request without response
                    MSG_FLAG_Init(&mf_send_time);
                    MSG_FLAG_Init(&mf_loop_timer);
                    MSG_FLAG_Init(&mf_time_float_send);
                }else if(subcommand == 1){   // send time every 'duration' milliseconds
                    mf_send_time.active = true;
                    mf_send_time.last_trigger_time = GetTime();
                    mf_send_time.duration = durcommand;
                }else if(subcommand == 2){   // send time to complete one loop iteration
                    mf_loop_timer.active = true;
                    mf_loop_timer.last_trigger_time = GetTime();
                    mf_loop_timer.duration = durcommand;
                }else if(subcommand == 3){  // send time to send float
                    mf_time_float_send.active = true;
                    mf_time_float_send.last_trigger_time = GetTime();
                    mf_time_float_send.duration = durcommand;
                }else{
                    usb_send_msg("cc", '?', &subcommand, sizeof(subcommand));
                }
            }
            break;
        case '~':
            if(usb_msg_length() >= MEGN540_Message_Len('~')){
                // then process your reset by setting the mf_restart flag 
                mf_restart.active = true;
            }
            break;
        default:
            // What to do if you dont recognize the command character
            usb_send_msg("cc", '?', &command, sizeof(command));
            usb_flush_input_buffer();
            break;
    }
}

/**
 * Function MEGN540_Message_Len returns the number of bytes associated with a command string per the
 * class documentation;
 * @param cmd
 * @return Size of expected string. Returns 0 if unreconized.
 */
uint8_t MEGN540_Message_Len( char cmd )
{
    switch(cmd){
        case '~': return	1; break;
        case '*': return	9; break;
        case '/': return	9; break;
        case '+': return	9; break;
        case '-': return	9; break;
        case 't': return	2; break;
        case 'T': return	6; break;
        case 'e': return	1; break;
        case 'E': return	5; break;
        case 'b': return	1; break;
        case 'B': return	5; break;
//        case 'a': return	1; break;
//        case 'A': return 	5; break;
//        case 'w': return	1; break;
//        case 'W': return 	5; break;
//        case 'm': return	1; break;
//        case 'M': return	5; break;
        case 'p': return	5; break;
        case 'P': return	9; break;
        case 's': return 	1; break;
        case 'S': return 	1; break;
        case 'q': return	1; break;
        case 'Q': return 	5; break;
        case 'd': return 	9; break;
        case 'D': return   13; break;
        case 'v': return	9; break;
        case 'V': return   13; break;
        default:  return	0; break;
    }
}