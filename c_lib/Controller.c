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

/**
 * Controller.h/c defines the functions necessary to implement a z-transform
 * based control of the forward angular and linear speed of the car. 
 * 
 * This code leverages the filter.h/c code developed in the homework. 
 *
 */

#include "Controller.h"
#include "Filter.h"

/**
 * Function Initialize_Controller setsup the z-transform based controller for the system.
 */
void Controller_Init( Controller_t* p_cont, float kp, float* num, float* den, uint8_t order, float update_period )
{
    p_cont->kp = kp;
    p_cont->update_period = update_period;
    //p_cont->controller = Filter_Init(&p_cont->controller, num, den, order); // Initalize control filter
    Filter_Init(&p_cont->controller,num,den,order); // Initalize control filter
}

/**
 * Function Controller_Set_Target_Velocity sets the target velocity for the 
 * controller.
 */
void Controller_Set_Target_Velocity( Controller_t* p_cont, float vel )
{
    p_cont->target_vel = vel;
}

/**
 * Function Controller_Set_Target_Position sets the target postion for the 
 * controller, this also sets the target velocity to 0.
 */
void Controller_Set_Target_Position( Controller_t* p_cont, float vel )
{
    p_cont->target_vel = 0.0;
    p_cont->target_pos = vel * p_cont->update_period;
}

/**
 * Function Controller_Update takes in a new measurement and returns the
 * new control value.
 */
float Controller_Update( Controller_t* p_cont, float measurement, float dt )
{
    float filter_val = Filter_Value(&p_cont->controller, measurement);

    // Position update
    if(p_cont->target_vel > 0){
        float target = measurement + dt * p_cont->target_vel;
    // Velocity update
    }else{
        float target = p_cont->target_pos;
    } 

    float last_control_command = p_cont->kp * (target - filter_val);
    return last_control_command;
}

/**
 * Function Controller_Last returns the last control command
 */
float Controller_Last( Controller_t* p_cont)
{
    return Filter_Last_Output(&p_cont->controller);
}

/**
 * Function Controller_SettTo sets the Filter's input and output lists
 * to match the measurement so it starts with zero error.
 */
void Controller_SetTo( Controller_t* p_cont, float measurement )
{
    Filter_SetTo(&p_cont->controller, measurement);
}

/**
 * Function Controller_ShiftBy shifts the Filter's input and output lists
 * by the desired amount. This is helpful when dealing with wrapping.
 */
void Controller_ShiftBy( Controller_t* p_cont, float measurement )
{
    Filter_ShiftBy(&p_cont->controller, measurement);
}