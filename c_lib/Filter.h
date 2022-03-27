#include "Filter.h"

/**
 * Function Filter_Init initializes the filter given two float arrays and the order of the filter.  Note that the
 * size of the array will be one larger than the order. (First order systems have two coefficients).
 *
 *  1/A_0*( SUM( B_i * input_i )  -   SUM( A_i * output_i) )
 *         i=0..N                    i=1..N
 *
 *  Note a 5-point moving average filter has coefficients:
 *      numerator_coeffs   = { 5 0 0 0 0 };
 *      denominator_coeffs = { 1 1 1 1 1 };
 *      order = 4;
 *
 * @param p_filt pointer to the filter object
 * @param numerator_coeffs The numerator coefficients (B/beta traditionally)
 * @param denominator_coeffs The denominator coefficients (A/alpha traditionally)
 * @param order The filter order
 */
void  Filter_Init ( Filter_Data_t* p_filt, float* numerator_coeffs, float* denominator_coeffs, uint8_t order )
{
    rb_initialize_F(&p_filt->numerator);
    rb_initialize_F(&p_filt->denominator);
    rb_initialize_F(&p_filt->out_list);
    rb_initialize_F(&p_filt->in_list);

    for(int i=0;i<order+1;i++){
        rb_push_back_F(&p_filt->numerator,numerator_coeffs[i]);
        rb_push_back_F(&p_filt->in_list,0);
        rb_push_back_F(&p_filt->denominator,denominator_coeffs[i]);
        rb_push_back_F(&p_filt->out_list,0);
    }

	return;
}

/**
 * Function Filter_ShiftBy shifts the input list and output list to keep the filter in the same frame. This especially
 * useful when initializing the filter to the current value or handling wrapping/overflow issues.
 * @param p_filt
 * @param shift_amount
 */
void  Filter_ShiftBy( Filter_Data_t* p_filt, float shift_amount )
{
    for(int i=0;i<rb_length_F(&p_filt->in_list);i++){
        float valN = rb_get_F(&p_filt->in_list,i);
        float valD = rb_get_F(&p_filt->out_list,i);

        rb_set_F(&p_filt->in_list,i,valN + shift_amount);
        rb_set_F(&p_filt->out_list,i,valD + shift_amount);
    }

    return;
}

/**
 * Function Filter_SetTo sets the initial values for the input and output lists to a constant defined value. This
 * helps to initialize or re-initialize the filter as desired.
 * @param p_filt Pointer to a Filter_Data sturcture
 * @param amount The value to re-initialize the filter to.
 */
void Filter_SetTo( Filter_Data_t* p_filt, float amount )
{
    for(int i=0;i<rb_length_F(&p_filt->in_list);i++){
        rb_set_F(&p_filt->in_list,i,amount);
        rb_set_F(&p_filt->out_list,i,amount);
    }

    return;
}

/**
 * Function Filter_Value adds a new value to the filter and returns the new output.
 * @param p_filt pointer to the filter object
 * @param value the new measurement or value
 * @return The newly filtered value
 */
float Filter_Value( Filter_Data_t* p_filt, float value)
{
    float first = 0;
    float last = 0;
    float fin = 0;

    float az = rb_get_F(&p_filt->denominator,0);

    rb_push_front_F(&p_filt->in_list,value);
    rb_pop_back_F(&p_filt->in_list);

    for(int i=0;i<rb_length_F(&p_filt->numerator);i++){
        float bn = rb_get_F(&p_filt->numerator,i);
        float in = rb_get_F(&p_filt->in_list,i);
        first += (bn*in);
    }

    for(int i=1;i<rb_length_F(&p_filt->denominator);i++){
        float an = rb_get_F(&p_filt->denominator,i);
        float out = rb_get_F(&p_filt->out_list,i-1);
        last += (an*out);
    }

    fin = (1/az)*(first-last);

    rb_push_front_F(&p_filt->out_list,fin);
    rb_pop_back_F(&p_filt->out_list);

	return fin;
}

/**
 * Function Filter_Last_Output returns the most up-to-date filtered value without updating the filter.
 * @return The latest filtered value
 */
float Filter_Last_Output( Filter_Data_t* p_filt )
{
	return rb_get_F(&p_filt->out_list,0);
}
