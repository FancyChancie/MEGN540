#include "Filter.h"

/**
 * Function Filter_Init initializes the filter given two float arrays and the order of the filter.  Note that the
 * size of the array will be one larger than the order. (First order systems have two coefficients).
 *
 *  1/A_0 *   ( SUM( B_i * input_i )  -   SUM( A_i * output_i) )
 *              i=0..N                    i=1..N
 *
 *  Note: A 5-point moving average filter has coefficients:
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
	// initialize ring buffer of p_filt
	rb_initialize_F(&p_filt->numerator);
	rb_initialize_F(&p_filt->denominator);
	rb_initialize_F(&p_filt->in_list);
	rb_initialize_F(&p_filt->out_list);

	// fill p_filt ring buffer appropriately
	for(int i=0; i <= order; i++){
		rb_push_back_F(&p_filt->numerator,numerator_coeffs[i]);
		rb_push_back_F(&p_filt->denominator,denominator_coeffs[i]);
		rb_push_back_F(&p_filt->in_list,0);
		rb_push_back_F(&p_filt->out_list,0);
	}
}

/**
 * Function Filter_ShiftBy shifts the input list and output list to keep the filter in the same frame. This is especially
 * useful when initializing the filter to the current value or handling wrapping/overflow issues.
 * @param p_filt
 * @param shift_amount
 */
void  Filter_ShiftBy( Filter_Data_t* p_filt, float shift_amount )
{
	// shift each value [i] in in_list & out_list by @param shift_amount
	for(int i = 0; i <= rb_length_F(&p_filt->in_list); i++){
		rb_set_F(&p_filt->in_list,i,shift_amount);
		rb_set_F(&p_filt->out_list,i,shift_amount);
	}
}

/**
 * Function Filter_SetTo sets the initial values for the input and output lists to a constant defined value.
 * This helps to initialize or re-initialize the filter as desired.
 * @param p_filt Pointer to a Filter_Data sturcture
 * @param amount The value to re-initialize the filter to.
 */
void Filter_SetTo( Filter_Data_t* p_filt, float amount )
{
	// set all values [i] in in_list & out_list to @param amount
	for(int i = 0; i <= rb_length_F(&p_filt->in_list); i++){
		rb_set_F(&p_filt->in_list,i,amount);
		rb_set_F(&p_filt->out_list,i,amount);
	}
}

/**
 * Function Filter_Value adds a new value to the filter and returns the new output.
 * @param p_filt pointer to the filter object
 * @param value the new measurement or value
 * @return The newly filtered value
 */
float Filter_Value( Filter_Data_t* p_filt, float value)
{
	// add @param value to in_list ring buffer
	rb_push_front_F(&p_filt->in_list,value);
	// remove the last value from in_list ring buffer
	rb_pop_back_F(&p_filt->in_list);

	// initialize return value
	float filtered_value = 0;

	// sum B_i * input_i, each iteration adds to filtered_value
    for(int i = 0; i <= rb_length_F(&p_filt->in_list); i++){
        filtered_value += rb_get_F(&p_filt->numerator,i) * rb_get_F(&p_filt->in_list,i);
    }

	// sum A_i * output_i, each iteration subtracts from filtered_value
    for(int i = 1; i <= rb_length_F(&p_filt->out_list); i++){
        filtered_value -= rb_get_F(&p_filt->denominator,i) * rb_get_F(&p_filt->out_list,i-1);
    }

	// divide by A_0
	filtered_value /= rb_get_F(&p_filt->denominator,0);

	// add filtered_value to out_list ring buffer
	rb_push_front_F(&p_filt->out_list,filtered_value);
	// remove last value from out_list ring buffer
	rb_pop_back_F(&p_filt->out_list);

	return filtered_value;
}

/**
 * Function Filter_Last_Output returns the most up-to-date filtered value without updating the filter.
 * @return The latest filtered value
 */
float Filter_Last_Output( Filter_Data_t* p_filt )
{
	return (float) rb_get_F(&p_filt->out_list,0);
}