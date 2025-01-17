#include "Ring_Buffer.h"
#include <stdio.h> // required for the printf in rb_print_data_X functions

// define constant masks for use later based on length chosen
// these are global scope only to this c file
const uint8_t RB_MASK_F = RB_LENGTH_F-1;
const uint8_t RB_MASK_C = RB_LENGTH_C-1; 


/* Initialization */
void rb_initialize_F( struct Ring_Buffer_F* p_buf )
{
    // set start and end indicies to 0
    // no point changing data
    p_buf->start_index = 0;
    p_buf->end_index = 0;
}

void rb_initialize_C( struct Ring_Buffer_C* p_buf )
{
    // set start and end indicies to 0
    // no point changing data
    p_buf->start_index = 0;
    p_buf->end_index = 0;
}


/* Return active Length of Buffer */
uint8_t rb_length_F( const struct Ring_Buffer_F* p_buf)
{
    // calculate the active length using the mask and 2's complement to help
    // verify for your self why this works!
    uint8_t length = (p_buf->end_index - p_buf->start_index) & RB_MASK_F;
    return length;
}
uint8_t rb_length_C( const struct Ring_Buffer_C* p_buf)
{
    uint8_t length = (p_buf->end_index - p_buf->start_index) & RB_MASK_C;
    return length;// update
}

/* Append element to end and lengthen */
void rb_push_back_F( struct Ring_Buffer_F* p_buf, float value)
{   
    // Put data at index end
    // Increment the end index and wrap using the mask.
    // If the end equals the start increment the start index`
    p_buf->buffer[p_buf->end_index] = value;

    p_buf->end_index = (p_buf->end_index + 1) & RB_MASK_F;

    if (p_buf->start_index == p_buf->end_index){
        p_buf->start_index = p_buf->start_index + 1;
    }
}
void rb_push_back_C( struct Ring_Buffer_C* p_buf, char value)
{
    // Put data at index end
    // Increment the end index and wrap using the mask.
    // If the end equals the start increment the start index`
    p_buf->buffer[p_buf->end_index] = value;

    p_buf->end_index = (p_buf->end_index + 1) & RB_MASK_C;

    if (p_buf->start_index == p_buf->end_index){
        p_buf->start_index = p_buf->start_index + 1;
    }
}

/* Append element to front and lengthen */
void rb_push_front_F( struct Ring_Buffer_F* p_buf, float value)
{
    // Decrement the start index and wrap using the mask.
    // If the end equals the start decrement the end index`
    // Set the value at the start index as desired.
    p_buf->start_index = (p_buf->start_index - 1) & RB_MASK_F;

    if (p_buf->end_index == p_buf->start_index){
        p_buf->end_index = p_buf->end_index - 1;
    }

    p_buf->buffer[p_buf->start_index] = value;
}
void rb_push_front_C( struct Ring_Buffer_C* p_buf, char value)
{
    // Decrement the start index and wrap using the mask.
    // If the end equals the start decrement the end index`
    // Set the value at the start index as desired.
    p_buf->start_index = (p_buf->start_index - 1) & RB_MASK_C;

    if (p_buf->end_index == p_buf->start_index){
        p_buf->end_index = p_buf->end_index - 1;
    }

    p_buf->buffer[p_buf->start_index] = value;
}

/* Remove element from end and shorten */
float rb_pop_back_F( struct Ring_Buffer_F* p_buf)
{
    // if end does not equal start (length zero),
    //    reduce end index by 1 and mask
    // 	  return value at at end
    // else return zero if length of list is zero
    if (p_buf->end_index != p_buf->start_index){
        p_buf->end_index = (p_buf->end_index - 1) & RB_MASK_F;
        return p_buf->buffer[p_buf->end_index];
    }

    return 0;
}
char  rb_pop_back_C( struct Ring_Buffer_C* p_buf)
{
    // if end does not equal start (length zero),
    //    reduce end index by 1 and mask
    // 	  return value at at end
    // else return zero if length of list is zero
    if (p_buf->end_index != p_buf->start_index){
        p_buf->end_index = (p_buf->end_index - 1) & RB_MASK_C;
        return p_buf->buffer[p_buf->end_index];
    }

    return 0;
}

/* Remove element from start and shorten */
float rb_pop_front_F( struct Ring_Buffer_F* p_buf)
{
    // get value to return at front
    // if end does not equal start (length zero),
    //    increase start index by 1 and mask
    // else return zero if length of list is zero
    // return value
    float hold = p_buf->buffer[p_buf->start_index];

    if (p_buf->end_index != p_buf->start_index){
        p_buf->start_index = (p_buf->start_index + 1) & RB_MASK_F;
        return hold;
    }

    return 0; 
}
char  rb_pop_front_C( struct Ring_Buffer_C* p_buf)
{
    // get value to return at front
    // if end does not equal start (length zero),
    //    increase start index by 1 and mask
    // else return zero if length of list is zero
    // return value
    char hold = p_buf->buffer[p_buf->start_index];

    if (p_buf->end_index != p_buf->start_index){
        p_buf->start_index = (p_buf->start_index + 1) & RB_MASK_C;
        return hold;
    }

    return 0;
}

/* access element */
float rb_get_F( const struct Ring_Buffer_F* p_buf, uint8_t index)
{
    // return value at start + index wrapped properly
    return p_buf->buffer[(p_buf->start_index+index) & RB_MASK_F];
}
char  rb_get_C( const struct Ring_Buffer_C* p_buf, uint8_t index)
{
    // return value at start + index wrapped properly
    return p_buf->buffer[(p_buf->start_index+index) & RB_MASK_C];
}

/* set element - This behavior is 
   poorly defined if inedex is outside of active length.
   Use of the push_back or push_front methods are prefered.
*/
void  rb_set_F( struct Ring_Buffer_F* p_buf, uint8_t index, float value)
{
    // set value at start + index wrapped properly
    p_buf->buffer[(p_buf->start_index+index) & RB_MASK_F] = value;
}
void  rb_set_C( struct Ring_Buffer_C* p_buf, uint8_t index, char value)
{
    // set value at start + index wrapped properly
    p_buf->buffer[(p_buf->start_index+index) & RB_MASK_C] = value;
}


/*
 * The below functions are provided to help you debug. They print out the length, start and end index, active elements,
 * and the contents of the buffer.
 */
void rb_print_data_F(struct Ring_Buffer_F *p_buf)
{
    printf("-------FLOAT RINGBUFFER INFO--------\nRing Buffer Length: %i\nStart index: %i\nEnd index: %i\n",rb_length_F(p_buf),p_buf->start_index,p_buf->end_index);

    printf("\nActive Storage\n");
    for(int i=0; i<rb_length_F(p_buf); i++)
        printf("Index: %i, Internal Index: %i, Value: %f\n", i, p_buf->start_index+i, rb_get_F(p_buf,i) );

    printf("\nInternal Storage\n");
    for(int i=0; i<RB_LENGTH_F; i++)
        printf("Internal Index: %i, Value: %f\n", i, p_buf->buffer[i] );

    printf("-------END FLOAT RINGBUFFER INFO---------\n\n");

}

void rb_print_data_C(struct Ring_Buffer_C *p_buf)
{
    printf("-------CHAR RINGBUFFER INFO--------\nRing Buffer Length: %i\nStart index: %i\nEnd index: %i\n",rb_length_C(p_buf),p_buf->start_index,p_buf->end_index);

    printf("\nActive Storage\n");
    for(int i=0; i<rb_length_C(p_buf); i++)
        printf("Index: %i, Internal Index: %i, Value: %c\n", i, p_buf->start_index+i, rb_get_C(p_buf,i) );

    printf("\nInternal Storage\n");
    for(int i=0; i<RB_LENGTH_C; i++)
        printf("Internal Index: %i, Value: %c\n", i, p_buf->buffer[i] );

    printf("-------END CHAR RINGBUFFER INFO---------\n\n");

}