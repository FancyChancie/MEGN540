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

/*
  Copyright 2020  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the DualVirtualSerial demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "SerialIO.h"

// *** MEGN540  ***
// Ring Buffer Objects
static struct Ring_Buffer_C _usb_receive_buffer;
static struct Ring_Buffer_C _usb_send_buffer;


/** Contains the current baud rate and other settings of the first virtual serial port. While this demo does not use
 *  the physical USART and thus does not use these settings, they must still be retained and returned to the host
 *  upon request or the host will assume the device is non-functional.
 *
 *  These values are set by the host via a class-specific request, however they are not required to be used accurately.
 *  It is possible to completely ignore these value or use other settings as the host is completely unaware of the physical
 *  serial link characteristics and instead sends and receives data in endpoint streams.
 */
static CDC_LineEncoding_t LineEncoding1 = { .BaudRateBPS = 0,
                                            .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                            .ParityType  = CDC_PARITY_None,
                                            .DataBits    = 8                            };


void USB_Upkeep_Task(){
    USB_USBTask();

    // *** MEGN540  ***
    // Get next byte from the USB hardware, send next byte to the USB hardware
    if(USB_DeviceState != DEVICE_STATE_Configured) return;

    usb_read_next_byte();
    usb_write_next_byte();
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void USB_SetupHardware(void){
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	/* Hardware Initialization */
	USB_Init();

	// *** MEGN540  ***
	// INITIALIZE RING BUFFERS AND OTHER DATA
    rb_initialize_C(&_usb_receive_buffer);
	rb_initialize_C(&_usb_send_buffer);
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void){
	/* Indicate USB enumerating */
	
	//Future LED MSG could be good here
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and CDC management tasks.
 */
void EVENT_USB_Device_Disconnect(void){
	/* Indicate USB not ready */
	
	//Future LED MSG could be good here
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the CDC management tasks are started.
 */
void EVENT_USB_Device_ConfigurationChanged(void){
	bool ConfigSuccess = true;

	/* Setup first CDC Interface's Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_TX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_RX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPADDR, EP_TYPE_INTERRUPT, CDC_NOTIFICATION_EPSIZE, 1);

	/* Reset line encoding baud rates so that the host knows to send new values */
	LineEncoding1.BaudRateBPS = 0;

}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void){
	/* Determine which interface's Line Coding data is being set from the wIndex parameter */
	void* LineEncodingData = &LineEncoding1;

	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(LineEncodingData, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearOUT();
			}

			break;
		case CDC_REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(LineEncodingData, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearIN();
			}

			break;
		case CDC_REQ_SetControlLineState:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
			}

			break;
	}
}


/** Function to manage CDC data transmission and reception to and from the host for the second CDC interface, which echoes back
 *  all data sent to it from the host.
 */
void USB_Echo_Task(void){
	/* Device must be connected and configured for the task to run */
	if(USB_DeviceState != DEVICE_STATE_Configured) return;

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Check to see if any data has been received */
	if(Endpoint_IsOUTReceived()){
		/* Create a temp buffer big enough to hold the incoming endpoint packet */
		uint8_t  Buffer[Endpoint_BytesInEndpoint()];

		/* Remember how large the incoming packet is */
		uint16_t DataLength = Endpoint_BytesInEndpoint();

		/* Read in the incoming packet into the buffer */
		Endpoint_Read_Stream_LE(&Buffer, DataLength, NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearOUT();

		/* Select the Serial Tx Endpoint */
		Endpoint_SelectEndpoint(CDC_TX_EPADDR);

		/* Write the received data to the endpoint */
		Endpoint_Write_Stream_LE(&Buffer, DataLength, NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();

		/* Wait until the endpoint is ready for the next packet */
		Endpoint_WaitUntilReady();

		/* Send an empty packet to prevent host buffering */
		Endpoint_ClearIN();
	}
}


/**
 * (non-blocking) Function usb_read_next_byte takes the next USB byte and reads it
 * into a ring buffer for latter processing.
 *
 */
void usb_read_next_byte(){
    // You'll need to take inspiration from the USB_Echo_Task above but
    // will need to adjust to make it non blocking. You'll need to dig into the library to understand
    // how the function above is working then interact at a slightly lower level, but still higher than
    // register level.

    /* Device must be connected and configured for the task to run */
	if(USB_DeviceState != DEVICE_STATE_Configured) return;

    /* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

    /* If selected OUT endpoint has received new packets AND there IS bytes in the endpoint,
    // add byte to the ring buffer and lengthen */
    if(Endpoint_IsOUTReceived() && Endpoint_BytesInEndpoint()){
        rb_push_back_C(&_usb_receive_buffer, Endpoint_Read_8());
    }

    /* If selected OUT endpoint has received new packets AND there is NOT any bytes in the endpoint,
    // finalize the stream transfer to send the last packet */
    if(Endpoint_IsOUTReceived() && !Endpoint_BytesInEndpoint()){
        Endpoint_ClearOUT();
    }
}

/**
 * (non-blocking) Function usb_write_next_byte takes the next byte from the output
 * ringbuffer and writes it to the USB port (if free).
 */
void usb_write_next_byte(){
    // You'll need to take inspiration from the USB_Echo_Task above but
    // will need to adjust to make it non blocking. You'll need to dig into the library to understand
    // how the function above is working then interact at a slightly lower level, but still higher than
    // register level.

    /* Device must be connected and configured for the task to run */
	if(USB_DeviceState != DEVICE_STATE_Configured) return;

	/* Select the Serial Tx Endpoint */
	Endpoint_SelectEndpoint(CDC_TX_EPADDR);

    /* If the selected IN endpoint IS ready for a new packet to be sent AND the send buffer has data*/
    if(Endpoint_IsINReady() && rb_length_C(&_usb_send_buffer) != 0){
        // Get size (in bytes) of the CDC data interface TX and RX data endpoint banks
        uint8_t tx_epsize_space_left = CDC_TXRX_EPSIZE;

        //If there is NOT data available to write (i.e., tx_epsize_space_left == 0)
        if(tx_epsize_space_left == 0){
            // Wait for endpoint to be ready for the next packet of data
            Endpoint_WaitUntilReady();
            // Send completed message to free up the endpoint for the next packet (prevents continued buffering)
            Endpoint_ClearIN();
        }

        // While there IS data available to write (i.e., tx_epsize_space_left != 0), write data
        while(tx_epsize_space_left && rb_length_C(&_usb_send_buffer)){
            // Pop off front of ring buffer & write
            Endpoint_Write_8(rb_pop_front_C(&_usb_send_buffer));
            // Decerment tx_epsize_space_left to control while loop
            tx_epsize_space_left--;
        }
        // Send completed message to free up the endpoint for the next packet (prevents continued buffering)
        Endpoint_ClearIN();
    }
}

/**
 * (non-blocking) Function usb_send_byte Adds a character to the output buffer
 * @param byte [uint8_t] Data to send
 */
void usb_send_byte(uint8_t byte){
	rb_push_back_C(&_usb_send_buffer,byte);
}

/**
 * (non-blocking) Function usb_send_data adds the data buffer to the output ring buffer.
 * @param p_data [void*] pointer to the data-object to be sent
 * @param data_len [uint8_t] size of data-object to be sent
 */
void usb_send_data(void* p_data, uint8_t data_len){
	char* data = p_data;
    for(uint8_t i=0;i<data_len;i++){
		rb_push_back_C(&_usb_send_buffer,data[i]);
	}
}

/**
 * (non-blocking) Function usb_send_str adds a c-style (null terminated) string to the output buffer
 * @param p_str [char*] Pointer to a c-string (null terminated) to send
 */
void usb_send_str(char* p_str){
    // Remember c-srtings are null terminated.
	uint8_t i = 0;
	while(p_str[i] != 0){
		rb_push_back_C(&_usb_send_buffer,p_str[i]);
		i++;
	}
    // Need to add 0 to the end to keep the Null character
    rb_push_back_C(&_usb_send_buffer,0);
}

/**
 * (non-blocking) Function usb_send_msg sends a message according to the MEGN540 USB message format.
 *      [MSG Length] [Format C-Str][Host Initiating CMD Char][DATA]
 *      MSG Length: [uint8_t] Number of bytes to follow in full message.
 *              Length of: Format C-String + 1 (CMD Char) + Length of DATA Array
 *      Format C-Str: [char*] null-terminated c-string defining the interpertation of the data bytes.
 *              Valid characters are defined in the Python3 struct defintion.
 *              Some examples are: c (char), b (int8_t), B (uint8_t), h (int16_t), f (float32)
 *      Host Initiating CMD Char: [char] The command sent from the host this message is in response to.
 *           All transmissions are host initiate.
 *      DATA: [Byte Array] Data byes that make up the message to be sent.
 *
 * @param format [c-str pointer] Pointer to interpertation string. e.g. ccf.  This alwasy starts with c because of the
 *          CMD char, here teh DATA object is then a char and a float.
 * @param cmd [char] Command this message is in respose to.
 * @param p_data [void*] pointer to the data-object to send.
 * @param data_len [uint8_t] size of the data-object to send. Remember sizeof() can help you with this!
 */
void usb_send_msg(char* format, char cmd, void* p_data, uint8_t data_len ){
    // Remember c-strings are null terminated. Use the above functions to help!

    // FUNCTION BEGIN
    //  Calculate the length of the format string taking advantage of the null-termination (+1 for null termination)
    //  Calculate the total message length:  1 + format_length + data_len
    //  Send data:
    //      usb_send_byte <-- length
    //      usb_send_str  <-- format
    //      usb_send_byte <-- cmd
    //      usb_send_data <-- p_data
    // FUNCTION END

    // Figure out the total length of message
    uint8_t msg_len = strlen(format)+1 + data_len+1;
    usb_send_byte(msg_len);
    usb_send_str(format);
    usb_send_byte(cmd);
    usb_send_data(p_data,data_len);
}

/**
 * (non-blocking) Funtion usb_msg_length returns the number of bytes in the receive buffer awaiting processing.
 * @return [uint8_t] Number of bytes ready for processing.
 */
uint8_t usb_msg_length(){
    return rb_length_C(&_usb_receive_buffer);
}

/**
 * (non-blocking) Function usb_msg_peek returns (without removal) the next byte in the
 * 
 * 
 
 
  receive buffer (null if empty).
 * @return [uint8_t] Next Byte
 */
uint8_t usb_msg_peek(){
    return rb_get_C(&_usb_receive_buffer,0); 
}

/**
 * (non-blocking) Function usb_msg_get removes and returns the next byte in the receive buffer (null if empty)
 * @return [uint8_t] Next Byte
 */
uint8_t usb_msg_get(){
    return rb_pop_front_C(&_usb_receive_buffer);
}

/**
 * (non-blocking) Function usb_msg_read_into populates the object with the data in the recieve buffer and
 * removes the bytes as they are read.  Returns false if receive buffer does not contain enough bytes to
 * fill the container and terminates without reading or removing.
 *
 * @param buff
 * @param data_len
 * @return [bool]  True: sucess, False: not enough bytes available
 */
bool usb_msg_read_into(void* p_obj, uint8_t data_len){
    if(usb_msg_length() < data_len) return false;
    
    char* msg = p_obj;
    for(uint8_t i=0;i<data_len;i++){
        msg[i] = rb_pop_front_C(&_usb_receive_buffer);
    }
    return true;
}

/**
 * (non-blocking) Function usb_flush_input_buffer sets the length of the recieve buffer to zero and disreguards
 * any bytes that remaining.
 */
void usb_flush_input_buffer(){
    rb_initialize_C(&_usb_receive_buffer);
}