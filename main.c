/*
 * lab8.c
 *
 * Created: 7/12/2023 1:45:01 AM
 * Author : Marwan & Ahmed
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "TWI_Master.h"

//in TWI_Master.c, TWI_Start_Transceiver_With_Data() was altered to copy all messages passed to it regardless of whether they were read or writes

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1 

volatile uint8_t USART_TransmitBuffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // initialize USART receive buffer
volatile uint8_t tx_buff_index = 0;										//initialize transmit buffer index

unsigned char buffer[5];

unsigned char slaveAddress = 0x68;
unsigned char wakeupReg = 0x6B;
unsigned char z_axis_addr = 0x3F;

void USART_Init( unsigned int ubrr)
{
	//Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	// Enable receiver and transmitter
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	// Set frame format: 8data, 2stop bit
	UCSR0C |= (1<<USBS0)|(3<<UCSZ00);
}

void wakeUp(void){
	buffer[0] = (slaveAddress << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);	// send address byte as well as the write bit
	buffer[1] = wakeupReg;
	buffer[2] = 0x00;
	
	TWI_Start_Transceiver_With_Data(buffer, 3);
}

float absolute(float num){
	if(num < 0)
	return num * -1;
	else
	return num;
}

int main(void)
{
	DIDR0 |= (1 << ADC4D);
	cli();
	//initialize TWI
	TWI_Master_Initialise();
	
	//initialize USART
	USART_Init(MYUBRR);
	
	//enable USART interrupts
	UCSR0B |= 1 << RXCIE0;
	UCSR0B |= 1 << UDRIE0;
	
	sei();
	
	//send the byte to the power management register to take MPU out of sleep mode
	wakeUp();
	
	//disable USART transmission until message is constructed
	UCSR0B &= ~(1<<UDRIE0);
	
	int acc_val;						// stores contents of the accelerometer register
	float temp;							// stores value in gs as a floating point unit to get decimal places
	uint8_t whole = 0, dec0 = 0,
	dec1 = 0;						// whole number portion, and two decimal places
	
	while (1){

		buffer[0] = (slaveAddress << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT);	// send address byte as well as the write bit
		buffer[1] = z_axis_addr;												// send location of where we are reading from
		TWI_Start_Transceiver_With_Data(buffer, 2);								// this function takes care of sending the data and generating the necessary start and stop condition
		
		buffer[0] = (slaveAddress << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT);	// send address byte as well as the read bit
		TWI_Start_Transceiver_With_Data(buffer, 3);								// send the read request
		

		TWI_Get_Data_From_Transceiver(buffer, 3);								
		
		acc_val = (int) ((buffer[1] << 8) | buffer[2]);						// left shift MSBs by 8 and bit-wise or to construct the full 12 bit number
		temp = (float) (acc_val + 2703) / 16384 ;							// 16384 is given from the data sheet for this level of accuracy, the offset was calculated manually using some trial and error
		
		whole = (uint8_t) absolute(temp);									// integer portion
		dec0 = (float) (absolute(temp) - whole) * 10;						// decimal place 0
		dec1 = (int) ((float) (absolute(temp) - whole) * 100) % 10;			// decimal place 1
		

		if(temp > 0)
		{	// store numbers (as ascii characters) in the buffer, adding the '-' if the acc_val was negative, also adding the \n\r
			USART_TransmitBuffer[0] = (uint8_t) '-';
			USART_TransmitBuffer[1] = (uint8_t) whole + 48;
			USART_TransmitBuffer[2] = (uint8_t) '.';
			USART_TransmitBuffer[3] = (uint8_t) dec0 + 48;
			USART_TransmitBuffer[4] = (uint8_t) dec1 + 48;
			USART_TransmitBuffer[5] = 10;
			USART_TransmitBuffer[6] = 13;
		}
		else
		{
			USART_TransmitBuffer[0] = 0;
			USART_TransmitBuffer[1] = (uint8_t) whole + 48;
			USART_TransmitBuffer[2] = (uint8_t) '.';
			USART_TransmitBuffer[3] = (uint8_t) dec0 + 48;
			USART_TransmitBuffer[4] = (uint8_t) dec1 + 48;
			USART_TransmitBuffer[5] = 10;
			USART_TransmitBuffer[6] = 13;
		}
		
		UCSR0B |= (1 << UDRIE0);	// enable transmission
	}
}

// ISR for USART transmit
ISR(USART_UDRE_vect){

	// transmit first 7 bytes of the buffer
	// after 7 bits are transmitted disable transmit interrupt
	if(USART_TransmitBuffer[tx_buff_index] == 0) 
	{
		tx_buff_index++; // if its not a negative number, skip the first location
	}
	
	UDR0 = USART_TransmitBuffer[tx_buff_index++];
	if(tx_buff_index == 7)
	{ // send 7 bytes and disable interrupts again
		UCSR0B &= ~(1<<UDRIE0);
		tx_buff_index = 0;
	}
}

