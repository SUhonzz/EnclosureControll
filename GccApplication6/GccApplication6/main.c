/*
 * UART.c
 *
 * Created: 17.12.2018 13:01:38
 * Author : Admin
 */ 

#define F_CPU				16000000UL
#define UART_BAUD_RATE		9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>
#include "uart.h"
#include "oled_ssd1306.h"

char			UART_IN_BUF			[20+1]		;
char			out_buf				[20+1]		;
int16_t			c					= 0			;
int16_t			count				= 0			;
uint8_t			counting_var		= 0			;



#define VENT_OUT PC0
#define HEAT_OUT PC1
#define PTC_1IN ADC6
#define PTC2IN ADC7
#define PWM_OUT PC3
#define NOC0 PC2
#define SDA_DISP PC4
#define SCL_DISP PC5
#define RX_RASP PD0
#define TX_RASP PD1
#define IR_IN PD2
#define BUTTON1 PD3
#define BUTTON2 PD4
#define BUTTON3 PD5
#define NEOPIXEL_PWM PD6
#define NOC1 PD7
#define NOC2 PB0
#define NOC3 PB1



void UART_REC ()
{
	c = uart_getc();
	_delay_ms(15);
	if (!(c & UART_NO_DATA))
	{
		count = 0;
		while ((char) c != '\0')
		{
			if (count < 21 && (! (c & UART_NO_DATA)))
			{
				UART_IN_BUF[count++] = (char) c;
			}
			c = uart_getc();
		}
		UART_IN_BUF[count] = '\0';
	}
}

void UART_SEND_DATA()
{
	counting_var += 1;
	itoa(counting_var, out_buf, 10);
	uart_puts(out_buf);
	uart_putc('\0');
	oled_gotoxy(0,0);
	oled_write_str(out_buf);
	_delay_ms(1000);
}

void UART_SHOW_DATA()
{
	for (int i=0; i<21; i++) 
	{
		if (UART_IN_BUF[i] == '\0')
		{
			oled_gotoxy(i,0);
			oled_write_char(' ');
			UART_IN_BUF[i+1] = '\0';
		} 
		else 
		{
			oled_gotoxy(i,0); 
			oled_write_char(UART_IN_BUF[i]);
		}
	}
}

int main(void)
{
	DDRC |= (1<<0);                         // Relais
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
	oled_init();
	oled_gotoxy(0,6);
	//oled_write_str("sending");
	oled_write_str("receiving");

	sei();
	
    while (1) 
    {
		UART_REC ();		
		UART_SHOW_DATA();
		//UART_SEND_DATA();
    }
}

