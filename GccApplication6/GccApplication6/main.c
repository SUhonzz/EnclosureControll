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
int PTC1_VAL;	//Temp. 1
#define PTC2IN ADC7
int PTC2_VAL;	//Temp. 2
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

typedef struct {
	volatile uint8_t *port;  // pointer to the PORT-register
	uint8_t pin;             // bit mask for the Pin
} Pin;
// function for creating a pin-struct out of a string (e.g. "PC2")
Pin createPin(const char* pinString) {
	Pin p;

	// checks, which port the pin-string contains
	switch (pinString[1]) {
		case 'B':
		p.port = &PORTB;
		break;
		case 'C':
		p.port = &PORTC;
		break;
		case 'D':
		p.port = &PORTD;
		break;
		default:
		// error handling, if an invalid port is provided
		p.port = NULL;
		return p;
	}

	// extract the Pin-Index from the string, e.g. from "PC2" the 2
	p.pin = pinString[2] - '0';

	return p;
}
// function, to set a pin to HIGH
void setPinHigh(Pin p) {
	*(p.port) |= (1 << p.pin);  // sets the corresponding bit
}
// function, to set a pin to LOW
void setPinLow(Pin p) {
	*(p.port) &= ~(1 << p.pin);  // deletes the corresponding bit
}
// works

void UART_REC()
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
	//_delay_ms(1000);
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

void PWM_INIT()
{
	TCCR0A |= ((1<<WGM00)	|	(1<<WGM01));	// Fast PWM 8Bit
	TCCR0A |= ((1<<COM0A0)	|	(1<<COM0A1));	// Inverting Mode - common anode
	TCCR0A |= ((1<<CS00)	|	(1<<CS02));		// Prescaler 1024
}

void TIMER_INIT()
{
	TCCR1A = 0;									// normal mode
	TCCR1B |= ((1<<CS11)	|	(1<<CS10));		// Prescaler 64
	TIMSK1 |= (1<<TOIE1);						// Enable Timer1 overflow interrupt
}

ISR(TIMER1_OVF_vect)
{
	Pin LED = createPin("PC3");
	uint8_t pwm_duty_cycle = 128; // 50% duty cycle if 0-255 range
	uint16_t pwm_period = 255;     // Full period
	static uint16_t pwm_counter = 0;
	
    if (pwm_counter < pwm_duty_cycle) {
	    setPinHigh(LED); // Set pin high
	    } else {
	    setPinLow(LED); // Set pin low
    }

    pwm_counter++;
    if (pwm_counter >= pwm_period) {
	    pwm_counter = 0; // Reset the counter at the end of the period
    }
}

void ADC_INIT()
{
	ADMUX	|= ((1<<MUX1)	|	(1<<MUX2));
	ADMUX	|= (1<<REFS0);	//Voltage Reference: AVCC with external capacitor at AREF pin
	ADCSRA	|= ((1<<ADPS0)	|	(1<<ADPS1)	|	(ADPS2));	//Prescaler Division Factor: 128
	ADCSRA	|= (1<<ADEN);
	ADCSRA	|= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)) {};	//trash first value
}

void ADC_READ_PRINT()
{
		//ADC6
	ADMUX	|=	((1<<MUX1)	|	(1<<MUX2));	// clear and select channel
	ADCSRA	|=	(1<<ADSC);					// start conversion
	while (ADCSRA & (1<<ADSC)) {};			// wait til finish conversion
	PTC1_VAL = (ADCW * 5.0 / 1024.0) * 100;	// convert temperature
		//ADC7
	ADMUX	|=	((1<<MUX1)	|	(1<<MUX2)	|	(1<<MUX3));
	ADCSRA	|=	(1<<ADSC);
	while (ADCSRA & (1<<ADSC)) {};
	PTC2_VAL = (ADCW * 5.0 / 1024.0) * 100;
		//output, display
	oled_font_size(3);
	oled_gotoxy(1, 1);
	oled_write("Temp 1: %i", PTC1_VAL);
	oled_gotoxy(1, 5);
	oled_write("Temp 2: %i", PTC2_VAL);
	int avg = (PTC1_VAL + PTC2_VAL)/2;
	oled_write("average: %i", avg);
}

void BUTTON_CHECK()
{
	if (PIND & (1 << BUTTON1))
	{
		oled_gotoxy(0,3);
		oled_write("button 1 pressed");
	}
	if (PIND & (1 << BUTTON2))
	{
		oled_gotoxy(0,3);
		oled_write("button 2 pressed");
	}
	if (PIND & (1 << BUTTON3))
	{
		oled_gotoxy(0,3);
		oled_write("button 3 pressed");
	}
}
//works

int main(void)
{
	
	DDRC	|=	(1<<0);		// VENTOUT RELAIS
	DDRC	|=	(1<<1);		// HEATOUT	
    //uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
	oled_init();
	ADC_INIT();
	PWM_INIT();
	Pin VO = createPin("PD6");
		oled_write("on");

	
	sei();
	
    while (1) 
    {
		
		setPinHigh(VO);
		_delay_ms(100);
		setPinLow(VO);
		_delay_ms(100);
		
		//ADC_READ_PRINT();	//temperature

		//OCR0A = 30;
		
		BUTTON_CHECK();

    }
}

