/*
 * UART.c
 *
 * Created: 17.12.2018 13:01:38
 * Author : Admin
 */ 

#define F_CPU				16000000UL
#define UART_BAUD_RATE		9600

// DEBUG
//#define debug
#ifdef debug
	#define log(...) oled_write(__VA_ARGS__) // Wenn DEBUG definiert ist, f�hrt log() oled_write aus
#else
	#define log(...) // Wenn DEBUG nicht definiert ist, macht log() nichts
#endif

// INCLUDES
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
#include "uart.h"
#include "oled_ssd1306.h"



// DEFINITIONS
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

// PINS
typedef struct {		// function for creating a pin-struct out of a string (e.g. "PC2")
	volatile uint8_t *port;	// pointer to the PORT-register
	uint8_t pin;			// bit mask for the Pin
} Pin;
Pin createPin(const char* pinString) {	// function, to set a pin to HIGH
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
void setPinHigh(Pin p) {	// function, to set a pin to LOW
	*(p.port) |= (1 << p.pin);  // sets the corresponding bit
}
void setPinLow(Pin p) {
	*(p.port) &= ~(1 << p.pin);  // deletes the corresponding bit
}
// works

// UART
char			UART_IN_BUF			[20+1]		;
char			out_buf				[20+1]		;
int16_t			c					= 0			;
int16_t			count				= 0			;
uint8_t			counting_var		= 0			;
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

// PWM
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

// ADC
void ADC_INIT()
{
	ADMUX	|= ((1<<MUX1)	|	(1<<MUX2));
	ADMUX	|= (1<<REFS0);	//Voltage Reference: AVCC with external capacitor at AREF pin
	ADCSRA	|= ((1<<ADPS0)	|	(1<<ADPS1)	|	(ADPS2));	//Prescaler Division Factor: 128
	ADCSRA	|= (1<<ADEN);
	ADCSRA	|= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)) {};	//trash first value
}
void ADC_READ()
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
	
}

// BUTTONS
bool S1 = false;
bool S2 = false;
bool S3 = false;
void BUTTON_CHECK()
{
	if (PIND & (1 << BUTTON1))
	{
		S1 = true;
		oled_gotoxy(0,3);
		log("button 1 pressed");
		while (PIND & (1 << BUTTON1)){}
	}
	if (PIND & (1 << BUTTON2))
	{
		S2 = true;
		oled_gotoxy(0,3);
		log("button 2 pressed");
		while (PIND & (1 << BUTTON1)){}
	}
	if (PIND & (1 << BUTTON3))
	{
		S3 = true;
		oled_gotoxy(0,3);
		log("button 3 pressed");
		while (PIND & (1 << BUTTON1)){}
	}
}



void write_pos_A(char* str){
	posx = 0;
	posy = 0;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}

void write_pos_B(char* str){
	posx = 6;
	posy = 0;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}

void write_pos_C(char* str){
	posx = 8;
	posy = 0;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}

void write_pos_D(char* str){
	posx = 8;
	posy = 1;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}

void write_pos_E(char* str){
	posx = 2;
	posy = 3;
	oled_gotoxy(posx,posy);
	oled_font_size(1);
	oled_write("%s", str);
	oled_font_size(0);
}

void write_pos_F(char* str){
	posx = 0;
	posy = 6;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}

void write_pos_G(char* str){
	posx = 15;
	posy = 0;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}
void write_help()
{
	oled_gotoxy(15,0);
	oled_write("+");
	oled_gotoxy(15,3);
	oled_write("-");
	oled_gotoxy(12,6);
	oled_write("Mode");
}

void write_pos_H(char* str){
	posx = 15;
	posy = 3;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}

void write_pos_I(char* str){
	posx = 12;
	posy = 6;
	oled_gotoxy(posx,posy);
	oled_write("%s", str);
}
//works

// OLED
/*void write_name(char* name) {
	static char scroll_name[100]; // Static array to store the name for scrolling
	static int offset = 0;         // Offset for scrolling position

	// If name length is greater than 6, prepare the scroll_name array
	if (strlen(name) > 6) {
		// Fill the scroll_name array with the name followed by a space
		strcpy(scroll_name, name);
		strcat(scroll_name, " "); // Append a space at the end
		} else {
		oled_write(name);  // Display the name directly if length is <= 6
		return;
	}

	// Rotate the name for scrolling (move one character each time)
	oled_write(&scroll_name[offset]);  // Display the current "slice" of the name

	// Update the offset for next scroll
	offset++;
	if (scroll_name[offset] == '\0') {
		offset = 0;  // Reset the offset if we reach the end
	}
}*/





int main(void)
{
	
	DDRC	|=	(1<<0);		// VENTOUT RELAIS
	DDRC	|=	(1<<1);		// HEATOUT	
	int cnt = 0;
	// Inits
    //uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
	oled_init();
	ADC_INIT();
	PWM_INIT();
	
	// Pins
	Pin VO = createPin("PD6");
	
	// Menu
	char name[] = "3Dprinter";
	int mode = 0;	// 0: automatic, 1: manual
	void toggle_mode() {
		mode = (mode + 1) % 2;  // Toggle between 0 and 1
		// %(max_mode + 1)
		S1 = false;
	}
	int state = 0;	// 0: Heat, 1: Vent, 2: Off
	int temp_set = 22;
	
	log("on");
	
	sei();
	
    while (1) 
    {

		/*setPinHigh(VO);
		_delay_ms(100);
		setPinLow(VO);
		_delay_ms(100);*/
		
		//ADC_READ_PRINT();	//temperature

		//OCR0A = 30;
		
		cnt++;
		if (cnt > 1000)
		{
			ADC_READ();
			cnt = 0;
		}
		
		
		string t = "22�C";
		oled_gotoxy(0,0);
		oled_write("%s", t);
		// Display Menu
		
		write_pos_A("3D"); //NAME
		
		oled_gotoxy(6,0);	// Mode
		if (mode == 0)
			write_pos_B("A");
		else
			write_pos_B("M");

		write_pos_C("%d", PTC1_VAL); //TEMP1
		
		write_pos_D("%d", PTC2_VAL); //TEMP2

		write_pos_E("%u",(PTC1_VAL+PTC2_VAL)/2); //Med Temp

	if (state = 0){
		write_pos_F("Heat"); //state
	}
	else if(state = 1) {
		write_pos_F("Vent"); //state
	}
	else write_pos_F("Off"); //state
	
	write_help();
		
		BUTTON_CHECK();
		if (S1 == true)
			toggle_mode();
			
		switch (mode) {
			case 0:					//automatic
				if (S2 == true) {
					temp_set += 1;
					S2 = false;
				}else if (S3 == true) {
					temp_set -= 1;
					S3 = false;
				}else break;
			case 1:					//manual
				if (S2 == true) {
					heat = true;
					vent = false;
					S2 = false;
				}else if (S3 == true) {
					heat = false;
					vent = true;
					S3 = false;
				}else break;
				oled_clear_row(7);
			default:
				break;
		}
    }
}

