#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>
#include "nrf24l01.h"


void setup_timer(void);
nRF24L01 *setup_rf(void);
uint16_t ReadADC(uint8_t ADCchannel);
volatile bool rf_interrupt = false;
volatile bool send_message = false;
uint16_t value = 0;
int main(void) {
	uint8_t to_address[5] = { 0x01, 0x01, 0x01, 0x01, 0x01 };
	bool on = false;
	
	DDRC |= 0b00001100;
	PORTC &= 0b11110011;
	
	init_ADC();
	uart_init(9600);
	start_msg();
	
	
	sei();
	nRF24L01 *rf = setup_rf();
	setup_timer();
	set_tx_address(rf,to_address);
	
	
	while (true) {
		
		if (rf_interrupt) {
			rf_interrupt = false;
			
			int success = nRF24L01_transmit_success(rf);
			
			if (success != 0){
				serial_writeln("Buffer FULL..");
				nRF24L01_flush_transmit_message(rf);
				PORTC |= 0b00001000;
				_delay_ms(150);
				PORTC &= 0b11110111;
			}
			else{
				nRF24L01_flush_transmit_message(rf);
				
				serial_writeln("MSG Sent...");
				
				PORTC |= 0b00000100;
				_delay_ms(150);
				PORTC &= 0b11111011;
			}
		}

		if (send_message) {
			send_message = false;
			on = !on;
			nRF24L01Message msg;
			
			char X[2];
			char Y[2];
			
			value = ReadADC(0);
			
			value = value/102.4;
			sprintf(X, "%d",value);
			
			value = ReadADC(1);
			value = value/102.4;
			
			sprintf(Y, "%d",value);
			
			strcat(X,Y);
				
			memcpy(msg.data, X, 3);
			
			
			serial_write("MSG DATA: ");
			serial_writeln(msg.data);
			msg.length = strlen((char *)msg.data) + 1;
			nRF24L01_transmit(rf, to_address, &msg);
		}
		
		
	}

	return 0;
}



void init_ADC(){
	
	// Select Vref=AVcc
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADEN);
	
	
}

uint16_t ReadADC(uint8_t ADCchannel)
{ uint16_t val;
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	val = ADC;
	
	return val;
}
nRF24L01 *setup_rf(void) {
	nRF24L01 *rf = nRF24L01_init();
	rf->ss.port = &PORTB;
	rf->ss.pin = PB2;
	rf->ce.port = &PORTB;
	rf->ce.pin = PB1;
	rf->sck.port = &PORTB;
	rf->sck.pin = PB5;
	rf->mosi.port = &PORTB;
	rf->mosi.pin = PB3;
	rf->miso.port = &PORTB;
	rf->miso.pin = PB4;
	// interrupt on falling edge of INT0 (PD2)
	EICRA |= _BV(ISC01);
	EIMSK |= _BV(INT0);
	nRF24L01_begin(rf);
	return rf;
}

// setup timer to trigger interrupt every second when at 1MHz
void setup_timer(void) {
	TCCR1B |= _BV(WGM12);
	TIMSK1 |= _BV(OCIE1A);
	OCR1A = 10000;
	TCCR1B |= _BV(CS12);
}

// each one second interrupt
ISR(TIMER1_COMPA_vect) {
	send_message = true;
}

// nRF24L01 interrupt
ISR(INT0_vect) {
	serial_writeln("Interrupt called...");
	rf_interrupt = true;
}
