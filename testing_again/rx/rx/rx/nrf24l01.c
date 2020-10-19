#define F_CPU 8000000UL
#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"

void uart_init(int baud_rate);
void serial_write(unsigned char data[]);
void serial_writeln(unsigned char data[]);
void serial_putch(unsigned char value);
static void copy_address(uint8_t *source, uint8_t *destination);
inline static void set_as_output(gpio_pin pin);
inline static void set_high(gpio_pin pin);
inline static void set_low(gpio_pin pin);
static void spi_init(nRF24L01 *rf);
static uint8_t spi_transfer(uint8_t data);
void show_board_info(nRF24L01 *rf);


uint8_t temp;
uint8_t status;

nRF24L01 *nRF24L01_init(void) {
    nRF24L01 *rf = malloc(sizeof(nRF24L01));
    memset(rf, 0, sizeof(nRF24L01));
    return rf;
}

void nRF24L01_begin(nRF24L01 *rf) {
	
	serial_writeln("Setting NRF24L01+ begin function...");
    set_as_output(rf->ss);
    set_as_output(rf->ce);

    set_high(rf->ss);
    set_low(rf->ce);
	serial_writeln("CE LOW");
	
	serial_writeln("Initiating SPI comm...");
    spi_init(rf);

    nRF24L01_send_command(rf, FLUSH_RX, NULL, 0);
    nRF24L01_send_command(rf, FLUSH_TX, NULL, 0);
    nRF24L01_clear_interrupts(rf);

    uint8_t data;
    data = _BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP) | _BV(PRIM_RX);
	
	serial_writeln("Writing CONFIG register in RX..");
	_delay_ms(200);
	
    nRF24L01_write_register(rf, CONFIG, &data, 1);
	
	
	
	status = nRF24L01_read_register(rf, CONFIG, &temp , 1);
	show_register_val(temp,"CONFIG:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	serial_writeln("Going ahead....");
	
    // enable Auto Acknowlegde on all pipes
    data = _BV(ENAA_P0) | _BV(ENAA_P1) | _BV(ENAA_P2)
         | _BV(ENAA_P3) | _BV(ENAA_P4) | _BV(ENAA_P5);
    nRF24L01_write_register(rf, EN_AA, &data, 1);
	
	serial_writeln("Enabling auto-acknowledgment...");
	status = nRF24L01_read_register(rf, EN_AA, &temp , 1);
	show_register_val(temp,"ENAA:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	
    // enable Dynamic Payload on al pipes
    data = _BV(DPL_P0) | _BV(DPL_P1) | _BV(DPL_P2)
         | _BV(DPL_P3) | _BV(DPL_P4) | _BV(DPL_P5);
    nRF24L01_write_register(rf, DYNPD, &data, 1);
	
	serial_writeln("Enabling dynamic-payload length...");
	status = nRF24L01_read_register(rf, DYNPD, &temp , 1);
	show_register_val(temp,"DYNPD:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	

    // enable Dynamic Payload (global)
    data = _BV(EN_DPL);
    nRF24L01_write_register(rf, FEATURE, &data, 1);
	
	serial_writeln("Enabling GLOBAL-DYN length...");
	status = nRF24L01_read_register(rf, FEATURE, &temp , 1);
	show_register_val(temp,"FEATURE:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);

    // disable all rx addresses
    data = 0;
    nRF24L01_write_register(rf, EN_RXADDR, &data, 1);
	
	serial_writeln("Disabling all RX_ADDRESS...");
	status = nRF24L01_read_register(rf, EN_RXADDR, &temp , 1);
	show_register_val(temp,"FEATURE:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
}

uint8_t nRF24L01_send_command(nRF24L01 *rf, uint8_t command, void *data,
    size_t length) {
    set_low(rf->ss);

    rf->status = spi_transfer(command);
    for (unsigned int i = 0; i < length; i++)
        ((uint8_t*)data)[i] = spi_transfer(((uint8_t*)data)[i]);

    set_high(rf->ss);

    return rf->status;
}

uint8_t nRF24L01_write_register(nRF24L01 *rf, uint8_t reg_address, void *data,
    size_t length) {
    return nRF24L01_send_command(rf, W_REGISTER | reg_address, data, length);
}

uint8_t nRF24L01_read_register(nRF24L01 *rf, uint8_t reg_address, void *data,
    size_t length) {
    return nRF24L01_send_command(rf, R_REGISTER | reg_address, data, length);
}

uint8_t nRF24L01_no_op(nRF24L01 *rf) {
    return nRF24L01_send_command(rf, NOP, NULL, 0);
}

uint8_t nRF24L01_update_status(nRF24L01 *rf) {
    return nRF24L01_no_op(rf);
}

uint8_t nRF24L01_get_status(nRF24L01 *rf) {
    return rf->status;
}

bool nRF24L01_data_received(nRF24L01 *rf) {
    set_low(rf->ce);
	serial_writeln("CE LOW");
    nRF24L01_update_status(rf);
    return nRF24L01_pipe_number_received(rf) >= 0;
}

void nRF24L01_listen(nRF24L01 *rf, int pipe, uint8_t *address) {
    uint8_t addr[5];
	
	
	//serial_writeln("Listening....");
	
	
    copy_address(address, addr);

    nRF24L01_write_register(rf, RX_ADDR_P0 + pipe, addr, 5);
	
	//serial_writeln("Writing address to RX_ADDR_PO");
	
	
	uint8_t read_addr[5];
	
	//status = nRF24L01_read_register(rf,  RX_ADDR_P0 + pipe, read_addr, 5);
	//serial_writeln("Reading written address..");
	/*
	for (char x = 0; x < 5; x++){
		serial_putch(x);
		show_register_val(read_addr[x],"   RX_ADDR_P0:  ");
		
	}
	
	*/
	
	
	//show_register_val(status,"STATUS:  ");
	//serial_writeln("------------------\n\r");
	//_delay_ms(500);
	
	

    uint8_t current_pipes;
    nRF24L01_read_register(rf, EN_RXADDR, &current_pipes, 1);
    current_pipes |= _BV(pipe);
    nRF24L01_write_register(rf, EN_RXADDR, &current_pipes, 1);
	
	
	
	
	
	
	
	//serial_writeln("Enabling given Pipe-0");
	
	//status = nRF24L01_read_register(rf, EN_RXADDR, &temp , 1);
	//show_register_val(temp,"EN_RXADDR:  ");
	//show_register_val(status,"STATUS:  ");
	//serial_writeln("------------------\n\r");
	//_delay_ms(500);
	
	
	//serial_writeln("Reading CONFIG register..");
	
	//status = nRF24L01_read_register(rf, CONFIG, &temp , 1);
	//show_register_val(temp,"CONFIG:  ");
	//show_register_val(status,"STATUS:  ");
	//serial_writeln("------------------\n\r");
	_delay_ms(20);
	
	
	
	
	
	
    set_high(rf->ce);
	serial_writeln("CE HIGH");
}

void show_board_info(nRF24L01 *rf){
	serial_writeln("Logging info from the board..");
	
	
	serial_writeln("Reading CONFIG register..");
	
	status = nRF24L01_read_register(rf, CONFIG, &temp , 1);
	show_register_val(temp,"CONFIG:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	serial_writeln("Reading SETUP_RETR register..");
	
	status = nRF24L01_read_register(rf, SETUP_RETR, &temp , 1);
	show_register_val(temp,"SETUP_RETR:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	serial_writeln("Reading RF_CH register..");
	
	status = nRF24L01_read_register(rf, RF_CH, &temp , 1);
	show_register_val(temp,"RF_CH:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	serial_writeln("Reading RF_SETUP register..");
	
	status = nRF24L01_read_register(rf, RF_SETUP, &temp , 1);
	show_register_val(temp,"RF_SETUP:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	
}

bool nRF24L01_read_received_data(nRF24L01 *rf, nRF24L01Message *message) {
    message->pipe_number = nRF24L01_pipe_number_received(rf);
    nRF24L01_clear_receive_interrupt(rf);
    if (message->pipe_number < 0) {
        message->length = 0;
        return false;
    }

    nRF24L01_read_register(rf, R_RX_PL_WID, &message->length, 1);

    if (message->length > 0) {
        nRF24L01_send_command(rf, R_RX_PAYLOAD, &message->data,
            message->length);
    }

    return true;
}

int nRF24L01_pipe_number_received(nRF24L01 *rf) {
    int pipe_number = (rf->status & RX_P_NO_MASK) >> 1;
    return pipe_number <= 5 ? pipe_number : -1;
}

void nRF24L01_transmit(nRF24L01 *rf, void *address, nRF24L01Message *msg) {
	
	serial_writeln("Sending msg...");
    nRF24L01_clear_transmit_interrupts(rf);
    uint8_t addr[5];
	
	serial_writeln("Writing TX_ADDR..");
	
	
    copy_address((uint8_t *)address, addr);
    nRF24L01_write_register(rf, TX_ADDR, addr, 5);
	
	uint8_t read_addr[5];
	
	/*
	status = nRF24L01_read_register(rf,  TX_ADDR , read_addr, 5);
	serial_writeln("Reading written address..");
	
	for (char x = 0; x < 5; x++){
		serial_putch(x+'0');
		show_register_val(read_addr[x],"   TX_ADDR:  ");
		
	}
	
	serial_writeln("Writing same address to RX_ADDR_P0 too:");
	*/
	
	
    copy_address((uint8_t *)address, addr);
    nRF24L01_write_register(rf, RX_ADDR_P0, addr, 5);
	
	/*
	status = nRF24L01_read_register(rf,  RX_ADDR_P0 , read_addr, 5);
	serial_writeln("Reading written address..");
	
	for (char x = 0; x < 5; x++){
		serial_putch(x+'0');
		show_register_val(read_addr[x],"   RX_ADDR_P0:  ");
		
	}
	*/
	
    nRF24L01_send_command(rf, W_TX_PAYLOAD, &msg->data, msg->length);
	
	
    uint8_t config;
    nRF24L01_read_register(rf, CONFIG, &config, 1);
    config &= ~_BV(PRIM_RX);
	
	serial_writeln("Writing TX to CONFIG..");
    nRF24L01_write_register(rf, CONFIG, &config, 1);
	
	
	status = nRF24L01_read_register(rf, CONFIG, &temp , 1);
	show_register_val(temp,"CONFIG:  ");
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
    set_high(rf->ce);
	serial_writeln("CE HIGH");
}

int nRF24L01_transmit_success(nRF24L01 *rf) {
    set_low(rf->ce);
	serial_writeln("CE LOW");
	
    nRF24L01_update_status(rf);
	
	serial_writeln("Reading STATUS register....");
	status = nRF24L01_read_register(rf, STATUS, &temp , 1);
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
	
    int success;
    if (rf->status & _BV(TX_DS)) success = 0;
    else if (rf->status & _BV(MAX_RT)) success = -1;
    else success = -2;
	
	serial_writeln("Clearing TX interrupts..");
	
	
	
    nRF24L01_clear_transmit_interrupts(rf);
	
	
	serial_writeln("Reading STATUS register AGAIN....");
	status = nRF24L01_read_register(rf, STATUS, &temp , 1);
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
	
    uint8_t config;
    nRF24L01_read_register(rf, CONFIG, &config, 1);
	
	serial_writeln("Reading CONFIG register (RX=0)....");
	status = nRF24L01_read_register(rf, STATUS, &temp , 1);
	show_register_val(status,"STATUS:  ");
	serial_writeln("------------------\n\r");
	_delay_ms(500);
	
    config |= _BV(PRIM_RX);
    nRF24L01_write_register(rf, CONFIG, &config, 1);
	set_high(rf->ce);
	serial_writeln("CE HIGH");
    return success;
}

void nRF24L01_flush_transmit_message(nRF24L01 *rf) {
    nRF24L01_send_command(rf, FLUSH_TX, NULL, 0);
}

void nRF24L01_retry_transmit(nRF24L01 *rf) {
    // XXX not sure it works this way, never tested
    uint8_t config;
    nRF24L01_read_register(rf, CONFIG, &config, 1);
    config &= ~_BV(PRIM_RX);
    nRF24L01_write_register(rf, CONFIG, &config, 1);
    set_high(rf->ce);
	serial_writeln("CE HIGH");
}

void nRF24L01_clear_interrupts(nRF24L01 *rf) {
    uint8_t data = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);
    nRF24L01_write_register(rf, STATUS, &data, 1);
}

void nRF24L01_clear_transmit_interrupts(nRF24L01 *rf) {
    uint8_t data = _BV(TX_DS) | _BV(MAX_RT);
    nRF24L01_write_register(rf, STATUS, &data, 1);
}

void nRF24L01_clear_receive_interrupt(nRF24L01 *rf) {
    uint8_t data = _BV(RX_DR) | rf->status;
    nRF24L01_write_register(rf, STATUS, &data, 1);
}

static void copy_address(uint8_t *source, uint8_t *destination) {
    for (int i = 0; i < 5; i++)
        destination[i] = source[i];
}

inline static void set_as_output(gpio_pin pin) {
    volatile uint8_t *ddr = pin.port - 1;
    *ddr |= _BV(pin.pin);
}

inline static void set_as_input(gpio_pin pin) {
    volatile uint8_t *ddr = pin.port - 1;
    *ddr &= ~_BV(pin.pin);
}

inline static void set_high(gpio_pin pin) {
    *pin.port |= _BV(pin.pin);
}

inline static void set_low(gpio_pin pin) {
    *pin.port &= ~_BV(pin.pin);
}

static void spi_init(nRF24L01 *rf) {
    // set as master
    SPCR |= _BV(MSTR);
    // enable SPI
    SPCR |= _BV(SPE);
    // MISO pin automatically overrides to input
    set_as_output(rf->sck);
    set_as_output(rf->mosi);
    set_as_input(rf->miso);
    // SPI mode 0: Clock Polarity CPOL = 0, Clock Phase CPHA = 0
    SPCR &= ~_BV(CPOL);
    SPCR &= ~_BV(CPHA);
    // Clock 2X speed
    SPCR &= ~_BV(SPR0);
    SPCR &= ~_BV(SPR1);
    SPSR |= _BV(SPI2X);
    // most significant first (MSB)
    SPCR &= ~_BV(DORD);
}

static uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}




void start_msg(){
	
	serial_writeln("Welcome to atmega328P");
	serial_write("Running onboard-diagnostics");
	
	for (int i = 0; i<10; i++){
		serial_putch('.');
		_delay_ms(400);
	}
	serial_writeln("");
	
}
void uart_init(int baud_rate){
	
	//int value =     ((freq)/(16*baud_rate)) - 1;
	int value = 51;
	UBRR0H = (value>>8);
	UBRR0L = (value);
	
	UCSR0C = 0x06;       /* Set frame format: 8data, 1stop bit  */
	UCSR0B = (1<<TXEN0); /* Enable  transmitter                 */
}

void serial_write(unsigned char data[]){
	int length = strlen(data);
	int i = 0;
	for (i = 0; i < length ; i++){
		UDR0 = data[i];
		while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
		
		
	}
	
}

void serial_writeln(unsigned char data[]){
	int length = strlen(data);
	int i = 0;
	for (i = 0; i < length ; i++){
		UDR0 = data[i];
		while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
		
		
	}
	
	serial_write("\n\r");
	
}

void serial_putch(unsigned char value){
	UDR0 = value;
	while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
	
}

void binary(uint8_t value){
	
	unsigned char bin[9] ;
	uint8_t i = 0;
	for (i = 0; i < 8 ; i++){
		
		if (value%2){
			bin[i] = '1';
		}
		else{
			bin[i] = '0';
			
		}
		value = value / 2;
	}
	strrev(bin);
	serial_writeln(bin);
	
}

void show_register_val(uint8_t reg,char name[]){
	serial_write(name);
	binary(reg);
}