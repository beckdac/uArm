#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "global.h"
#include "usart.h"

FILE usart_output = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE usart_input = FDEV_SETUP_STREAM(NULL, usart_getchar, _FDEV_SETUP_READ);

void usart_init(uint16_t ubrr) {
	/* clear out the registers as a fix up from the state optiboot left the usart in */
	UCSR0A = 0;
	UCSR0B = 0;	// not really necessary, see below
	UCSR0C = 0;	// not really necessary, see below

	/* Set the baud rate */
	UBRR0H = (ubrr >> 8);
   	UBRR0L = ubrr;

	/* enable transmit and receive circuitry */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	/* set to 8 data bits, 1 stop bit */
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	/* complete byte receive interrupt */
	UCSR0B |= (1 << RXCIE0);

	/* the UDRE interrupt is enabled on send */

	/* setup FDEV */
	stdout = &usart_output;
	stdin  = &usart_input;
}

/* no software buffer send and receive */
uint8_t usart_receive(void) {
	/* wait for incomming data */
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void usart_send(uint8_t data) {
	/* wait for empty transmit buffer */
	while (!(UCSR0A & (1 << UDRE0)));
	/* start transmittion */
	UDR0 = data;
}

/* for use with FDEV */
int usart_putchar(char c, FILE *stream) {
	if (c == '\n') {
		usart_putchar('\r', stream);
	}
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;

	return 1;
}

int usart_getchar(FILE *stream) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}
