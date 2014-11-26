#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"

uint8_t buf[32];

int main(void) {
	uint8_t c, buf_i;

	// disable the watchdog timer, this is necessary if the timer was used to reset the device
	MCUSR &= ~(1<<WDRF);
	wdt_disable();

	usart_init(BAUD_TO_UBRR(USART_BAUD));
	printf_P(PSTR("hello!\n"));

	// turn on interrupts
	sei();

	while (1) {
		// use not interrupt to provide maximum deference to servo timer
		c = usart_receive();
		if (c == 3) { // Control-C, reset state
			state = ADDRESS;
			continue;
		}
		// depending on state, process command
		switch (state) {
			case ADDRESS:	// fresh command
				if (c >= 'A' && c <= 'Z')
					addr = c - 'A';
				else if (c >= 'a' && c <= 'z')
					addr = (c - 'a');
				addr += 'A';
				buf_i = 0;
				buf[0] = '\0';
				switch (addr) {
					case 'G':
						printf_P(PSTR("G command!\n"));
						state = REGISTER;
						break;
					case 'M':
						printf_P(PSTR("M command!\n"));
						state = REGISTER;
						break;
					default:	// unknown command address
						printf_P(PSTR("invalid command address!\n"));
						state = ADDRESS;
						break;
				};
				break;
			case REGISTER:	// command #
				if (c >= '0' && c <= '9') {
					buf[buf_i++] = c;
				} else {
					reg = atoi(buf);
					state = PARAMETERS;
				}
				break;
			case PARAMETERS: // optional parameters for a command
				if (c == '\n' || c == '\r') {
				}
				break;
			default:
				// on bad state, try a restart
				wdt_enable(WDT_15ms);
				for (;;);
				break;
		};
	}
}
