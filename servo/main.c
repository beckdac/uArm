#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "main.h"
#include "servo.h"
#include "usiTwiSlave.h"

#define LED_ON		PORTD |= (1 << PORTD5)
#define LED_OFF		PORTD &= ~(1 << PORTD5)
#define LED_BLINK	PORTD ^= (1 << PORTD5)

// The lines that mention MCUSR_mirror are Michael Hennebry's 
// The rest, except this one are Clawson's 
unsigned char MCUSR_mirror __attribute__((section(".noinit"))); 
void early(void) __attribute__((section(".init3"), naked)); 
void early(void) {
	MCUSR_mirror=MCUSR; 
        // if the watchdog reset bit is set
        if (MCUSR & (1 << WDRF)) {
                // clear it
                MCUSR &= ~(1 << WDRF);
                // disable the watchdog so it doesn't go off during initialization
                wdt_disable();
	}
	LED_OFF;
}

// i2c read register callback
uint8_t i2c_read_register(uint8_t reg) {
	if (reg == REG_RESET) {
		return 0xff;
	} else if (reg / REG_ENABLE_BASE == 1) {
		return servo_is_enabled(reg % REG_ENABLE_BASE);
	} else if (reg / REG_DEGREES_BASE == 2) {
		return servo_get_degrees(reg % REG_DEGREES_BASE);
	} else if (reg / REG_TICKS_BASE == 3) {
		return (uint8_t)servo_get_pulse_length(reg % REG_TICKS_BASE);
	}
	return 0xff;	// return 0xff on bad register
}

// i2c write register callback
void i2c_write_register(uint8_t reg, uint8_t value) {
	if (reg == REG_RESET) {
		if (value > 0) {
			wdt_enable(WDTO_15MS);
			for (;;);
		} else {
			// do nothing when value == 0
		}
	} else if (reg / REG_ENABLE_BASE == 1) {
		return servo_enable(reg % REG_ENABLE_BASE);
	} else if (reg / REG_DEGREES_BASE == 2) {
		return servo_set_degrees(reg % REG_DEGREES_BASE, value);
	} else if (reg / REG_TICKS_BASE == 3) {
		return servo_set_pulse_length(reg % REG_TICKS_BASE, (uint8_t)value * 8);
	}
	// do nothing on bad register
	return;
}

int main(void) {
	DDRD |= (1 << DDD5);
	LED_ON;

	servo_init();
	usiTwiSlaveInit(I2C_SLAVE_ADDR, i2c_read_register, i2c_write_register);

	sei();

	while (1) {
		LED_ON;
		//cmd = usart_getc_wait();
		LED_OFF;
	}
}
