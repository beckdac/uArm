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
	switch (reg) {
		case REG_RESET:
			// this register always returnes 0xff
			return 0xff;
			break;
		case REG_SERVO_0_ENABLE:
			return servo_is_enabled(0);
			break;
		case REG_SERVO_0_DEGREES:
			return servo_get_degrees(0);
			break;
		case REG_SERVO_1_ENABLE:
			return servo_is_enabled(1);
			break;
		case REG_SERVO_1_DEGREES:
			return servo_get_degrees(1);
			break;
		case REG_SERVO_2_ENABLE:
			return servo_is_enabled(2);
			break;
		case REG_SERVO_2_DEGREES:
			return servo_get_degrees(2);
			break;
		case REG_SERVO_3_ENABLE:
			return servo_is_enabled(3);
			break;
		case REG_SERVO_3_DEGREES:
			return servo_get_degrees(3);
			break;
		default:
			// return 0xff on bad register
			return 0xff;
	}
}

// i2c write register callback
void i2c_write_register(uint8_t reg, uint8_t value) {
	switch (reg) {
		case REG_RESET:
			if (value > 0) {
				wdt_enable(WDTO_15MS);
				for (;;);
			} else {
				// do nothing when value == 0
			}
			break;
		case REG_SERVO_0_ENABLE:
			servo_enable(0);
			break;
		case REG_SERVO_0_DEGREES:
			servo_set_degrees(0, value);
			break;
		case REG_SERVO_1_ENABLE:
			servo_enable(1);
			break;
		case REG_SERVO_1_DEGREES:
			servo_set_degrees(1, value);
			break;
		case REG_SERVO_2_ENABLE:
			servo_enable(2);
			break;
		case REG_SERVO_2_DEGREES:
			servo_set_degrees(2, value);
			break;
		case REG_SERVO_3_ENABLE:
			servo_enable(3);
			break;
		case REG_SERVO_3_DEGREES:
			servo_set_degrees(3, value);
			break;
		default:
			// do nothing on bad register
			return;
	}
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
