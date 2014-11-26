#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "servo.h"

volatile static uint8_t servo_index = 0;
volatile static uint8_t state = WAITING_TO_SET_PIN_HIGH;
static servo_t servo[MAX_SERVOS];

static long map(long x, long in_min, long in_max, long out_min, long out_max);

void servo_init(void) {
	uint8_t i;

	// reset the Timer Counter Control Register to its reset value
	TCCR0B = 0;

	//set counter0 prescaler to 64
	//our FCLK is 8mhz so this makes each timer tick be 8 microseconds long
	TCCR0B &= ~(1<< CS02); //clear
	TCCR0B |=  (1<< CS01); //set
	TCCR0B |=  (1<< CS00); //set

	// enable output compare match interrupt
	TIMSK |= (1 << OCIE0A);
	// reset to 0
	TCNT0 = 0;
	OCR0A = 255;

	// initialize the data array
	for (i = 0; i < MAX_SERVOS; ++i) {
		servo[i].pulse_length_ticks = 128;
		servo[i].min = SERVO_MIN;
		servo[i].max = SERVO_MAX;
		servo[i].enabled = 0;
	}

	// set the current servo index for the ISR and the state
	servo_index = 0;
	state = WAITING_TO_SET_PIN_HIGH;
}

void servo_enable(uint8_t s) {
	if (s < MAX_SERVOS) {
		DDRB |= (1 << s);
		servo[s].enabled = 1;
	} else {
		// invalid servo id
	}
}

void servo_disable(uint8_t s) {
	if (s < MAX_SERVOS) {
		DDRB &= ~(1 << s);
		servo[s].enabled = 0;
	} else {
		// invalid servo id
	}
}

ISR(TIMER0_COMPA_vect) {
	switch (state) {
		case WAITING_TO_SET_PIN_HIGH:
			++servo_index;
			if (servo_index == MAX_SERVOS) {
				servo_index = 0;
			} else {
				// leave index as is
			}
			if (servo[servo_index].enabled) {
				PORTB |= (1 << servo_index);
			} else {
				// servo not enabled
			}

			TCNT0 = 0;
			// set the compare value to 64 (512 us); this is the constant pulse offset
			OCR0A = 64 - TRIM_DURATION;	// trim off 4 ticks (32us), this is about the total combined time in ISR
			state = WAITING_FOR_512_MARK;
			break;
		case WAITING_FOR_512_MARK:
			// set the compare value to the additional amount of timer ticks the pulse should last
			OCR0A = servo[servo_index].pulse_length_ticks;
			// update our state
			state = WAITING_TO_SET_PIN_LOW;

			// reset the counter to 0
			TCNT0  = 0;

			// did we just set OCR0A to zero?
			if(OCR0A == 0) {
		   		// because we are setting OCR0A and TCNT0 to 0 we are not going to get an interrupt
		   		// until the counter overflows and goes back to 0.
		   		// set the counter its highest value, to have it overflow right away.
		   		TCNT0 = 0xFF;
		   		// this will cause this interrupt to fire again almost immediately (at the next timer tick)
			} else {
				//otherwise we need to clear the OCF0A flag because it is possible that the
				//counter value incremented and matched the output compare value while this
				//function was being executed
				TIFR = (1 << OCF0A);  // write logical 1 to the OCF0A flag to clear it
						  // also have to write 0 to all other bits for this to work.
			}
			break;
		case WAITING_TO_SET_PIN_LOW:
			// if this servo is enabled set the pin low
			if (servo[servo_index].enabled) {
				PORTB &= ~(1 << servo_index);
			} else {
				// servo not enabled
			}
			//check if the length of this pulse is 2048 microseconds or longer
			if ((64 + servo[servo_index].pulse_length_ticks) > 255) {
				// this pulse length has passed the 2048 us mark, so we skip state WAITING_FOR_2048_MARK
				// update state
				state = WAITING_TO_SET_PIN_HIGH;
				// set the compare value to the amount of time (in timer ticks) we need to wait to reach
				// 4096 microseconds mark
				// which is 512 minus the total pulse length. (resulting number will be between 0 and 255 inclusive)
				OCR0A = 512 - (64 + servo[servo_index].pulse_length_ticks);
			} else {
				// this pulse length has not reached the 2048 us mark, therefor we have to get to that mark first
				// update state
				state = WAITING_FOR_2048_MARK;
				// set OCR0A to the amount of time (in timer ticks) we have to wait to reach this mark
				// which is 255 minus the total pulse length
				OCR0A = 255 - (64 + servo[servo_index].pulse_length_ticks);
			}
			// reset the counter to 0
			TCNT0  = 0;
			break;
		case WAITING_FOR_2048_MARK:
			// update state
			state = WAITING_TO_SET_PIN_HIGH;
			// reset the counter to 0
			TCNT0  = 0;
			// set the compare value to the longest length of time, 255 ticks, or 2040 microseconds
			// this will take us to the ~4096 microsecond mark,
			// at which point the cycle starts again with the next servo slot.
			OCR0A = 255;
			break;
	}
}

void servo_set_degrees(uint8_t s, uint8_t degrees) {
	if (s < MAX_SERVOS) {
		if (degrees <= 180) {
			uint16_t pulse_length_us = map(degrees, 0, 180, servo[s].min, servo[s].max);
			servo_set_pulse_length(s, pulse_length_us);
		} else {
			// invalid servo position in degrees
		}
	} else {
		// invalid servo id
	}
}

void servo_set_pulse_length(uint8_t s, uint16_t pulse_length_us) {
	if (s < MAX_SERVOS) {
		int16_t pulse_length_ticks = pulse_length_us / 8;
		// subtract pulse offset
		pulse_length_ticks -= 64;
		if (pulse_length_ticks > -1 && pulse_length_ticks < 256) {
			servo[s].pulse_length_ticks = (uint8_t)pulse_length_ticks;
		}
	} else {
		// invalid servo id
	}
}

uint16_t servo_get_pulse_length(uint8_t s) {
	if (s < MAX_SERVOS) {
		return servo[s].pulse_length_ticks;
	}
	// invalid servo id
	return 0;
}

uint8_t servo_get_degrees(uint8_t s) {
	if (s < MAX_SERVOS) {
		uint16_t pulse_length_us = servo_get_pulse_length(s);
		uint16_t position_degrees = map(pulse_length_us, servo[s].min, servo[s].max, 0, 180);
		return position_degrees;
	}
	// invalid servo id
	return 0;
}

uint8_t servo_is_enabled(uint8_t s) {
	if (s < MAX_SERVOS) {
		return servo[s].enabled;
	}
	// invalid servo id
	return 0;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
