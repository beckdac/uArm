#ifndef _SERVO_H_
#define _SERVO_H_

// defines
#define MAX_SERVOS	4
#define TRIM_DURATION	4
#define SERVO_MIN	544
#define SERVO_MAX	2400
#define WAITING_FOR_512_MARK	0
#define WAITING_TO_SET_PIN_LOW	1
#define WAITING_FOR_2048_MARK	2
#define WAITING_TO_SET_PIN_HIGH	3

#define TCNTn   TCNT0
#define OCRnx   OCR0A
#define OCFnx   OCF0A
#define OCIEnx  OCIE0A

// data structures
typedef struct servo {
	uint8_t pulse_length_ticks;
	uint8_t enabled;
	uint16_t min;
	uint16_t max;
} servo_t;

// prototypes
void servo_init(void);
// enable or disable servo pulses
void servo_enable(uint8_t);
void servo_disable(uint8_t);
// get and set functions
void servo_set_degrees(uint8_t s, uint8_t degrees);
void servo_set_pulse_length(uint8_t s, uint16_t pulse_length_us);
uint16_t servo_get_pulse_length(uint8_t s);
uint8_t servo_get_degrees(uint8_t s);
uint8_t servo_is_enabled(uint8_t);

#endif // _SERVO_H_
