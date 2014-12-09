#include <avr/wdt.h>

#include <7segment.h>
#include <font5x7.h>
#include <font8x16.h>
#include <fontlargenumber.h>
#include <MicroView.h>
#include <space01.h>
#include <space02.h>
#include <space03.h>

#include <VarSpeedServo.h>

int SCREEN_WIDTH = uView.getLCDWidth();
int SCREEN_HEIGHT = uView.getLCDHeight();
MicroViewWidget *widgetL, *widgetR, *widgetRot, *widgetHRot, *widgetH;

const int servoPinL = 2;
const int servoPinR = 3;
const int servoPinRot = 5;
const int servoPinHRot = 6;
const int servoPinH = A0;
const int slowSweepSpeed = 20;
VarSpeedServo servoL, servoR, servoRot, servoHRot, servoH;
const int defaultServoPos = 90;
int servoSpeed = slowSweepSpeed;

void moveServo(VarSpeedServo &servo, MicroViewWidget *widget, uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	if (degrees >= 0 && degrees <= 180) {
		servo.write(degrees, servoSpeed, wait);
		widget->setValue(degrees);
		if (updateDisplay)
			uView.display();
	}
}

#define	moveServoL(degrees, servoSpeed, wait, updateDisplay)	moveServo(servoL, widgetL, degrees, servoSpeed, wait, updateDisplay)
#define	moveServoR(degrees, servoSpeed, wait, updateDisplay)	moveServo(servoR, widgetR, degrees, servoSpeed, wait, updateDisplay)
#define	moveServoRot(degrees, servoSpeed, wait, updateDisplay)	moveServo(servoRot, widgetRot, degrees, servoSpeed, wait, updateDisplay)
#define	moveServoHRot(degrees, servoSpeed, wait, updateDisplay)	moveServo(servoHRot, widgetHRot, degrees, servoSpeed, wait, updateDisplay)
#define	moveServoH(degrees, servoSpeed, wait, updateDisplay)	moveServo(servoH, widgetH, degrees, servoSpeed, wait, updateDisplay)

// helper functions
void softwareReset(void) {
	wdt_enable(WDTO_15MS);
	for(;;);
}

int freeRam(void) {
	extern int __heap_start, *__brkval; 
	int v; 
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// setup functions
void setupMicroView(void) {
	// setup MicroView
	uView.begin();
	uView.clear(PAGE);
	uView.display();

	widgetL = new MicroViewSlider(0, 0, 0, 180);
	widgetR = new MicroViewSlider(0, 10, 0, 180);
	widgetRot = new MicroViewSlider(0, 20, 0, 180);
	widgetHRot = new MicroViewSlider(0, 30, 0, 180);
	widgetH = new MicroViewSlider(0, 40, 0, 180);
}

void attachServos(void) {
	// setup servos
	servoL.attach(servoPinL);
	servoR.attach(servoPinR);
	servoRot.attach(servoPinRot);
	servoHRot.attach(servoPinHRot);
	servoH.attach(servoPinH);
}

void detachServos(void) {
	servoL.detach();
	servoR.detach();
	servoRot.detach();
	servoHRot.detach();
	servoH.detach();
}

void setup() {
	// setup serial port
	Serial.begin(9600);

	// setup MicroView
	setupMicroView();

	// setup servos
	attachServos();

	// place servo in positions
	moveServoL(defaultServoPos, servoSpeed, false, false);
	moveServoR(defaultServoPos, servoSpeed, false, false);
	moveServoRot(defaultServoPos, servoSpeed, false, false);
	moveServoHRot(defaultServoPos, servoSpeed, false, false);
	moveServoH(defaultServoPos, servoSpeed, true, true);
}

/*
** accepted serial commands
** FXXX
**	X >= 0 & X <= 255; sets servo speed
** GX WY ZAAA [ZAAA] [ZAAA] [ZAAA] [ZAAA] [...]
**	X == 0 || X == 1; use speed control? (false) rapid or (true) speed control move, boolean
**	Y == 0 || Y == 1; wait? (false) no or (true) yes, boolean
**	Z == L || Z == R || Z == O || Z == T || Z == H; left, right, rot, hrot and hand respectively
**	AAA >= 0 && AAA <= 180; absolute location in degrees
** MXXX
**	XXX = 100; reset
**	XXX = 101; print free memory
**	XXX = 112; emergency stop
*/

bool parseCommandGServo(char **buf, boolean speedControl, boolean wait) {
	char *endptr;
	uint8_t pos, i, len;
	int servoSpeedLocal = servoSpeed;

	if (!speedControl)
		servoSpeedLocal = 0;

	if (*buf[0] == '\0')
		return false;
	switch(*buf[0]) {
		case 'L': case 'l':
			pos = strtoul(buf[1], &endptr, 10);
			if (buf[1] != endptr) {
				len = strlen(*buf);
				for (i = 0; i < len; ++i) {
					if (*buf[i] != ' ' && *buf[i] != '\t' && *buf[i] != '\0')
						break;
				}
				if (i == len) {
					moveServoL(defaultServoPos, servoSpeedLocal, wait, true);
				} else {
					moveServoL(defaultServoPos, servoSpeedLocal, false, false);
				}
			} else {
				Serial.println(F("ERROR: invalid servoL position"));
			}
			return true;
		case 'R': case 'r':
			pos = strtoul(buf[1], &endptr, 10);
			if (buf[1] != endptr) {
				len = strlen(*buf);
				for (i = 0; i < len; ++i) {
					if (*buf[i] != ' ' && *buf[i] != '\t' && *buf[i] != '\0')
						break;
				}
				if (i == len) {
					moveServoR(defaultServoPos, servoSpeedLocal, wait, true);
				} else {
					moveServoR(defaultServoPos, servoSpeedLocal, false, false);
				}
			} else {
				Serial.println(F("ERROR: invalid servoR position"));
			}
			return true;
		case 'O': case 'o':
			pos = strtoul(buf[1], &endptr, 10);
			if (buf[1] != endptr) {
				len = strlen(*buf);
				for (i = 0; i < len; ++i) {
					if (*buf[i] != ' ' && *buf[i] != '\t' && *buf[i] != '\0')
						break;
				}
				if (i == len) {
					moveServoRot(defaultServoPos, servoSpeedLocal, wait, true);
				} else {
					moveServoRot(defaultServoPos, servoSpeedLocal, false, false);
				}
			} else {
				Serial.println(F("ERROR: invalid servoRot position"));
			}
			return true;
		case 'T': case 't':
			pos = strtoul(buf[1], &endptr, 10);
			if (buf[1] != endptr) {
				len = strlen(*buf);
				for (i = 0; i < len; ++i) {
					if (*buf[i] != ' ' && *buf[i] != '\t' && *buf[i] != '\0')
						break;
				}
				if (i == len) {
					moveServoHRot(defaultServoPos, servoSpeedLocal, wait, true);
				} else {
					moveServoHRot(defaultServoPos, servoSpeedLocal, false, false);
				}
			} else {
				Serial.println(F("ERROR: invalid servoHRot position"));
			}
			return true;
		case 'H': case 'h':
			pos = strtoul(buf[1], &endptr, 10);
			if (buf[1] != endptr) {
				len = strlen(*buf);
				for (i = 0; i < len; ++i) {
					if (*buf[i] != ' ' && *buf[i] != '\t' && *buf[i] != '\0')
						break;
				}
				if (i == len) {
					moveServoH(defaultServoPos, servoSpeedLocal, wait, true);
				} else {
					moveServoH(defaultServoPos, servoSpeedLocal, false, false);
				}
			} else {
				Serial.println(F("ERROR: invalid servoH position"));
			}
			return true;
		default:
			Serial.println(F("ERROR: invalid servo identifier"));
			return false;
	};
	return false;
}

bool processCommand(char *cmd) {
	char *endptr, *buf = cmd;
	uint8_t len = strlen(buf);
	uint8_t code;
	bool speedControl = true;
	bool wait;

	if (len == 0) return false;

	switch (buf[0]) {
		case 'G': case 'g':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				Serial.println(F("ERROR: missing G command code"));
				return false;
			}
			if (code == 0) {
				// rapid move
				speedControl = false;
			} else if (code == 1) {
				// speed control move
				speedControl = true;
			} else {
				Serial.println(F("ERROR: invalid G command code"));
				return false;
			}
			// absorb spaces
			for (; *endptr == ' ' && *endptr == '\t'; ++endptr);
			// check for wait flag
			wait = true;
			if (*endptr == 'W' || *endptr == 'w') {
				buf = ++endptr;
				code = strtoul(buf, &endptr, 10);
				if (buf != endptr) {
					if (code == 0) {
						wait = false;
					} else if (code == 1) {
						wait = true;
					} else {
						Serial.println(F("ERROR: invalid G command wait flag (bad number)"));
						return false;
					}
				} else {
					Serial.println(F("ERROR: invalid G command wait flag (no number)"));
					return false;
				}
			}
			do {
				// absorb spaces
				for (; *endptr == ' ' && *endptr == '\t'; ++endptr);
			} while (parseCommandGServo(&endptr, speedControl, wait));
			break;
		case 'F': case 'f':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				Serial.println(F("ERROR: missing F speed parameter"));
				return false;
			}
			if (code >= 0 && code <= 255) {
				servoSpeed = code;
				return true;
			} else {
				Serial.println(F("ERROR: invalid F speed parameter"));
				return false;
			}
			break;
		case 'M': case 'm':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				Serial.println(F("ERROR: missing M address parameter"));
				return false;
			}
			switch (code) {
				case 100:
					Serial.println(F("OK"));
					softwareReset();
					break;
				case 101:
					Serial.println(freeRam(), DEC);
					break;
				case 112:
					detachServos();
					Serial.println(F("OK"));
					break;
				default:
					Serial.println(F("ERROR: invalid M address parameter"));
					return false;
			};
			break;
		default:
			Serial.println(F("ERROR: invalid command"));
			return false;
	};
}

const uint8_t MAX_CMD_LEN = 32;

void loop() {
	char buf[MAX_CMD_LEN + 1], c;
	bool suspendStore = false;
	uint8_t i = 0;

	// read to newline
	// ignore any line that has over 32 characters
	if (Serial.available() > 0) {
		c = Serial.read();
		if (c == '\n' || c == '\r') {
			if (suspendStore == false) {
				buf[i] = '\0';
				processCommand(buf);
			} else {
				suspendStore = false;
			}
			i = 0;
		} else {
			if (i < MAX_CMD_LEN) {
				buf[i] = c;
				++i;
			} else {
				Serial.println(F("ERROR: command line too long"));
				suspendStore = true;
			}
		}
	}
}
