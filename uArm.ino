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

// MicroView
int SCREEN_WIDTH = uView.getLCDWidth();
int SCREEN_HEIGHT = uView.getLCDHeight();
MicroViewWidget *widgetL, *widgetR, *widgetRot, *widgetHRot, *widgetH;


// Servos
const uint8_t servoPinL = 2;
const uint8_t servoPinR = 3;
const uint8_t servoPinRot = 5;
const uint8_t servoPinHRot = 6;
const uint8_t servoPinH = A0;
const uint8_t slowSweepSpeed = 20;
VarSpeedServo servoL, servoR, servoRot, servoHRot, servoH;
const uint8_t defaultServoPos = 90;
uint8_t servoSpeed = slowSweepSpeed;

void moveServo(VarSpeedServo &servo, MicroViewWidget *widget, uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	if (degrees >= 0 && degrees <= 180) {
		servo.write(degrees, servoSpeed, wait);
		widget->setValue(degrees);
		if (updateDisplay)
			uView.display();
	}
}

typedef struct servoMove {
	void (*moveServoFuncPtr)(uint8_t, uint8_t, bool, bool);
	uint8_t pos;
	uint8_t servoSpeed;
} servoMoves_t;

#define MAX_CMD_SERVO_MOVES	5
uint8_t servoMoveCount;
bool servoMoveError;
servoMoves_t servoMoves[MAX_CMD_SERVO_MOVES];

// these are functions vs macros so we can use a function pointer during parsing
void moveServoL(uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	moveServo(servoL, widgetL, degrees, servoSpeed, wait, updateDisplay);
}
void moveServoR(uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	moveServo(servoR, widgetR, degrees, servoSpeed, wait, updateDisplay);
}
void moveServoRot(uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	moveServo(servoRot, widgetRot, degrees, servoSpeed, wait, updateDisplay);
}
void moveServoHRot(uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	moveServo(servoHRot, widgetHRot, degrees, servoSpeed, wait, updateDisplay);
}
void moveServoH(uint8_t degrees, uint8_t servoSpeed, bool wait, bool updateDisplay) {
	moveServo(servoH, widgetH, degrees, servoSpeed, wait, updateDisplay);
}

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
**	XXX = 102; report current servo desination positions
**	XXX = 112; emergency stop
*/

bool parseCommandGServo(char **buf, bool speedControl, bool wait) {
	char *endptr, servoShortName;
	uint8_t pos;
	int servoSpeedLocal = servoSpeed;
	void (*moveServoFuncPtr)(uint8_t, uint8_t, bool, bool) = NULL;

	if (!speedControl)
		servoSpeedLocal = 0;

	if (*buf[0] == '\0' || *buf[0] == '\n' || *buf[0] == '\r')
		return false;

	// should this display an error?
	if (servoMoveCount == MAX_CMD_SERVO_MOVES) {
		return false;
	}

	servoShortName = *buf[0];
	switch(servoShortName) {
		case 'L': case 'l':
			moveServoFuncPtr = &moveServoL;
			break;
		case 'R': case 'r':
			moveServoFuncPtr = &moveServoR;
			break;
		case 'O': case 'o':
			moveServoFuncPtr = &moveServoRot;
			break;
		case 'T': case 't':
			moveServoFuncPtr = &moveServoHRot;
			break;
		case 'H': case 'h':
			moveServoFuncPtr = &moveServoH;
			break;
	};

	if (moveServoFuncPtr != NULL) {
		(*buf)++;
		pos = strtoul(*buf, &endptr, 10);
		if (*buf != endptr && pos >= 0 && pos <= 180) {
			for (; **buf == ' ' || **buf == '\t'; (*buf)++);
			servoMoves[servoMoveCount].moveServoFuncPtr = moveServoFuncPtr;
			servoMoves[servoMoveCount].pos = pos;
			servoMoves[servoMoveCount].servoSpeed = servoSpeedLocal;
			servoMoveCount++;
		} else {
			servoMoveError = true;
			if (*buf == endptr) {
				Serial.print(F("ERROR: missing servo position for servo "));
				Serial.println(servoShortName);
			} else {
				Serial.print(F("ERROR: missing servo position for servo "));
				Serial.print(servoShortName);
				Serial.print(F(": "));
				Serial.println(pos, DEC);
			}
		}
		*buf = endptr;
		return true;
	} else {
		servoMoveError = true;
		Serial.print(F("ERROR: invalid servo identifier: "));
		Serial.println(servoShortName);
	}
	return false;
}

bool processCommand(char *cmd) {
	char *endptr, *buf = cmd;
	uint8_t i, len = strlen(buf);
	int code;
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
			for (; *endptr == ' ' || *endptr == '\t'; ++endptr);
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
						Serial.print(F("ERROR: invalid G command wait flag, bad value: "));
						Serial.println(code, DEC);
						return false;
					}
				} else {
					Serial.println(F("ERROR: invalid G command wait flag (missing value)"));
					return false;
				}
			}
			servoMoveCount = 0;
			servoMoveError = false;
			do {
				// absorb spaces
				for (; *endptr == ' ' || *endptr == '\t'; ++endptr);
			} while (parseCommandGServo(&endptr, speedControl, wait) && !servoMoveError);
			for (i = 0; !servoMoveError && i < servoMoveCount; ++i) {
				// perform this move, if the last move in the list, use the wait variable value and update display
				servoMoves[i].moveServoFuncPtr(servoMoves[i].pos, servoMoves[i].servoSpeed, 
					(i == servoMoveCount - 1 ? false : wait), (i == servoMoveCount - 1 ? true : false));
			}
			if (i != 0)
				Serial.println(F("OK"));
			break;
		case 'F': case 'f':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				Serial.println(F("ERROR: missing F speed parameter"));
				return false;
			}
			if (code >= 0 && code <= 255) {
				servoSpeed = code;
				Serial.println(F("OK"));
				return true;
			} else {
				Serial.print(F("ERROR: invalid F speed parameter: "));
				Serial.println(code, DEC);
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
				case 102:
					Serial.print(F("L"));
					Serial.print(servoL.read(), DEC);
					Serial.print(F(" R"));
					Serial.print(servoR.read(), DEC);
					Serial.print(F(" O"));
					Serial.print(servoRot.read(), DEC);
					Serial.print(F(" T"));
					Serial.print(servoHRot.read(), DEC);
					Serial.print(F(" H"));
					Serial.println(servoH.read(), DEC);
					break;
				case 112:
					detachServos();
					Serial.println(F("OK"));
					break;
				case 113:
					attachServos();
					Serial.println(F("OK"));
					break;
					
				default:
					Serial.print(F("ERROR: invalid M address parameter: "));
					Serial.println(code, DEC);
					return false;
			};
			break;
		default:
			Serial.print(F("ERROR: invalid command: "));
			Serial.println(buf[0]);
			return false;
	};
}

// main loop
const uint8_t MAX_CMD_LEN = 128;
char buf[MAX_CMD_LEN + 1], c;
bool suspendStore = false;
uint8_t len = 0, cidx = 0;

void loop() {
	// read to newline
	// ignore any line that has over 32 characters
	if (Serial.available() > 0) {
		c = Serial.read();
		if (c == '\n' || c == '\r') {
			if (suspendStore == false) {
				buf[cidx] = '\0';
				processCommand(buf);
			} else {
				suspendStore = false;
			}
			cidx = 0;
		} else {
			if (cidx < MAX_CMD_LEN) {
				buf[cidx] = c;
				++cidx;
			} else {
				Serial.println(F("ERROR: command line too long"));
				suspendStore = true;
			}
		}
	}
}
