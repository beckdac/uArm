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
#define DEFAULT_SERVO_SWEEP_SPEED 10
const uint8_t servoPinL = 2;
const uint8_t servoPinR = 3;
const uint8_t servoPinRot = 5;
const uint8_t servoPinHRot = 6;
const uint8_t servoPinH = A0;
VarSpeedServo servoL, servoR, servoRot, servoHRot, servoH;
const uint8_t defaultServoPos = 90;
uint8_t servoSpeed = DEFAULT_SERVO_SWEEP_SPEED;

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
	char servoShortName;
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

void servoWidgetUpdate(bool updateDisplay) {
	widgetL->setValue(servoL.read());
	widgetR->setValue(servoR.read());
	widgetRot->setValue(servoRot.read());
	widgetHRot->setValue(servoHRot.read());
	widgetH->setValue(servoH.read());
	if (updateDisplay)
		uView.display();
}

#define MSG_SCROLL_LEN 32
char messageScroll[MSG_SCROLL_LEN] = "READY";
uint8_t messageScrollHead = 0;
#define MSG_X_POS 59

void messageScrollSet(char *buf) {
	uint8_t i = 0;
	for (; *buf != '\0' && *buf != '\n' && *buf != '\r' && *buf != '\t' && i < MSG_SCROLL_LEN - 1; ++buf, ++i) {
		messageScroll[i] = *buf;
	}
	for (; i < MSG_SCROLL_LEN; ++i)
		messageScroll[i] = '\0';
	// reset the scroller on next update call
	messageScrollHead = 0;
}

void messageScrollUpdate(bool updateDisplay) {
	char c;
	uint8_t i;

	// reset to start of message
	if (messageScroll[messageScrollHead] == '\0') {
		messageScrollHead = 0;
	}
	i = messageScrollHead;
	// display 5 characters
	uView.setCursor(MSG_X_POS, 1);
	c = messageScroll[i];
	if (c != '\0') {
		uView.write(c);
	} else {
		uView.write(' ');
	}
	uView.setCursor(MSG_X_POS, 11);
	if (i + 1 < MSG_SCROLL_LEN && messageScroll[i + 1] != '\0') {
		c = messageScroll[i + 1];
		uView.write(c);
	} else {
		uView.write(' ');
	}
	uView.setCursor(MSG_X_POS, 21);
	if (i + 2 < MSG_SCROLL_LEN && messageScroll[i + 2] != '\0') {
		c = messageScroll[i + 2];
		uView.write(c);
	} else {
		uView.write(' ');
	}
	uView.setCursor(MSG_X_POS, 31);
	if (i + 3 < MSG_SCROLL_LEN && messageScroll[i + 3] != '\0') {
		c = messageScroll[i + 3];
		uView.write(c);
	} else {
		uView.write(' ');
	}
	uView.setCursor(MSG_X_POS, 41);
	if (i + 4 < MSG_SCROLL_LEN && messageScroll[i + 4] != '\0') {
		c = messageScroll[i + 4];
		uView.write(c);
	} else {
		uView.write(' ');
	}
	// increment head pointer
	messageScrollHead++;
	if (updateDisplay)
		uView.display();
}

// setup functions
void setupMicroView(void) {
	// setup MicroView
	uView.begin();
	uView.clear(PAGE);
	uView.display();
}

#define SLIDER_LABEL_X_POS 52

void setupMicroViewSliders(void) {
	widgetL = new MicroViewSlider(0, 0, 0, 180);
	uView.setCursor(SLIDER_LABEL_X_POS, 1);
	uView.write('L');
	widgetR = new MicroViewSlider(0, 10, 0, 180);
	uView.setCursor(SLIDER_LABEL_X_POS, 11);
	uView.write('R');
	widgetRot = new MicroViewSlider(0, 20, 0, 180);
	uView.setCursor(SLIDER_LABEL_X_POS, 21);
	uView.write('O');
	widgetHRot = new MicroViewSlider(0, 30, 0, 180);
	uView.setCursor(SLIDER_LABEL_X_POS, 31);
	uView.write('T');
	widgetH = new MicroViewSlider(0, 40, 0, 180);
	uView.setCursor(SLIDER_LABEL_X_POS, 41);
	uView.write('H');
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


#define sendMsg(msg) Serial.print(msg)

void setup() {
	String msg;
	uint8_t i;

	// setup serial port
	Serial.begin(115200);

	// setup MicroView
	setupMicroView();

	// boot pause
	for (i = 0; i < 8; ++i) {
		uView.circleFill(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, (SCREEN_WIDTH < SCREEN_HEIGHT ? SCREEN_WIDTH : SCREEN_HEIGHT) / 2 / (i + 2), (i % 2 == 0 ? WHITE : BLACK), NORM);
		uView.display();
		delay(500);
	}
	uView.clear(PAGE);

	// setup servos
	setupMicroViewSliders();
	attachServos();

	// place servo in positions
	moveServoL(defaultServoPos, servoSpeed, false, false);
	moveServoR(defaultServoPos, servoSpeed, false, false);
	moveServoRot(defaultServoPos, servoSpeed, false, false);
	moveServoHRot(defaultServoPos, servoSpeed, false, false);
	moveServoH(defaultServoPos, servoSpeed, true, true);

	sendMsg(F("READY\n"));
}

/*
** accepted serial commands
** FXXX
**      X >= 0 & X <= 255; sets servo speed
** GX WY [FWWW] ZAAA [[FWWW] ZAAA] [[FWWW] ZAAA] [...]
**      X == 0 || X == 1; use speed control? (false) rapid or (true) speed control move, boolean
**      Y == 0 || Y == 1; wait? (false) no or (true) yes, boolean
**      WWW >= 0 && WWW <= 255; servo feed rate (move rate)
**      Z == L || Z == R || Z == O || Z == T || Z == H; left, right, rot, hrot and hand respectively
**      AAA >= 0 && AAA <= 180; absolute location in degrees
** MXXX
**      XXX = 100; reset
**      XXX = 101; report free memory
**      XXX = 102; report current servo destinations (if move complete then positions)
**      XXX = 112; emergency stop (detach servos)
**      XXX = 113; emergency stop resume (reattach servos)
**		XXX = 117; set display message
*/

bool parseCommandGServo(char **buf, bool speedControl, bool wait) {
	char *endptr, servoShortName;
	int pos;
	int servoSpeedLocal = servoSpeed;
	void (*moveServoFuncPtr)(uint8_t, uint8_t, bool, bool) = NULL;
	String msg;

	if (!speedControl)
		servoSpeedLocal = 0;

	if (*buf[0] == '\0' || *buf[0] == '\n' || *buf[0] == '\r')
		return false;

	if (servoMoveCount == MAX_CMD_SERVO_MOVES) {
		sendMsg(String(F("ERROR: servo command has too many simultaneous moves, max = ")) + String(MAX_CMD_SERVO_MOVES, DEC) + String(F("\n")));
		return false;
	}

	servoShortName = *buf[0];
	switch(servoShortName) {
		case 'L': case 'l':
			moveServoFuncPtr = &moveServoL;
			servoShortName = 'L';
			break;
		case 'R': case 'r':
			moveServoFuncPtr = &moveServoR;
			servoShortName = 'R';
			break;
		case 'O': case 'o':
			moveServoFuncPtr = &moveServoRot;
			servoShortName = 'O';
			break;
		case 'T': case 't':
			moveServoFuncPtr = &moveServoHRot;
			servoShortName = 'T';
			break;
		case 'H': case 'h':
			moveServoFuncPtr = &moveServoH;
			servoShortName = 'H';
			break;
		case 'F': case 'f':
			(*buf)++;
			pos = strtoul(*buf, &endptr, 10);
			if (pos > 0 && pos <= 255) {
				servoSpeed = pos;
				servoSpeedLocal = pos;
			} else {
				sendMsg(String(F("ERROR: invalid feed rate\n")));
				return false;
			}
			*buf = endptr;
			return true;
			break;
	};

	if (moveServoFuncPtr != NULL) {
		(*buf)++;
		pos = strtoul(*buf, &endptr, 10);
		if (*buf != endptr && pos >= 0 && pos <= 180) {
			for (; **buf == ' ' || **buf == '\t'; (*buf)++);
			servoMoves[servoMoveCount].moveServoFuncPtr = moveServoFuncPtr;
			servoMoves[servoMoveCount].servoShortName = servoShortName;
			servoMoves[servoMoveCount].pos = pos;
			servoMoves[servoMoveCount].servoSpeed = servoSpeedLocal;
			servoMoveCount++;
		} else {
			servoMoveError = true;
			if (*buf == endptr) {
				msg = String(F("ERROR: missing servo position for servo ")) + String(servoShortName) + String(F("\n"));
			} else {
				msg = String(F("ERROR: invalid servo position for servo ")) + String(servoShortName) + String(F(": ")) + String(pos, DEC) + String(F("\n"));
			}
			sendMsg(msg);
			return false;
		}
		*buf = endptr;
		return true;
	} else {
		servoMoveError = true;
		msg = String(F("ERROR: invalid servo identifier: ")) + String(servoShortName) + String(F("\n"));
		sendMsg(msg);
		return false;
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
				sendMsg(F("ERROR: missing G command code\n"));
				return false;
			}
			if (code == 0) {
				// rapid move
				speedControl = false;
			} else if (code == 1) {
				// speed control move
				speedControl = true;
			} else {
				String msg = String(F("ERROR: invalid G command code: ")) + String(code, DEC) + String(F("\n"));
				sendMsg(msg);
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
						String msg = String(F("ERROR: invalid G command wait flag, bad value: ")) + String(code, DEC) + String(F("\n"));
						sendMsg(msg);
						return false;
					}
				} else {
					sendMsg(F("ERROR: invalid G command wait flag (missing value)\n"));
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
				servoMoves[i].moveServoFuncPtr(servoMoves[i].pos, servoMoves[i].servoSpeed, false,
					(i == servoMoveCount - 1 ? true : false));
			}
			// do software wait while updating widgets
			if (!servoMoveError && wait) {
				bool done = false;
				while (!done) {
					servoWidgetUpdate(true);
					done = true;
					for (i = 0; i < servoMoveCount; ++i) {
						switch (servoMoves[i].servoShortName) {
							case 'L':
								if (servoMoves[i].pos != servoL.read())
									done = false;
								break;
							case 'R':
								if (servoMoves[i].pos != servoR.read())
									done = false;
								break;
							case 'O':
								if (servoMoves[i].pos != servoRot.read())
									done = false;
								break;
							case 'T':
								if (servoMoves[i].pos != servoHRot.read())
									done = false;
								break;
							case 'H':
								if (servoMoves[i].pos != servoH.read())
									done = false;
								break;
							default:
								break;
						}
					}
				}
			}
			if (i != 0) {
				sendMsg(F("OK\n"));
				return true;
			}
			return false;
			break;
		case 'F': case 'f':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				sendMsg(F("ERROR: missing F speed parameter\n"));
				return false;
			}
			if (code >= 0 && code <= 255) {
				servoSpeed = code;
				sendMsg(F("OK\n"));
				return true;
			} else {
				String msg = String(F("ERROR: invalid F speed parameter: ")) + String(code, DEC) + String(F("\n"));
				sendMsg(msg);
				return false;
			}
			break;
		case 'M': case 'm':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				sendMsg(F("ERROR: missing M address parameter\n"));
				return false;
			}
			switch (code) {
				case 100:
					sendMsg(F("OK\n"));
					softwareReset();
					break;
				case 101:
					sendMsg(String(freeRam(), DEC) + String(F("\n")));
					break;
				case 102: {
					String msg = String(F("L")) + String(servoL.read(), DEC) +
							String(F(" R")) + String(servoR.read(), DEC) +
							String(F(" O")) + String(servoRot.read(), DEC) +
							String(F(" T")) + String(servoHRot.read(), DEC) +
							String(F(" H")) + String(servoH.read(), DEC) + String(F("\n"));
					sendMsg(msg);
					break; }
				case 112:
					detachServos();
					sendMsg(F("OK\n"));
					break;
				case 113:
					attachServos();
					sendMsg(F("OK\n"));
					break;
				case 117:
					// absorb spaces
					for (; *endptr == ' ' || *endptr == '\t'; ++endptr);
					messageScrollSet(endptr);
					messageScrollUpdate(true);
					break;
				default:
					sendMsg(String(F("ERROR: invalid M address parameter: ")) + String(code, DEC) + String(F("\n")));
					return false;
			};
			return true;
			break;
		default:
			sendMsg(String(F("ERROR: invalid command: ")) + String(buf[0]) + String(F("\n")));
			return false;
	};
	sendMsg(String(F("ERROR: Unhandled exception in processCommand\n")));
	return false;
}

// main loop
const uint8_t MAX_CMD_LEN = 128;
char buf[MAX_CMD_LEN + 1], c;
bool suspendStore = false;
uint8_t cidx = 0;
uint8_t len = 0;
uint8_t cycles = 0;

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
				// if the first character is not printable, suspendStore and ignore the line
				if (cidx == 0 && !isprint(c)) {
					suspendStore = true;
				}
				// if suspendStore is false, then populate the command buffer
				if (!suspendStore) {
					buf[cidx] = c;
					++cidx;
				}
			} else {
				sendMsg(F("ERROR: command line too long\n"));
				suspendStore = true;
			}
		}
	} else {
		bool updateDisplay = true;
		if (cycles == 0)
			updateDisplay = false;
		servoWidgetUpdate(updateDisplay);
		if (cycles == 0)
			messageScrollUpdate(true);
		cycles++;
		if (cycles == 120)
			cycles = 0;
	}
}
