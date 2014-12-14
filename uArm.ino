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

#define USE_ESP8266_LIB
#undef USE_ESP8266_LIB
#ifdef USE_ESP8266_LIB
#include "uartWIFI.h"
#endif

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
}

void setupMicroViewSliders(void) {
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


// ESP8266
#ifdef USE_ESP8266_LIB
#define SSID       "root"
#define PASSWORD   "conortimothy"

WIFI wifi;
extern int chlID;

void setup_ESP8266(void) {
	uView.setCursor(0,0);
	uView.print("wifi begin");
	uView.display();
	while (!wifi.begin());
	uView.setCursor(0,10);
	uView.print("wifi init");
	uView.display();
	while (!wifi.Initialize(AP_STA, SSID, PASSWORD));
	delay(2000);
	String ipstring = wifi.showIP();
	uView.setCursor(0,20);
	uView.print(ipstring.substring(8));
	uView.display();
	uView.setCursor(0,30);
	delay(1000);
	uView.print("mux 1");
	uView.display();
	wifi.confMux(1);
	delay(100);
	uView.setCursor(0,40);
	uView.print("server 23");
	uView.display();
	wifi.confServer(1, 23);
	delay(8000);
	uView.clear(PAGE);
	uView.display();
}
#endif

void sendMsg(String msg) {
#ifdef USE_ESP8266_LIB
	wifi.Send(chlID, msg);
#else
	Serial.print(msg);
#endif
}

void setup() {
	String msg;

	// setup serial port
	Serial.begin(9600);

	// setup MicroView
	setupMicroView();

#ifdef USE_ESP8266_LIB
	// ESP8266
	setup_ESP8266();
#endif

	// setup servos
	setupMicroViewSliders();
	attachServos();

	// place servo in positions
	moveServoL(defaultServoPos, servoSpeed, false, false);
	moveServoR(defaultServoPos, servoSpeed, false, false);
	moveServoRot(defaultServoPos, servoSpeed, false, false);
	moveServoHRot(defaultServoPos, servoSpeed, false, false);
	moveServoH(defaultServoPos, servoSpeed, true, true);

	delay(4000);

	sendMsg(F("READY\n"));
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
**	XXX = 112; emergency stop (detach servo timers)
**	XXX = 113; reattach servo timers
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
				msg = String(F("ERROR: missing servo position for servo ")) + String(servoShortName) + String("\n");
			} else {
				msg = String(F("ERROR: invalid servo position for servo ")) + String(servoShortName) + String(": ") + String(pos, DEC) + String("\n");
			}
			sendMsg(msg);
			return false;
		}
		*buf = endptr;
		return true;
	} else {
		servoMoveError = true;
		msg = String(F("ERROR: invalid servo identifier: ")) + String(servoShortName) + String("\n");
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
				String msg = String(F("ERROR: invalid G command code: ")) + String(code, DEC) + String("\n");
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
						String msg = String(F("ERROR: invalid G command wait flag, bad value: ")) + String(code, DEC) + String("\n");
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
				servoMoves[i].moveServoFuncPtr(servoMoves[i].pos, servoMoves[i].servoSpeed, 
					(i == servoMoveCount - 1 ? false : wait), (i == servoMoveCount - 1 ? true : false));
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
				String msg = String(F("ERROR: invalid F speed parameter: ")) + String(code, DEC) + String("\n");
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
					sendMsg(String(freeRam(), DEC) + String("\n"));
					break;
				case 102: {
					String msg = String(F("L")) + String(servoL.read(), DEC) +
							String(F(" R")) + String(servoR.read(), DEC) +
							String(F(" O")) + String(servoRot.read(), DEC) +
							String(F(" T")) + String(servoHRot.read(), DEC) +
							String(F(" H")) + String(servoH.read(), DEC) + String("\n");
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
					
				default:
					sendMsg(String(F("ERROR: invalid M address parameter: ")) + String(code, DEC) + String("\n"));
					return false;
			};
			return true;
			break;
		default:
			sendMsg(String(F("ERROR: invalid command: ")) + String(buf[0]) + String("\n"));
			return false;
	};
	sendMsg(String(F("ERROR: Unhandled exception in processCommand\n")));
	return false;
}

// main loop
const uint8_t MAX_CMD_LEN = 128;
char buf[MAX_CMD_LEN + 1], c;
#ifndef USE_ESP8266_LIB
bool suspendStore = false;
uint8_t cidx = 0;
#endif
uint8_t len = 0;

void loop() {
#ifdef USE_ESP8266_LIB
        len = wifi.ReceiveMessage(buf);
        if (len > 0) {
                processCommand(buf);
        }
#else
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
				sendMsg(F("ERROR: command line too long\n"));
				suspendStore = true;
			}
		}
	}
#endif
}