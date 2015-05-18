#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include <7segment.h>
#include <font5x7.h>
#include <font8x16.h>
#include <fontlargenumber.h>
#include <MicroView.h>
#include <space01.h>
#include <space02.h>
#include <space03.h>

#include <VarSpeedServo.h>

#include <Wire.h>
#include <ArduinoNunchuk.h>

// FLASH strings
const char msg_OK[] PROGMEM = "OK\n";
const char msg_ERROR[] PROGMEM = "ERROR";
const char msg_LF[] PROGMEM = "\n";
const char msg_CSEP[] PROGMEM = ": ";
const char msg_MAXMOVES[] PROGMEM = "servo command has too many simultaneous moves, max";
const char msg_INVF[] PROGMEM = "invalid feed rate";
const char msg_MISSP[] PROGMEM = "missing servo position for servo";
const char msg_INVSP[] PROGMEM = "invalid servo position for servo";
const char msg_INVS[] PROGMEM = "invalid servo identifier";
const char msg_MISG[] PROGMEM = "missing G command code";
const char msg_INVG[] PROGMEM = "invalid G command code";
const char msg_INVW[] PROGMEM = "invalid G command wait flag, bad value";
const char msg_MISW[] PROGMEM = "missing G command wait flag";
const char msg_MISGF[] PROGMEM = "missing F speed parameter";
const char msg_INVGF[] PROGMEM = "invalid F speed parameter";
const char msg_MISM[] PROGMEM = "missing M address parameter";
const char msg_INVM500[] PROGMEM = "invalid M500 min/max flag, bad value";
const char msg_INVM[] PROGMEM = "invalid M address parameter";
const char msg_INV[] PROGMEM = "invalid command";
const char msg_TOOLONG[] PROGMEM = "command line too long";
const char msg_UNKPC[] PROGMEM = "unhandled exception in processCommand";
const char msg_INTEMSG[] PROGMEM = "requested message index is invalid";
const char msg_READY[] PROGMEM = "READY\n";
const char msg_POS[] PROGMEM = "POS";
const char msg_MIN[] PROGMEM = "MIN";
const char msg_MAX[] PROGMEM = "MAX";
const char msg_L[] PROGMEM = "L";
const char msg_R[] PROGMEM = " R";
const char msg_O[] PROGMEM = " O";
const char msg_T[] PROGMEM = " T";
const char msg_H[] PROGMEM = " H";
const char msg_SAFESP[] PROGMEM = "servo outside of safe limits in EEPROM";
const char msg_NCM_ON[] PROGMEM = "nunchuk control mode enabled\n";
const char msg_NCM_OFF[] PROGMEM = "nunchuk control mode disabled\n";
const char msg_ANALOG[] PROGMEM = "ANALOG: ";
const char msg_ACCEL[] PROGMEM = "ACCEL: ";
const char msg_BUTTON[] PROGMEM = "BUTTON: ";
const char msg_X[] PROGMEM = "X";
const char msg_Y[] PROGMEM = " Y";
const char msg_Z[] PROGMEM = " Z";
const char msg_C[] PROGMEM = "C";

const char * const msgTable[] PROGMEM = {
	msg_OK,
	msg_ERROR,
	msg_LF,
	msg_CSEP,
	msg_MAXMOVES,
	msg_INVF,
	msg_MISSP,
	msg_INVSP,
	msg_INVS,
	msg_MISG,
	msg_INVG,
	msg_INVW,
	msg_MISW,
	msg_MISGF,
	msg_INVGF,
	msg_MISM,
	msg_INVM500,
	msg_INVM,
	msg_INV,
	msg_TOOLONG,
	msg_UNKPC,
	msg_INTEMSG,
	msg_READY,
	msg_POS,
	msg_MIN,
	msg_MAX,
	msg_L,
	msg_R,
	msg_O,
	msg_T,
	msg_H,
	msg_SAFESP,
	msg_NCM_ON,
	msg_NCM_OFF,
	msg_ANALOG,
	msg_ACCEL,
	msg_BUTTON,
	msg_X,
	msg_Y,
	msg_Z,
	msg_C
};

#define MSG_OK		0
#define MSG_ERROR	1
#define MSG_LF		2
#define MSG_CSEP	3
#define MSG_MAXMOVES	4
#define MSG_INVF	5
#define MSG_MISSP	6
#define MSG_INVSP	7
#define MSG_INVS	8
#define MSG_MISG	9
#define MSG_INVG	10
#define MSG_INVW	11
#define MSG_MISW	12
#define MSG_MISGF	13
#define MSG_INVGF	14
#define MSG_MISM	15
#define MSG_INVM500	16
#define MSG_INVM	17
#define MSG_INV		18
#define MSG_TOOLONG	19
#define MSG_UNKPC	20
#define MSG_INTEMSG	21
#define MSG_READY	22
#define MSG_POS		23
#define MSG_MIN		24
#define MSG_MAX		25
#define MSG_L		26
#define MSG_R		27
#define MSG_O		28
#define MSG_T		29
#define MSG_H		30
#define MSG_SAFESP	31
#define MSG_NCM_ON	32
#define MSG_NCM_OFF	33
#define MSG_ANALOG	34
#define MSG_ACCEL	35
#define MSG_BUTTON	36
#define MSG_X		37
#define MSG_Y		38
#define MSG_Z		39
#define MSG_C		40
#define MAX_MSG		MSG_C

void sendMsgF(uint8_t idx) {
	unsigned int addr;
	char c;

	if (idx > MAX_MSG) {
		idx = MSG_INTEMSG;
	}

	addr = pgm_read_word(&msgTable[idx]);
	while ((c = pgm_read_byte(addr++)))
		Serial.write(c);
}

#define SEND_ERROR(msg)	sendMsgF(MSG_ERROR); sendMsgF(MSG_CSEP); sendMsgF(msg); sendMsgF(MSG_LF);
#define SEND_ERROR_DEC(msg, num)	sendMsgF(MSG_ERROR); sendMsgF(MSG_CSEP); sendMsgF(msg); sendMsgF(MSG_CSEP); Serial.print(num, DEC); sendMsgF(MSG_LF);
#define SEND_ERROR_CHAR(msg, ch)	sendMsgF(MSG_ERROR); sendMsgF(MSG_CSEP); sendMsgF(msg); sendMsgF(MSG_CSEP); Serial.write(ch); sendMsgF(MSG_LF);
#define SEND_ERROR_CHAR_DEC(msg, ch, num)	sendMsgF(MSG_ERROR); sendMsgF(MSG_CSEP); sendMsgF(msg); sendMsgF(MSG_CSEP); Serial.write(ch); sendMsgF(MSG_CSEP); Serial.print(num, DEC); sendMsgF(MSG_LF);
#define SEND_SERVO_POS(msg, L, R, O, T, H)	sendMsgF(msg); sendMsgF(MSG_CSEP); sendMsgF(MSG_L); Serial.print(L, DEC); sendMsgF(MSG_R); Serial.print(R, DEC); sendMsgF(MSG_O); Serial.print(O, DEC); sendMsgF(MSG_T); Serial.print(T, DEC); sendMsgF(MSG_H); Serial.print(H, DEC); sendMsgF(MSG_LF);

// EEPROM stored configuration
#define EEPROM_CONFIG_MAGIC	8576
#define EEPROM_CONFIG_VERSION		1
#define EEPROM_CONFIG_SERVO_L		0
#define EEPROM_CONFIG_SERVO_R		1
#define EEPROM_CONFIG_SERVO_ROT		2
#define EEPROM_CONFIG_SERVO_HROT	3
#define EEPROM_CONFIG_SERVO_H		4
struct config_t {
	uint16_t	magic;
	uint8_t		version;
	struct config_servoSafe_t {
		uint8_t	min;
		uint8_t	max;
	}	servoSafe[5];
} config;

void eepromConfigWrite(void) {
	eeprom_write_block((const void*)&config, (void*)0, sizeof(config));
}

void eepromConfigInitDefaults(void) {
	uint8_t i;
	config.magic = EEPROM_CONFIG_MAGIC;
	config.version = EEPROM_CONFIG_VERSION;
	for (i = 0; i < 5; ++i) {
		config.servoSafe[i].min = 10;
		config.servoSafe[i].max = 170;
	}
	eepromConfigWrite();
}

void eepromConfigRead(void) {
	eeprom_read_block((void*)&config, (void*)0, sizeof(config));
	if (config.magic != EEPROM_CONFIG_MAGIC || config.version != EEPROM_CONFIG_VERSION) {
		eepromConfigInitDefaults();
	}
}

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

// nunchuk
ArduinoNunchuk nunchuk = ArduinoNunchuk();
bool nunchukControlMode = false;
bool nunchukControlJustEntered = false;
bool nunchukControlJustExited = false;

void nunchukControlModeToggle(void) {
	nunchukControlMode = !nunchukControlMode;
	if (nunchukControlMode) {
		sendMsgF(MSG_NCM_ON);
		messageScrollSet("NCM+");
		nunchukControlJustEntered = true;
	} else {
		sendMsgF(MSG_NCM_OFF);
		messageScrollSet("NCM-");
		nunchukControlJustExited = true;
	}
	messageScrollUpdate(true);
}

#define NUNCHUK_DEADZONE 5
bool nunchukControlIsDeadzone(int x) {
	if (x >= 127 - NUNCHUK_DEADZONE && x <= 127 + NUNCHUK_DEADZONE)
		return true;
	return false;
}
#define NUNCHUK_SWEEP_SCALE DEFAULT_SERVO_SWEEP_SPEED / 2

void nunchukControlModeUpdate() {
	nunchuk.update();
	// the Z-Button disables nunchukcontrolmode
	// but only if we didn't just enable it
	//	i.e. it is still pressed from enabling the mode
	if (nunchuk.zButton)
		if (!nunchukControlJustEntered && !nunchukControlJustExited)
			nunchukControlModeToggle();
	else  {
		nunchukControlJustEntered = false;
		nunchukControlJustExited = false;

		Serial.print("update\n");
		if (!nunchukControlMode)
			return;
		// if the stick is centered, hold the current position
		if (nunchukControlIsDeadzone(nunchuk.analogX)) {
			moveServo(servoRot, widgetRot, servoRot.read(), 10, false, true);
			Serial.print("deadzone\n");
		} else if (nunchuk.analogX < 127) {
			// rotate
			moveServo(servoRot, widgetRot, 0, (127 - nunchuk.analogX) / NUNCHUK_SWEEP_SCALE, false, true);
			Serial.print("less\n");
		} else {
			moveServo(servoRot, widgetRot, 180, (nunchuk.analogX - 127) / NUNCHUK_SWEEP_SCALE, false, true);
			Serial.print("more\n");
		}
		if (nunchukControlIsDeadzone(nunchuk.analogY)) {
			moveServo(servoL, widgetL, servoL.read(), 10, true, true);
			moveServo(servoR, widgetR, servoR.read(), 10, true, true);
		}
	}
}

void nunchukControlModeSendValues(void) {
	if (nunchukControlMode) {
		sendMsgF(MSG_ANALOG);
		sendMsgF(MSG_X); Serial.print(nunchuk.analogX, DEC);
		sendMsgF(MSG_Y); Serial.print(nunchuk.analogY, DEC);
		sendMsgF(MSG_LF);
		sendMsgF(MSG_ACCEL);
		sendMsgF(MSG_X); Serial.print(nunchuk.accelX, DEC);
		sendMsgF(MSG_Y); Serial.print(nunchuk.accelY, DEC);
		sendMsgF(MSG_Z); Serial.print(nunchuk.accelZ, DEC);
		sendMsgF(MSG_LF);
		sendMsgF(MSG_BUTTON);
		sendMsgF(MSG_C); Serial.print(nunchuk.cButton, DEC);
		sendMsgF(MSG_Z); Serial.print(nunchuk.zButton, DEC);
		sendMsgF(MSG_LF);
	} else {
		sendMsgF(MSG_NCM_OFF);
	}
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


void setup() {
	String msg;
	uint8_t i;

	// setup serial port
	Serial.begin(115200);

	// load configuration from EEPROM
	eepromConfigRead();

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

	// initialize nunchuk
	nunchuk.init();

	sendMsgF(MSG_READY);
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
**		XXX = 500; set EEPROM configuration (i.e. servo safe limits)
**					M500 Y ZAAA [ZAAA] [...]
**					Y == I (min) or A (max)
**					Z == L || Z == R || Z == O || Z == T || Z == H; left, right, rot, hrot and hand respectively
**					AAA >= 0 && AAA <= 180; absolute location in degrees
**		XXX = 501; get EEPROM configuration (i.e. servo safe limits)
**		XXX = 576; toggle wii nunchuk control mode
**		XXX = 577; send the current nunchuk readings (only valid during M576)
*/

bool parseCommandGServo(char **buf, bool speedControl, bool wait) {
	char *endptr, servoShortName;
	int pos;
	uint8_t servoConfigIdx;
	int servoSpeedLocal = servoSpeed;
	void (*moveServoFuncPtr)(uint8_t, uint8_t, bool, bool) = NULL;
	String msg;

	if (!speedControl)
		servoSpeedLocal = 0;

	if (*buf[0] == '\0' || *buf[0] == '\n' || *buf[0] == '\r')
		return false;

	if (servoMoveCount == MAX_CMD_SERVO_MOVES) {
		SEND_ERROR_DEC(MSG_MAXMOVES, MAX_CMD_SERVO_MOVES);
		return false;
	}

	servoShortName = *buf[0];
	switch(servoShortName) {
		case 'L': case 'l':
			moveServoFuncPtr = &moveServoL;
			servoShortName = 'L';
			servoConfigIdx = EEPROM_CONFIG_SERVO_L;
			break;
		case 'R': case 'r':
			moveServoFuncPtr = &moveServoR;
			servoShortName = 'R';
			servoConfigIdx = EEPROM_CONFIG_SERVO_R;
			break;
		case 'O': case 'o':
			moveServoFuncPtr = &moveServoRot;
			servoShortName = 'O';
			servoConfigIdx = EEPROM_CONFIG_SERVO_ROT;
			break;
		case 'T': case 't':
			moveServoFuncPtr = &moveServoHRot;
			servoShortName = 'T';
			servoConfigIdx = EEPROM_CONFIG_SERVO_HROT;
			break;
		case 'H': case 'h':
			moveServoFuncPtr = &moveServoH;
			servoShortName = 'H';
			servoConfigIdx = EEPROM_CONFIG_SERVO_H;
			break;
		case 'F': case 'f':
			(*buf)++;
			pos = strtoul(*buf, &endptr, 10);
			if (pos > 0 && pos <= 255) {
				servoSpeed = pos;
				servoSpeedLocal = pos;
			} else {
				SEND_ERROR_DEC(MSG_INVGF, pos);
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
#warning check the pos against safe ranges
			if (pos < config.servoSafe[servoConfigIdx].min || pos > config.servoSafe[servoConfigIdx].max) {
				SEND_ERROR_CHAR_DEC(MSG_SAFESP, servoShortName, pos);
				servoMoveError = true;
				return false;
			}
			for (; **buf == ' ' || **buf == '\t'; (*buf)++);
			servoMoves[servoMoveCount].moveServoFuncPtr = moveServoFuncPtr;
			servoMoves[servoMoveCount].servoShortName = servoShortName;
			servoMoves[servoMoveCount].pos = pos;
			servoMoves[servoMoveCount].servoSpeed = servoSpeedLocal;
			servoMoveCount++;
		} else {
			servoMoveError = true;
			if (*buf == endptr) {
				SEND_ERROR_CHAR(MSG_MISSP, servoShortName);
			} else {
				SEND_ERROR_CHAR_DEC(MSG_INVSP, servoShortName, pos);
			}
			return false;
		}
		*buf = endptr;
		return true;
	} else {
		servoMoveError = true;
		SEND_ERROR_CHAR(MSG_INVS, servoShortName);
		return false;
	}
	return false;
}

bool parseCommandM500Servo(char **buf, bool min) {
	char *endptr, servoShortName = '\0';
	int pos;
	String msg;
	uint8_t servoID = 0;

	if (*buf[0] == '\0' || *buf[0] == '\n' || *buf[0] == '\r')
		return false;

	servoShortName = *buf[0];
	switch(servoShortName) {
		case 'L': case 'l':
			servoShortName = 'L';
			servoID = EEPROM_CONFIG_SERVO_L;
			break;
		case 'R': case 'r':
			servoShortName = 'R';
			servoID = EEPROM_CONFIG_SERVO_R;
			break;
		case 'O': case 'o':
			servoShortName = 'O';
			servoID = EEPROM_CONFIG_SERVO_ROT;
			break;
		case 'T': case 't':
			servoShortName = 'T';
			servoID = EEPROM_CONFIG_SERVO_HROT;
			break;
		case 'H': case 'h':
			servoShortName = 'H';
			servoID = EEPROM_CONFIG_SERVO_H;
			break;
	};

	if (servoShortName != '\0') {
		(*buf)++;
		pos = strtoul(*buf, &endptr, 10);
		if (*buf != endptr && pos >= 0 && pos <= 180) {
			for (; **buf == ' ' || **buf == '\t'; (*buf)++);
			if (min) {
				config.servoSafe[servoID].min = pos;
			} else {
				config.servoSafe[servoID].max = pos;
			}
		} else {
			if (*buf == endptr) {
				SEND_ERROR_CHAR(MSG_MISSP, servoShortName);
			} else {
				SEND_ERROR_CHAR_DEC(MSG_INVSP, servoShortName, pos);
			}
			return false;
		}
		*buf = endptr;
		return true;
	} else {
		SEND_ERROR_CHAR(MSG_INVS, servoShortName);
		return false;
	}
	return false;
}

bool processCommand(char *cmd) {
	char *endptr, *buf = cmd;
	uint8_t i, len = strlen(buf);
	int code;
	bool speedControl = true;
	bool wait, min;
	String msg;

	if (len == 0) return false;

	switch (buf[0]) {
		case 'G': case 'g':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				SEND_ERROR(MSG_MISG);
				return false;
			}
			if (code == 0) {
				// rapid move
				speedControl = false;
			} else if (code == 1) {
				// speed control move
				speedControl = true;
			} else {
				SEND_ERROR_DEC(MSG_INVG, code);
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
						SEND_ERROR_DEC(MSG_INVW, code);
						return false;
					}
				} else {
					SEND_ERROR(MSG_MISW);
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
				sendMsgF(MSG_OK);
				return true;
			}
			return false;
			break;
		case 'F': case 'f':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				SEND_ERROR(MSG_MISGF);
				return false;
			}
			if (code >= 0 && code <= 255) {
				servoSpeed = code;
				sendMsgF(MSG_OK);
				return true;
			} else {
				SEND_ERROR_DEC(MSG_INVGF, code);
				return false;
			}
			break;
		case 'M': case 'm':
			code = strtoul(&buf[1], &endptr, 10);
			if (endptr == &buf[1]) {
				SEND_ERROR(MSG_MISM);
				return false;
			}
			switch (code) {
				case 100:
					sendMsgF(MSG_OK);
					delay(100);
					softwareReset();
					break;
				case 101:
					Serial.print(freeRam(), DEC);
					sendMsgF(MSG_LF);
					sendMsgF(MSG_OK);
					break;
				case 102: {
					SEND_SERVO_POS(MSG_POS, servoL.read(), servoR.read(), servoRot.read(), servoHRot.read(), servoH.read());
					sendMsgF(MSG_OK);
					break; }
				case 112:
					detachServos();
					sendMsgF(MSG_OK);
					break;
				case 113:
					attachServos();
					sendMsgF(MSG_OK);
					break;
				case 117:
					// absorb spaces
					for (; *endptr == ' ' || *endptr == '\t'; ++endptr);
					messageScrollSet(endptr);
					messageScrollUpdate(true);
					sendMsgF(MSG_OK);
					break;
				case 500:
					// absorb spaces
					for (; *endptr == ' ' || *endptr == '\t'; ++endptr);
					// look for min/max flag
					if (*endptr == 'I' || *endptr == 'i') {
						min = true;
					} else if (*endptr == 'A' || *endptr == 'a') {
						min = false;
					} else {
						SEND_ERROR_CHAR(MSG_INVM500, endptr[0]);
						return false;
					}
					endptr++;
					do {
						// absorb spaces
						for (; *endptr == ' ' || *endptr == '\t'; ++endptr);
					} while (parseCommandM500Servo(&endptr, min));
					eepromConfigWrite();
					sendMsgF(MSG_OK);
					break;
				case 501:
					SEND_SERVO_POS(MSG_MIN, config.servoSafe[EEPROM_CONFIG_SERVO_L].min, config.servoSafe[EEPROM_CONFIG_SERVO_R].min, config.servoSafe[EEPROM_CONFIG_SERVO_ROT].min, config.servoSafe[EEPROM_CONFIG_SERVO_HROT].min, config.servoSafe[EEPROM_CONFIG_SERVO_H].min);
					SEND_SERVO_POS(MSG_MAX, config.servoSafe[EEPROM_CONFIG_SERVO_L].max, config.servoSafe[EEPROM_CONFIG_SERVO_R].max, config.servoSafe[EEPROM_CONFIG_SERVO_ROT].max, config.servoSafe[EEPROM_CONFIG_SERVO_HROT].max, config.servoSafe[EEPROM_CONFIG_SERVO_H].max);
					sendMsgF(MSG_OK);
					break;
				case 576:
					nunchukControlModeToggle();
					sendMsgF(MSG_OK);
					break;
				case 577:
					nunchukControlModeSendValues();
					sendMsgF(MSG_OK);
					break;
				default:
					SEND_ERROR_DEC(MSG_INVM, code);
					return false;
			};
			return true;
			break;
		default:
			SEND_ERROR_CHAR(MSG_INV, buf[0]);
			return false;
	};
	SEND_ERROR(MSG_UNKPC);
	return false;
}

// main loop
const uint8_t MAX_CMD_LEN = 64;
char buf[MAX_CMD_LEN + 1], c;
bool suspendStore = false;
uint8_t cidx = 0;
uint8_t len = 0;
#define CYCLES_RESET_COUNT 120
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
				SEND_ERROR(MSG_TOOLONG);
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
		if (cycles == CYCLES_RESET_COUNT)
			cycles = 0;

		// if we are in the nunchuk control mode 
		// or once per CYCLES_RESET_COUNT cycles check for a Z-button press
		if (nunchukControlMode || cycles == 0)
			nunchukControlModeUpdate();
	}
}
