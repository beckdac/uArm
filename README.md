LiteArm / uArm
====

LiteArm / uArm firmware for the MicroView + esp8266 enabled control board: https://github.com/beckdac/uArm-MicroView-esp8266

Supports nunchuk control mode where a Wii Nunchuk conneted via I2C can be used to control the arm.  Enabled with the M576 commonad or pushing and holding Z-until the display changess and disabled by sending the M576 command again or hitting the Z-button on the Nunchuk.

```
/*
** accepted serial commands
** FXXX
**	X >= 0 & X <= 255; sets servo speed
** GX WY [FWWW] ZAAA [[FWWW] ZAAA] [[FWWW] ZAAA] [...]
**	X == 0 || X == 1; use speed control? (false) rapid or (true) speed control move, boolean
**	Y == 0 || Y == 1; wait? (false) no or (true) yes, boolean
**	WWW >= 0 && WWW <= 255; servo feed rate (move rate)
**	Z == L || Z == R || Z == O || Z == T || Z == H; left, right, rot, hrot and hand respectively
**	AAA >= 0 && AAA <= 180; absolute location in degrees
** MXXX
**	XXX = 100; reset
**	XXX = 101; report free memory
**	XXX = 102; report current servo destinations (if move complete then positions)
**	XXX = 112; emergency stop (detach servos)
**	XXX = 113; emergency stop resume (reattach servos)
**	XXX = 117; set display message
**  XXX = 500; set EEPROM configuration (i.e. servo safe limits)
**     M500 Y ZAAA [ZAAA] [...]
**       Y == I (min) or A (max)
**       Z == L || Z == R || Z == O || Z == T || Z == H; left, right, rot, hrot and hand respectively
**       AAA >= 0 && AAA <= 180; absolute location in degrees
**  XXX = 501; get EEPROM configuration (i.e. servo safe limits)
**	XXX = 576; toggle wii nunchuk control mode
**	XXX = 577; send the current nunchuk readings (only valid during M576)
*/
```

My safe limits:
```
M500 I L40
M500 A L120
M500 I R60
M500 A R90
M500 I O10
M500 A O170
```
