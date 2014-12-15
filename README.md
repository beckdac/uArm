LiteArm / uArm
====

LiteArm / uArm firmware for the MicroView + esp8266 enabled control board: https://github.com/beckdac/uArm-MicroView-esp8266

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
*/
```
