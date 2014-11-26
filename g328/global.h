#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#define TRUE	1
#define FALSE	0

#define nop()  __asm__ __volatile__("nop")

#define HINIB(byte) ((byte) & 0xF0)
#define LONIB(byte) ((byte) & 0x0F)

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )

#endif /* _GLOBAL_H_ */
