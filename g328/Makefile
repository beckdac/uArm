## deployment specific defines
DEVICE     = atmega328
#CLOCK      = 16000000L
#CLOCK      = 20000000L
# for internal oscillator
CLOCK      = 8000000L

## compile time defines
USART_BAUD=38400

## sources
SRCS_UARM = main.c
SRCS_UNIVERSAL = usart.c
SRCS = $(SRCS_CORE) $(SRCS_UNIVERSAL)

## objects
OBJS_UARM = $(SRCS_UARM:.c=.o)
OBJS_UNIVERSAL = $(SRCS_UNIVERSAL:.c=.o)
OBJS = $(OBJS_UARM)

## bins
ELFS = uArm.elf
BINS = $(ELFS:.elf=.hex)

DEFINES = -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -DUSART_BAUD=$(USART_BAUD)
DEFINES = -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -DUSART_BAUD=$(USART_BAUD) -DDEBUG=1
CFLAGS = -Wall -std=gnu99 -funsigned-char -funsigned-bitfields -ffunction-sections -fpack-struct -fshort-enums -I. -Os $(DEFINES)
CC = avr-gcc $(CFLAGS)
LINK = -Wl,-u,vfprintf -lprintf_flt -lm

all: $(BINS)

.c.o: Makefile
	$(CC) -c $< -o $@

clean: depend
	rm -f $(ELFS) $(BINS) $(OBJS)

uArm.elf: $(OBJS_UARM) $(OBJS_UNIVERSAL)
	$(CC) -o $@ $(OBJS_UARM) $(OBJS_UNIVERSAL) $(LINK)

%.hex: %.elf
	\rm -rf $@
	avr-objcopy -j .text -j .data -O ihex $< $@
	avr-size $@
	\ls -lh $@

Makefile.depend:
	touch Makefile.depend

depend: Makefile.depend
	makedepend -fMakefile.depend -- $(CFLAGS) -- $(SRCS) 

include Makefile.depend
