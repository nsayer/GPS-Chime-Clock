
OUT=GPS_Chime_Clock

CHIP = attiny841
PROGRAMMER = usbtiny

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude
OPTS = -Os -g -std=c11 -Wall -Wno-main

CFLAGS = -mmcu=$(CHIP) $(OPTS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	$(OUT).hex $(OUT).hex

clean:
	rm -f *.hex *.elf *.o

flash:	$(OUT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:$(OUT).hex

fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xd5:m -U lfuse:w:0xe2:m -U efuse:w:0xff:m

init:	fuse flash
