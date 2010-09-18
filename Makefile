TARGET = rgb_matrix
MCU_TARGET = atmega168

SRC := $(TARGET).c
OBJ = $(SRC:.c=.o)

CC = avr-gcc
CFLAGS = -g -Wall -Os -std=gnu99 -mmcu=$(MCU_TARGET)
LDFLAGS = -g -mmcu=$(MCU_TARGET) -Wl,-u,vfprintf -lprintf_flt -lm
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump

all: $(TARGET).hex $(TARGET).lst $(TARGET).od

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.od: %.elf
	$(OBJDUMP) -zhD $< > $@

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ)

clobber: clean
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).lst $(TARGET).od

.PHONY: all clean clobber
