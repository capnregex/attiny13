##

PROGRAM = charlie
MCU = attiny13
CC = avr-gcc
OBJCOPY = avr-objcopy
F_CPU = 128000UL # khz
CFLAGS += -Wall -g -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU)
LDFLAGS +=
OBJS = $(PROGRAM).o

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

all: $(PROGRAM).hex eeprom.hex

$(PROGRAM).elf: $(PROGRAM).o
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(PROGRAM).hex: $(PROGRAM).elf
	@printf "  OBJCOPY $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(OBJCOPY) -gO ihex -R .eeprom $< $@

eeprom.hex: $(PROGRAM).elf
	@printf "  OBJCOPY $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(OBJCOPY) -gO ihex -j .eeprom $< $@

%.o: %.c
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

flash: $(PROGRAM).hex eeprom.hex
	@printf "  FLASH   $(PROGRAM).hex\n"
	$(Q)sudo avrdude -c usbtiny -p t13 -i 80 -B 64 -U flash:w:$(PROGRAM).hex
	$(Q)sudo avrdude -c usbtiny -p t13 -i 80 -B 64 -U eeprom:w:eeprom.hex

fuse: 
	$(Q)sudo avrdude -c usbtiny -p t13 -B 64 -U lfuse:w:0x7b:m -U hfuse:w:0xfd:m

fuse1Mhz: 
	$(Q)sudo avrdude -c usbtiny -p t13 -B 64 -U lfuse:w:0x6a:m -U hfuse:w:0xf9:m

fusereset: 
	$(Q)sudo avrdude -c usbtiny -p t13 -B 64 -U lfuse:w:0x6a:m -U hfuse:w:0xf9:m

clean:
	@printf "  CLEAN   $(subst $(shell pwd)/,,$(OBJS))\n"
	$(Q)rm -f $(OBJS)
	@printf "  CLEAN   $(PROGRAM).elf\n"
	$(Q)rm -f *.elf
	@printf "  CLEAN   $(PROGRAM).hex\n"
	$(Q)rm -f *.hex

