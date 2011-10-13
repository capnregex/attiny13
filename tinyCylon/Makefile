##
## This file is part of the avr-gcc-examples project.
##
## Copyright (C) 2008 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

PROGRAM = tinyCylon
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
	$(Q)avrdude -c usbtiny -p t13 -i 80 -B 64 -U flash:w:$(PROGRAM).hex
	$(Q)avrdude -c usbtiny -p t13 -i 80 -B 64 -U eeprom:w:eeprom.hex

fuse: 
	$(Q)avrdude -c usbtiny -p t13 -B 64 -U lfuse:w:0x7b:m -U hfuse:w:0xfd:m
#	$(Q)avrdude -c usbtiny -p t13 -U lfuse:w:0x6b:m -U hfuse:w:0xfd:m

fusereset: 
	$(Q)avrdude -c usbtiny -p t13 -i 80 -U lfuse:w:0x6a:m -U hfuse:w:0xf9:m

clean:
	@printf "  CLEAN   $(subst $(shell pwd)/,,$(OBJS))\n"
	$(Q)rm -f $(OBJS)
	@printf "  CLEAN   $(PROGRAM).elf\n"
	$(Q)rm -f *.elf
	@printf "  CLEAN   $(PROGRAM).hex\n"
	$(Q)rm -f *.hex

