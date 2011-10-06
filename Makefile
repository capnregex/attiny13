##
## This file is part of the avr-gcc-examples project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

all: blink breathing tinyCylon

tinyCylon:
	@printf "  BUILD   tinyCylon\n"
	$(Q)$(MAKE) -C tinyCylon

blink:
	@printf "  BUILD   blink\n"
	$(Q)$(MAKE) -C blink

breathing:
	@printf "  BUILD   breathing\n"
	$(Q)$(MAKE) -C breathing

clean:
	@printf "  CLEAN   tinyCylon\n"
	$(Q)$(MAKE) -C tinyCylon clean
	@printf "  CLEAN   blink\n"
	$(Q)$(MAKE) -C blink clean
	@printf "  CLEAN   breathing\n"
	$(Q)$(MAKE) -C breathing clean

.PHONY: blink breathing

