/*
 * This file is part of the avr-gcc-examples project.
 *
 * Copyright (C) 2008 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <stdint.h>

int main(void)
{
	uint8_t i, delay = 128, direction = 1;

	DDRB = 0xff;

	while (1) {
		for (i = 1; i < 15; i++) {
			PORTB &= ~(1 << 4);	/* LED on */
			_delay_loop_2(delay);
			PORTB |= (1 << 4);	/* LED off */
			_delay_loop_2(256 - delay);
		}

		(direction == 1) ? delay++ : delay--;

		if (delay >= 128)
			direction = 0;
		
		if (delay <= 10)
			direction = 1;
	}

	return 0;
}
