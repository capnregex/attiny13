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

int main(void)
{
    int i;
    DDRB |= 0b00011111;
//    for(i = 0; i < 5; i++) {
//      DDRB |= (1 << i);		/* Set PORTB bit 4 to output. */
//    }

    while (1) {
        for(i = 0; i < 5; i++) {
          PORTB |= (1 << i);	
          _delay_ms(200);
          PORTB &= ~(1 << i);
        }
    }

    return 0;
}
