/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdint.h>

const uint32_t icons48x20[][2*20] = {
		{	// LSB
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00111111111111111111111101111111, 0b11111111111111000000000000000000,
		0b01111100001111111111110000011110, 0b00000001111111100000000000000000,
		0b01111100001111111111000000000110, 0b00000000011111100000000000000000,
		0b11111100001111111110000000000010, 0b00000000001111110000000000000000,
		0b11111100001111111100000110000010, 0b00000000000111110000000000000000,
		0b11111100001111111100001111000010, 0b00011100000111110000000000000000,
		0b11111100001111111100000111111110, 0b00011110000111110000000000000000,

		0b11111100001111111110000001111110, 0b00011100001111110000000000000000,
		0b11111100001111111111000000011110, 0b00000000011111110000000000000000,
		0b11111100001111111111110000001110, 0b00000000011111110000000000000000,
		0b11111100001111111111111100000110, 0b00011100001111110000000000000000,
		0b11111100001111111111111110000010, 0b00011110000111110000000000000000,
		0b11111100001111111100001111000010, 0b00011100000111110000000000000000,
		0b11111100000000000100000110000010, 0b00000000000111110000000000000000,
		0b11111100000000000110000000000110, 0b00000000001111110000000000000000,

		0b01111100000000000111000000001110, 0b00000000011111100000000000000000,
		0b01111100000000000111100000011110, 0b00000001111111100000000000000000,
		0b00111111111111111111111101111111, 0b11111111111111000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
		{	// USB
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00111111111111111111111101111111, 0b11111111111111000000000000000000,
		0b01111000011110000111100000001110, 0b00000001111111100000000000000000,
		0b01111000011110000111000000000110, 0b00000000011111100000000000000000,
		0b11111000011110000110000000000010, 0b00000000001111110000000000000000,
		0b11111000011110000100000110000010, 0b00000000000111110000000000000000,
		0b11111000011110000100001111000010, 0b00011100000111110000000000000000,
		0b11111000011110000100000111111110, 0b00011110000111110000000000000000,

		0b11111000011110000110000011111110, 0b00011100001111110000000000000000,
		0b11111000011110000111000000111110, 0b00000000011111110000000000000000,
		0b11111000011110000111100000001110, 0b00000000011111110000000000000000,
		0b11111000011110000111111000000110, 0b00011100001111110000000000000000,
		0b11111000011110000111111110000010, 0b00011110000111110000000000000000,
		0b11111000001100000100001111000010, 0b00011100000111110000000000000000,
		0b11111000000000000100000110000010, 0b00000000000111110000000000000000,
		0b11111100000000001110000000000010, 0b00000000001111110000000000000000,

		0b01111110000000011111000000000110, 0b00000000011111100000000000000000,
		0b01111111000000111111100000001110, 0b00000001111111100000000000000000,
		0b00111111111111111111111101111111, 0b11111111111111000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
		{	// OFF
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00100000000000000000000000000000, 0b00000000000011000000000000000000,
		0b01000000011111000001111111111101, 0b11111111110000100000000000000000,
		0b10000001111111110001111111111101, 0b11111111110000100000000000000000,
		0b10000011111111111001111111111101, 0b11111111110000010000000000000000,
		0b10000111110001111101111111111101, 0b11111111110000010000000000000000,
		0b10000111100000111101111000000001, 0b11100000000000010000000000000000,
		0b10000111100000111101111000000001, 0b11100000000000010000000000000000,

		0b10000111100000111101111000000001, 0b11100000000000010000000000000000,
		0b10000111100000111101111111111001, 0b11111111100000010000000000000000,
		0b10000111100000111101111111111001, 0b11111111100000010000000000000000,
		0b10000111100000111101111111111001, 0b11111111100000010000000000000000,
		0b10000111100000111101111111111001, 0b11111111100000010000000000000000,
		0b10000111100000111101111000000001, 0b11100000000000010000000000000000,
		0b10000111110001111101111000000001, 0b11100000000000010000000000000000,
		0b10000011111111111001111000000001, 0b11100000000000010000000000000000,

		0b01000001111111110001111000000001, 0b11100000000000100000000000000000,
		0b01000000011111000001111000000001, 0b11100000000000100000000000000000,
		0b00110000000000000000000000000000, 0b00000000000011000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
		{	// SLOW
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00100000000000000000000000000000, 0b00000000000011000000000000000000,
		0b01000001111000011100000000111110, 0b00111000001110100000000000000000,
		0b10000111111110011100000001111111, 0b00111000001110100000000000000000,
		0b10001111111111011100000011111111, 0b10111000001110010000000000000000,
		0b10001111111111011100000011111111, 0b10111011101110010000000000000000,
		0b10001110001111011100000011100011, 0b10111011101110010000000000000000,
		0b10001111000111011100000011100011, 0b10111011101110010000000000000000,

		0b10000111110000011100000011100011, 0b10111011101110010000000000000000,
		0b10000011111100011100000011100011, 0b10111011101110010000000000000000,
		0b10000001111100011100000011100011, 0b10111011101110010000000000000000,
		0b10000000111110011100000011100011, 0b10111111111110010000000000000000,
		0b10001110001111011100000011100011, 0b10111111111110010000000000000000,
		0b10001111000111011100000011100011, 0b10011111111100010000000000000000,
		0b10001111111111011111111011111111, 0b10011111111100010000000000000000,
		0b10001111111111011111111011111111, 0b10011110111100010000000000000000,

		0b01000111111110011111111001111111, 0b00011100011100100000000000000000,
		0b01000001111000011111111000111110, 0b00011100011100100000000000000000,
		0b00110000000000000000000000000000, 0b00000000000011000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
		{	// MID
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00100000000000000000000000000000, 0b00000000000011000000000000000000,
		0b01000111100000001111011111111011, 0b11111100000000100000000000000000,
		0b10000111110000011111011111111011, 0b11111111000000100000000000000000,
		0b10000111110000011111011111111011, 0b11111111110000010000000000000000,
		0b10000111111000111111000111100011, 0b11111111110000010000000000000000,
		0b10000111111000111111000111100011, 0b11000111111000010000000000000000,
		0b10000111111101111111000111100011, 0b11000011111000010000000000000000,

		0b10000111111111111111000111100011, 0b11000001111000010000000000000000,
		0b10000111111111111111000111100011, 0b11000001111000010000000000000000,
		0b10000111101111101111000111100011, 0b11000001111000010000000000000000,
		0b10000111101111101111000111100011, 0b11000001111000010000000000000000,
		0b10000111100111001111000111100011, 0b11000011111000010000000000000000,
		0b10000111100111001111000111100011, 0b11000111111000010000000000000000,
		0b10000111100111001111000111100011, 0b11111111110000010000000000000000,
		0b10000111100000001111011111111011, 0b11111111110000010000000000000000,

		0b01000111100000001111011111111011, 0b11111111000000100000000000000000,
		0b01000111100000001111011111111011, 0b11111100000000100000000000000000,
		0b00110000000000000000000000000000, 0b00000000000011000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
		{	// FAST
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00100000000000000000000000000000, 0b00000000000011000000000000000000,
		0b01001111111110001111000000011110, 0b00011111111100100000000000000000,
		0b10001111111110001111000001111111, 0b10011111111100100000000000000000,
		0b10001111111110011111100011111111, 0b11011111111100010000000000000000,
		0b10001111111110011111100011111111, 0b11011111111100010000000000000000,
		0b10001110000000011111100011100011, 0b11000011100000010000000000000000,
		0b10001110000000111001110011110001, 0b11000011100000010000000000000000,

		0b10001110000000111001110001111100, 0b00000011100000010000000000000000,
		0b10001111111100111001110000111111, 0b00000011100000010000000000000000,
		0b10001111111100111001110000011111, 0b00000011100000010000000000000000,
		0b10001111111100111001110000001111, 0b10000011100000010000000000000000,
		0b10001111111101111111111011100011, 0b11000011100000010000000000000000,
		0b10001110000001111111111011110001, 0b11000011100000010000000000000000,
		0b10001110000001111111111011111111, 0b11000011100000010000000000000000,
		0b10001110000001110000111011111111, 0b11000011100000010000000000000000,

		0b01001110000001110000111001111111, 0b10000011100000100000000000000000,
		0b01001110000001110000111000011110, 0b00000011100000100000000000000000,
		0b00110000000000000000000000000000, 0b00000000000011000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
		{	// WFM
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		0b00111111111111111111111101111111, 0b11111111111111000000000000000000,
		0b01111000011110000111100000001110, 0b00000001111111100000000000000000,
		0b01111000011110000111000000000110, 0b00000000011111100000000000000000,
		0b11111000011110000110000000000010, 0b00000000001111110000000000000000,
		0b11111000011110000100000110000010, 0b00000000000111110000000000000000,
		0b11111000011110000100001111000010, 0b00011100000111110000000000000000,
		0b11111000011110000100000111111110, 0b00011110000111110000000000000000,

		0b11111000011110000110000011111110, 0b00011100001111110000000000000000,
		0b11111000011110000111000000111110, 0b00000000011111110000000000000000,
		0b11111000011110000111100000001110, 0b00000000011111110000000000000000,
		0b11111000011110000111111000000110, 0b00011100001111110000000000000000,
		0b11111000011110000111111110000010, 0b00011110000111110000000000000000,
		0b11111000001100000100001111000010, 0b00011100000111110000000000000000,
		0b11111000000000000100000110000010, 0b00000000000111110000000000000000,
		0b11111100000000001110000000000010, 0b00000000001111110000000000000000,

		0b01111110000000011111000000000110, 0b00000000011111100000000000000000,
		0b01111111000000111111100000001110, 0b00000001111111100000000000000000,
		0b00111111111111111111111101111111, 0b11111111111111000000000000000000,
		0b00001111111111111111111111111111, 0b11111111111100001000000000000000,
		},
};

#if 0
const uint32_t icons64x24[][2*24] = {
		{	// LSB
		0b00001111111111111111111111111111, 0b11111111111111111111111111110000,
		0b00111111111111111111111111111111, 0b11111111111111111111111111111100,
		0b00111111111111111111111111111111, 0b11111111111111111111111111111100,
		0b01111111100001111111111111110000, 0b01111111000000000011111111111110,
		0b01111111100001111111111110000000, 0b00001111000000000000011111111110,
		0b11111111100001111111111100000000, 0b00000111000000000000001111111111,
		0b11111111100001111111111000000000, 0b00000011000000000000000111111111,
		0b11111111100001111111111000001111, 0b10000011000011111100000111111111,

		0b11111111100001111111111000000111, 0b11000011000011111110000111111111,
		0b11111111100001111111111000000001, 0b11111111000011111000000111111111,
		0b11111111100001111111111110000000, 0b11111111000000000000011111111111,
		0b11111111100001111111111111100000, 0b00111111000000000000111111111111,
		0b11111111100001111111111111111000, 0b00011111000000000000011111111111,
		0b11111111100001111111111111111110, 0b00000111000011111000001111111111,
		0b11111111100001111111111000011111, 0b00000011000011111110000111111111,
		0b11111111100001111111111000001111, 0b10000011000011111100000111111111,

		0b11111111100000000000001000000000, 0b00000011000000000000000111111111,
		0b11111111100000000000001100000000, 0b00000111000000000000001111111111,
		0b01111111100000000000001111000000, 0b00001111000000000000011111111110,
		0b01111111100000000000001111111000, 0b01111111000000000011111111111110,
		0b00111111111111111111111111111111, 0b11111111111111111111111111111100,
		0b00111111111111111111111111111111, 0b11111111111111111111111111111100,
		0b00001111111111111111111111111111, 0b11111111111111111111111111110000,
		0b00000000000000000000000000000000, 0b00000000000000000000000000000000,
		},
};
const uint32_t icons64x16[][2*16] = {
		{	// LSB
		0b00001111111111111111111111111111, 0b11111111111111111111111111110000,
		0b00111111111111111111111111111111, 0b11111111111111111111111111111100,
		0b01110001111111111111111111100000, 0b00000111111000000000000001111110,
		0b01110001111111111111111100000000, 0b00000000111000000000000000011110,
		0b11110001111111111111110000000000, 0b00000000011000000000000000001111,
		0b11110001111111111111110000011111, 0b11111000011000111111111110001111,
		0b11110001111111111111111000000111, 0b11111100011000111111111100001111,
		0b11110001111111111111111110000000, 0b00011111111000000000000000011111,

		0b11110001111111111111111111110000, 0b00000011111000000000000000011111,
		0b11110001111111111111110001111111, 0b10000000111000111111111100001111,
		0b11110001111111111111110000111111, 0b11110000011000111111111110001111,
		0b11110000000000000000111000000000, 0b00000000011000000000000000001111,
		0b01110000000000000000111100000000, 0b00000000111000000000000000011110,
		0b01110000000000000000111111100000, 0b00000111111000000000000001111110,
		0b00111111111111111111111111111111, 0b11111111111111111111111111111100,
		0b00001111111111111111111111111111, 0b11111111111111111111111111110000,
		}
};

const uint32_t icons32x12[][12] = {
		{	// LSB
		0b00111111111111111111111111111000,
		0b01100111111110000111100000111100,
		0b11100111111100000011100000011110,
		0b11100111111000110001100111001110,
		0b11100111111001111001100111001110,
		0b11100111111000111111100000011110,
		0b11100111111100000111100000011110,
		0b11100111111111110000100111001110,

		0b11100111111001111000100111001110,
		0b11100000001000000000100000001110,
		0b01100000001110000011100000111100,
		0b00111111111111111111111111111000,
		},
};
#endif
