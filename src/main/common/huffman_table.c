/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "huffman.h"

/*
 * Huffman Table, used to compress a bytestream
 */
const huffmanTable_t huffmanTable[HUFFMAN_TABLE_SIZE] = {
//   Len    Code       Char Bitcode
    {  2, 0xC000 }, // 0x00 11
    {  3, 0xA000 }, // 0x01 101
    {  4, 0x9000 }, // 0x02 1001
    {  5, 0x8800 }, // 0x03 10001
    {  5, 0x8000 }, // 0x04 10000
    {  6, 0x7400 }, // 0x05 011101
    {  6, 0x7000 }, // 0x06 011100
    {  6, 0x6C00 }, // 0x07 011011
    {  6, 0x6800 }, // 0x08 011010
    {  7, 0x6200 }, // 0x09 0110001
    {  7, 0x6000 }, // 0x0A 0110000
    {  7, 0x5E00 }, // 0x0B 0101111
    {  7, 0x5C00 }, // 0x0C 0101110
    {  7, 0x5A00 }, // 0x0D 0101101
    {  7, 0x5800 }, // 0x0E 0101100
    {  7, 0x5600 }, // 0x0F 0101011
    {  6, 0x6400 }, // 0x10 011001
    {  7, 0x5400 }, // 0x11 0101010
    {  7, 0x5200 }, // 0x12 0101001
    {  8, 0x5100 }, // 0x13 01010001
    {  8, 0x5000 }, // 0x14 01010000
    {  8, 0x4F00 }, // 0x15 01001111
    {  8, 0x4E00 }, // 0x16 01001110
    {  8, 0x4D00 }, // 0x17 01001101
    {  8, 0x4C00 }, // 0x18 01001100
    {  8, 0x4B00 }, // 0x19 01001011
    {  8, 0x4A00 }, // 0x1A 01001010
    {  8, 0x4900 }, // 0x1B 01001001
    {  8, 0x4800 }, // 0x1C 01001000
    {  8, 0x4700 }, // 0x1D 01000111
    {  8, 0x4600 }, // 0x1E 01000110
    {  8, 0x4500 }, // 0x1F 01000101
    {  8, 0x4400 }, // 0x20 01000100
    {  8, 0x4300 }, // 0x21 01000011
    {  8, 0x4200 }, // 0x22 01000010
    {  8, 0x4100 }, // 0x23 01000001
    {  8, 0x4000 }, // 0x24 01000000
    {  9, 0x3C80 }, // 0x25 001111001
    {  9, 0x3C00 }, // 0x26 001111000
    {  9, 0x3B80 }, // 0x27 001110111
    {  9, 0x3B00 }, // 0x28 001110110
    {  9, 0x3A80 }, // 0x29 001110101
    {  9, 0x3A00 }, // 0x2A 001110100
    {  9, 0x3980 }, // 0x2B 001110011
    {  9, 0x3900 }, // 0x2C 001110010
    {  9, 0x3880 }, // 0x2D 001110001
    {  9, 0x3800 }, // 0x2E 001110000
    {  9, 0x3780 }, // 0x2F 001101111
    {  8, 0x3F00 }, // 0x30 00111111
    {  9, 0x3700 }, // 0x31 001101110
    {  9, 0x3680 }, // 0x32 001101101
    {  9, 0x3600 }, // 0x33 001101100
    {  9, 0x3580 }, // 0x34 001101011
    {  9, 0x3500 }, // 0x35 001101010
    {  9, 0x3480 }, // 0x36 001101001
    {  9, 0x3400 }, // 0x37 001101000
    {  9, 0x3380 }, // 0x38 001100111
    {  9, 0x3300 }, // 0x39 001100110
    {  9, 0x3280 }, // 0x3A 001100101
    {  9, 0x3200 }, // 0x3B 001100100
    {  9, 0x3180 }, // 0x3C 001100011
    {  9, 0x3100 }, // 0x3D 001100010
    {  9, 0x3080 }, // 0x3E 001100001
    {  9, 0x3000 }, // 0x3F 001100000
    {  8, 0x3E00 }, // 0x40 00111110
    {  9, 0x2F80 }, // 0x41 001011111
    {  9, 0x2F00 }, // 0x42 001011110
    {  9, 0x2E80 }, // 0x43 001011101
    {  9, 0x2E00 }, // 0x44 001011100
    {  9, 0x2D80 }, // 0x45 001011011
    {  9, 0x2D00 }, // 0x46 001011010
    {  9, 0x2C80 }, // 0x47 001011001
    {  9, 0x2C00 }, // 0x48 001011000
    {  9, 0x2B80 }, // 0x49 001010111
    { 10, 0x27C0 }, // 0x4A 0010011111
    { 10, 0x2780 }, // 0x4B 0010011110
    {  9, 0x2B00 }, // 0x4C 001010110
    { 10, 0x2740 }, // 0x4D 0010011101
    { 10, 0x2700 }, // 0x4E 0010011100
    {  9, 0x2A80 }, // 0x4F 001010101
    {  5, 0x7800 }, // 0x50 01111
    {  9, 0x2A00 }, // 0x51 001010100
    { 10, 0x26C0 }, // 0x52 0010011011
    { 10, 0x2680 }, // 0x53 0010011010
    { 10, 0x2640 }, // 0x54 0010011001
    { 10, 0x2600 }, // 0x55 0010011000
    { 10, 0x25C0 }, // 0x56 0010010111
    { 10, 0x2580 }, // 0x57 0010010110
    { 10, 0x2540 }, // 0x58 0010010101
    { 10, 0x2500 }, // 0x59 0010010100
    { 10, 0x24C0 }, // 0x5A 0010010011
    { 10, 0x2480 }, // 0x5B 0010010010
    { 10, 0x2440 }, // 0x5C 0010010001
    { 10, 0x2400 }, // 0x5D 0010010000
    { 10, 0x23C0 }, // 0x5E 0010001111
    { 10, 0x2380 }, // 0x5F 0010001110
    { 10, 0x2340 }, // 0x60 0010001101
    { 10, 0x2300 }, // 0x61 0010001100
    { 10, 0x22C0 }, // 0x62 0010001011
    { 10, 0x2280 }, // 0x63 0010001010
    { 10, 0x2240 }, // 0x64 0010001001
    { 10, 0x2200 }, // 0x65 0010001000
    { 10, 0x21C0 }, // 0x66 0010000111
    { 10, 0x2180 }, // 0x67 0010000110
    { 10, 0x2140 }, // 0x68 0010000101
    { 10, 0x2100 }, // 0x69 0010000100
    { 10, 0x20C0 }, // 0x6A 0010000011
    { 10, 0x2080 }, // 0x6B 0010000010
    { 10, 0x2040 }, // 0x6C 0010000001
    { 10, 0x2000 }, // 0x6D 0010000000
    { 10, 0x1FC0 }, // 0x6E 0001111111
    { 10, 0x1F80 }, // 0x6F 0001111110
    { 10, 0x1F40 }, // 0x70 0001111101
    { 10, 0x1F00 }, // 0x71 0001111100
    { 10, 0x1EC0 }, // 0x72 0001111011
    { 10, 0x1E80 }, // 0x73 0001111010
    { 10, 0x1E40 }, // 0x74 0001111001
    { 10, 0x1E00 }, // 0x75 0001111000
    { 10, 0x1DC0 }, // 0x76 0001110111
    { 10, 0x1D80 }, // 0x77 0001110110
    { 10, 0x1D40 }, // 0x78 0001110101
    { 10, 0x1D00 }, // 0x79 0001110100
    { 10, 0x1CC0 }, // 0x7A 0001110011
    { 10, 0x1C80 }, // 0x7B 0001110010
    { 10, 0x1C40 }, // 0x7C 0001110001
    { 10, 0x1C00 }, // 0x7D 0001110000
    { 10, 0x1BC0 }, // 0x7E 0001101111
    { 10, 0x1B80 }, // 0x7F 0001101110
    {  9, 0x2980 }, // 0x80 001010011
    { 10, 0x1B40 }, // 0x81 0001101101
    { 10, 0x1B00 }, // 0x82 0001101100
    { 10, 0x1AC0 }, // 0x83 0001101011
    { 10, 0x1A80 }, // 0x84 0001101010
    { 10, 0x1A40 }, // 0x85 0001101001
    { 10, 0x1A00 }, // 0x86 0001101000
    { 10, 0x19C0 }, // 0x87 0001100111
    { 10, 0x1980 }, // 0x88 0001100110
    { 10, 0x1940 }, // 0x89 0001100101
    { 10, 0x1900 }, // 0x8A 0001100100
    { 10, 0x18C0 }, // 0x8B 0001100011
    { 10, 0x1880 }, // 0x8C 0001100010
    { 10, 0x1840 }, // 0x8D 0001100001
    { 10, 0x1800 }, // 0x8E 0001100000
    { 10, 0x17C0 }, // 0x8F 0001011111
    { 10, 0x1780 }, // 0x90 0001011110
    { 10, 0x1740 }, // 0x91 0001011101
    { 10, 0x1700 }, // 0x92 0001011100
    { 10, 0x16C0 }, // 0x93 0001011011
    { 10, 0x1680 }, // 0x94 0001011010
    { 10, 0x1640 }, // 0x95 0001011001
    { 10, 0x1600 }, // 0x96 0001011000
    { 10, 0x15C0 }, // 0x97 0001010111
    { 10, 0x1580 }, // 0x98 0001010110
    { 10, 0x1540 }, // 0x99 0001010101
    { 10, 0x1500 }, // 0x9A 0001010100
    { 10, 0x14C0 }, // 0x9B 0001010011
    { 10, 0x1480 }, // 0x9C 0001010010
    { 10, 0x1440 }, // 0x9D 0001010001
    { 10, 0x1400 }, // 0x9E 0001010000
    { 10, 0x13C0 }, // 0x9F 0001001111
    { 10, 0x1380 }, // 0xA0 0001001110
    { 10, 0x1340 }, // 0xA1 0001001101
    { 10, 0x1300 }, // 0xA2 0001001100
    { 10, 0x12C0 }, // 0xA3 0001001011
    { 10, 0x1280 }, // 0xA4 0001001010
    { 10, 0x1240 }, // 0xA5 0001001001
    { 10, 0x1200 }, // 0xA6 0001001000
    { 10, 0x11C0 }, // 0xA7 0001000111
    { 10, 0x1180 }, // 0xA8 0001000110
    { 10, 0x1140 }, // 0xA9 0001000101
    { 10, 0x1100 }, // 0xAA 0001000100
    { 10, 0x10C0 }, // 0xAB 0001000011
    { 10, 0x1080 }, // 0xAC 0001000010
    { 10, 0x1040 }, // 0xAD 0001000001
    { 10, 0x1000 }, // 0xAE 0001000000
    { 10, 0x0FC0 }, // 0xAF 0000111111
    { 10, 0x0F80 }, // 0xB0 0000111110
    { 10, 0x0F40 }, // 0xB1 0000111101
    { 10, 0x0F00 }, // 0xB2 0000111100
    { 10, 0x0EC0 }, // 0xB3 0000111011
    { 10, 0x0E80 }, // 0xB4 0000111010
    { 10, 0x0E40 }, // 0xB5 0000111001
    { 10, 0x0E00 }, // 0xB6 0000111000
    { 10, 0x0DC0 }, // 0xB7 0000110111
    { 10, 0x0D80 }, // 0xB8 0000110110
    { 10, 0x0D40 }, // 0xB9 0000110101
    { 10, 0x0D00 }, // 0xBA 0000110100
    { 10, 0x0CC0 }, // 0xBB 0000110011
    { 10, 0x0C80 }, // 0xBC 0000110010
    { 10, 0x0C40 }, // 0xBD 0000110001
    { 10, 0x0C00 }, // 0xBE 0000110000
    { 10, 0x0BC0 }, // 0xBF 0000101111
    { 10, 0x0B80 }, // 0xC0 0000101110
    { 10, 0x0B40 }, // 0xC1 0000101101
    { 10, 0x0B00 }, // 0xC2 0000101100
    { 10, 0x0AC0 }, // 0xC3 0000101011
    { 10, 0x0A80 }, // 0xC4 0000101010
    { 10, 0x0A40 }, // 0xC5 0000101001
    { 10, 0x0A00 }, // 0xC6 0000101000
    { 10, 0x09C0 }, // 0xC7 0000100111
    { 10, 0x0980 }, // 0xC8 0000100110
    { 10, 0x0940 }, // 0xC9 0000100101
    { 10, 0x0900 }, // 0xCA 0000100100
    { 10, 0x08C0 }, // 0xCB 0000100011
    { 10, 0x0880 }, // 0xCC 0000100010
    { 10, 0x0840 }, // 0xCD 0000100001
    { 10, 0x0800 }, // 0xCE 0000100000
    { 10, 0x07C0 }, // 0xCF 0000011111
    { 10, 0x0780 }, // 0xD0 0000011110
    { 10, 0x0740 }, // 0xD1 0000011101
    { 10, 0x0700 }, // 0xD2 0000011100
    { 10, 0x06C0 }, // 0xD3 0000011011
    { 10, 0x0680 }, // 0xD4 0000011010
    { 11, 0x0320 }, // 0xD5 00000011001
    { 10, 0x0640 }, // 0xD6 0000011001
    { 10, 0x0600 }, // 0xD7 0000011000
    { 10, 0x05C0 }, // 0xD8 0000010111
    { 10, 0x0580 }, // 0xD9 0000010110
    { 10, 0x0540 }, // 0xDA 0000010101
    { 10, 0x0500 }, // 0xDB 0000010100
    { 10, 0x04C0 }, // 0xDC 0000010011
    { 11, 0x0300 }, // 0xDD 00000011000
    { 10, 0x0480 }, // 0xDE 0000010010
    { 10, 0x0440 }, // 0xDF 0000010001
    {  9, 0x2900 }, // 0xE0 001010010
    { 10, 0x0400 }, // 0xE1 0000010000
    { 10, 0x03C0 }, // 0xE2 0000001111
    { 11, 0x02E0 }, // 0xE3 00000010111
    { 10, 0x0380 }, // 0xE4 0000001110
    { 11, 0x02C0 }, // 0xE5 00000010110
    { 11, 0x02A0 }, // 0xE6 00000010101
    { 11, 0x0280 }, // 0xE7 00000010100
    { 11, 0x0260 }, // 0xE8 00000010011
    { 11, 0x0240 }, // 0xE9 00000010010
    { 11, 0x0220 }, // 0xEA 00000010001
    { 11, 0x0200 }, // 0xEB 00000010000
    { 11, 0x01E0 }, // 0xEC 00000001111
    { 11, 0x01C0 }, // 0xED 00000001110
    { 11, 0x01A0 }, // 0xEE 00000001101
    { 10, 0x0340 }, // 0xEF 0000001101
    {  8, 0x3D00 }, // 0xF0 00111101
    {  9, 0x2880 }, // 0xF1 001010001
    { 11, 0x0180 }, // 0xF2 00000001100
    { 11, 0x0160 }, // 0xF3 00000001011
    { 11, 0x0140 }, // 0xF4 00000001010
    { 11, 0x0120 }, // 0xF5 00000001001
    { 11, 0x0100 }, // 0xF6 00000001000
    { 11, 0x00E0 }, // 0xF7 00000000111
    { 11, 0x00C0 }, // 0xF8 00000000110
    { 12, 0x0010 }, // 0xF9 000000000001
    { 11, 0x00A0 }, // 0xFA 00000000101
    { 11, 0x0080 }, // 0xFB 00000000100
    { 11, 0x0060 }, // 0xFC 00000000011
    { 11, 0x0040 }, // 0xFD 00000000010
    { 11, 0x0020 }, // 0xFE 00000000001
    {  9, 0x2800 }, // 0xFF 001010000
    { 12, 0x0000 }, // EOF  000000000000
};

