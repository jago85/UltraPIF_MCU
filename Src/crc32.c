// THIS SOURCE WAS PORTED FROM mupen64plus
// https://github.com/mupen64plus/mupen64plus-video-glide64/blob/master/src/CRC.cpp
// BELOW IS THE COPYRIGHT NOTICE OF THE ORIGINAL SOURCE
/*
*   Glide64 - Glide video plugin for Nintendo 64 emulators.
*   Copyright (c) 2002  Dave2001
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public
*   Licence along with this program; if not, write to the Free
*   Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
*   Boston, MA  02110-1301, USA
*/

#include "crc32.h"

unsigned int CRCTable[ 256 ];

#define CRC32_POLYNOMIAL     0x04C11DB7

static uint32_t Reflect(uint32_t ref, char ch )
{
   char i;
   uint32_t value = 0;
   /* Swap bit 0 for bit 7.
    * bit 1 for bit 6, etc.
    */
   for (i = 1; i < (ch + 1); i++)
   {
      if(ref & 1)
         value |= 1 << (ch - i);
      ref >>= 1;
   }
   return value;
}

void CRC_BuildTable(void)
{
   int i, j;
   uint32_t crc;

   for (i = 0; i < 256; i++)
   {
      crc = Reflect( i, 8 ) << 24;
      for (j = 0; j < 8; j++)
         crc = (crc << 1) ^ (crc & (1 << 31) ? CRC32_POLYNOMIAL : 0);
      CRCTable[i] = Reflect( crc, 32 );
   }
}

uint32_t CRC_Calculate(void *buffer, uint32_t count)
{
   uint32_t crc = 0xffffffff;
   uint8_t *p = (uint8_t*) buffer;
   while (count--)
      crc = (crc >> 8) ^ CRCTable[(crc & 0xFF) ^ *p++];
   return ~crc;
}

void CRC_Initialize(uint32_t *crc)
{
    *crc = 0xffffffff;
}

void CRC_Finalize(uint32_t *crc)
{
    *crc = ~(*crc);
}

void CRC_CalculateCont(uint32_t *crc, void *buffer, uint32_t count)
{
    uint8_t *p = (uint8_t*) buffer;
    uint32_t crcTemp = *crc;
    while (count--)
        crcTemp = (crcTemp >> 8) ^ CRCTable[(crcTemp & 0xFF) ^ *p++];
    *crc = crcTemp;
}
