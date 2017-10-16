/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

#ifndef _AQ_DATA_STORAGE_H_
#define _AQ_DATA_STORAGE_H_
#include <EEPROM.h>
// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address) {
    
    union floatStore {
      byte floatByte[4];
      unsigned short floatUShort[2];
      float floatVal;
    } floatOut;

#ifdef EEPROM_USES_16BIT_WORDS
    for (int i = 0; i < 2; i++) {
      floatOut.floatUShort[i] = EEPROM.read(address + 2*i);
    }
#else
    for (int i = 0; i < 4; i++) {
      floatOut.floatByte[i] = EEPROM.read(address + i);
    }
#endif

    return floatOut.floatVal;
}

void nvrWriteFloat(float value, int address) {
  
  union floatStore {
    byte floatByte[4];
    unsigned short floatUShort[2];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    EEPROM.write(address + 2*i, floatIn.floatUShort[i]);
  }
#else
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, floatIn.floatByte[i]);
  }
#endif
}

long nvrReadLong(int address) {
  union longStore {
    byte longByte[4];
    unsigned short longUShort[2];
    long longVal;
  } longOut;  

#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    longOut.longUShort[i] = EEPROM.read(address + 2*i);
  }
#else
  for (byte i = 0; i < 4; i++) {
    longOut.longByte[i] = EEPROM.read(address + i);
  }
#endif
    
  return longOut.longVal;
}

void nvrWriteLong(long value, int address) {
  union longStore {
    byte longByte[4];
    unsigned short longUShort[2];
    long longVal;
  } longIn;  

  longIn.longVal = value;
  
#ifdef EEPROM_USES_16BIT_WORDS
  for (int i = 0; i < 2; i++) {
    EEPROM.write(address + 2*i, longIn.longUShort[i]);
  }
#else
  for (int i = 0; i < 4; i++) {
    EEPROM.write(address + i, longIn.longByte[i]);
  }
#endif
}

#endif // _AQ_DATA_STORAGE_H_

