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

