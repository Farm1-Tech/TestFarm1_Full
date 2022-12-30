#pragma once

#include "Arduino.h"

void I2CSetup() ;
int I2CWrite(uint8_t addr, uint8_t *data, uint8_t len, bool stop = true) ;
int I2CRead(uint8_t addr, uint8_t *writeData, uint8_t writeLen, uint8_t *readData, uint8_t readLen) ;
int I2CMemWrite(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) ;
int I2CMemRead(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) ;

int I2CMemWrite(uint8_t addr, uint8_t reg, uint8_t data) ;
int I2CMemRead(uint8_t addr, uint8_t reg, uint8_t *data) ;

int I2CAddressCStringToInt(const char* str) ;
bool CheckI2CDevice(int addr) ;
