
/*
ADS126X.cpp - Library for ADS1260 and ADS1261 .
Created by Juber Choudhari ,July 30, 2023 

Realeased into the public domain
*/
#include "Arduino.h"
#include "ADS126X.h"
#include <SPI.h>

#define CS 10


void ADS126X::begin() {
  _chip_select = CS;
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2mhz
  SPI.setDataMode(SPI_MODE1);
}

void ADS126X ::begin(uint8_t ss) {
  _chip_select = ss;
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2mhz
  SPI.setDataMode(SPI_MODE1);
}
int32_t ADS126X ::readChannel(ADS126X_REGISTERS_Type* args) {
  int32_t result;
  writeAllRegisters(args);
  delayMicroseconds(10);
  result = readConversionData();
  return result;
}
int32_t ADS126X ::readChannel(uint8_t pos, uint8_t neg) {
  ADS126X_INPMUX_Type inp;
  ADS126X_PGA_Type pga;
  int32_t result;


  inp.bit.MUXP = pos;
  inp.bit.MUXN = neg;
  pga.bit.GAIN = PGA_GAIN_1;

  writeConfigRegister(ADS126X_INPMUX, inp.reg);
  //Serial.println(inp.reg);
  writeConfigRegister(ADS126X_PGA, pga.reg);
  delayMicroseconds(10);
  result = readConversionData();
  return result;
}

int32_t ADS126X ::readChannel(uint8_t pos, uint8_t neg, uint8_t gain) {
  ADS126X_INPMUX_Type inp;
  ADS126X_PGA_Type pga;
  int32_t result;

  inp.bit.MUXP = pos;
  inp.bit.MUXN = neg;
  pga.bit.GAIN = gain;

  writeConfigRegister(ADS126X_INPMUX, inp.reg);

  writeConfigRegister(ADS126X_PGA, pga.reg);
  delayMicroseconds(10);
  result = readConversionData();
  return result;
}

int32_t ADS126X ::readChannel(uint8_t pos) {
  ADS126X_INPMUX_Type inp;
  ADS126X_PGA_Type pga;
  int32_t result;
  inp.bit.MUXP = pos;
  inp.bit.MUXN = INPMUX_MUXN_AINCOM;
  pga.bit.GAIN = PGA_GAIN_1;

  writeConfigRegister(ADS126X_INPMUX, inp.reg);
  writeConfigRegister(ADS126X_PGA, pga.reg);
  delayMicroseconds(10);
  result = readConversionData();
  return result;
}


int32_t ADS126X ::readConversionData() {
  ThreeBytesTo24Bit value;
  ADS126X_STATUS_Type status;
  digitalWrite(_chip_select, LOW);
  writeCommand(ADS126X_COMMAND_START);
  status.reg = readRegister(ADS126X_STATUS);
  unsigned long currentMillis = millis();

  while ((status.bit.DRDY == 0) && ((millis() - currentMillis) < CONVERSION_TIMEOUT)) {
    status.reg = readRegister(ADS126X_STATUS);
  }
  writeCommand(ADS126X_COMMAND_RDATA);
  value.bytes[2] = SPI.transfer(0x00);
  value.bytes[1] = SPI.transfer(0x00);
  value.bytes[0] = SPI.transfer(0x00);
  int32_t signedValue = value.value;

  if (signedValue & 0x800000) {
    signedValue = signedValue - 0x1000000;
  }
  writeCommand(ADS126X_COMMAND_STOP);
  digitalWrite(SS, HIGH);
  return signedValue;
}

uint8_t ADS126X::readAllRegisters(ADS126X_REGISTERS_Type* args) {
  digitalWrite(_chip_select, LOW);
  args->ID.reg = readRegister(ADS126X_ID);
  args->STATUS.reg = readRegister(ADS126X_STATUS);
  args->MODE0.reg = readRegister(ADS126X_MODE0);
  args->MODE1.reg = readRegister(ADS126X_MODE1);
  args->MODE2.reg = readRegister(ADS126X_MODE2);
  args->MODE3.reg = readRegister(ADS126X_MODE3);
  args->REF.reg = readRegister(ADS126X_REF);
  args->OFCAL.byte.OFC0 = readRegister(ADS126X_OFCAL0);
  args->OFCAL.byte.OFC1 = readRegister(ADS126X_OFCAL1);
  args->OFCAL.byte.OFC2 = readRegister(ADS126X_OFCAL2);

  args->FSCAL.byte.FSC0 = readRegister(ADS126X_FSCAL0);
  args->FSCAL.byte.FSC1 = readRegister(ADS126X_FSCAL1);
  args->FSCAL.byte.FSC2 = readRegister(ADS126X_FSCAL2);

  args->IMUX.reg = readRegister(ADS126X_IMUX);
  args->IMAG.reg = readRegister(ADS126X_IMAG);
  args->PGA.reg = readRegister(ADS126X_PGA);
  args->INPMUX.reg = readRegister(ADS126X_INPMUX);
  args->INPBIAS.reg = readRegister(ADS126X_INPBIAS);

  digitalWrite(_chip_select, HIGH);


  Serial.print("ID:");
  Serial.print(args->ID.reg);
  Serial.print("\t STATUS:");
  Serial.print(args->STATUS.reg);
  Serial.print("\t MODE0:");
  Serial.print(args->MODE0.reg);
  Serial.print("\t MODE1:");
  Serial.print(args->MODE1.reg);
  Serial.print("\t MODE2:");
  Serial.print(args->MODE2.reg);
  Serial.print("\t MODE3:");
  Serial.print(args->MODE3.reg);
  Serial.print("\t REF:");
  Serial.print(args->REF.reg);
  Serial.print("\t IMUX:");
  Serial.print(args->IMUX.reg);
  Serial.print("\t IMAG:");
  Serial.print(args->IMAG.reg);
  Serial.print("\t PGA:");
  Serial.print(args->PGA.reg);
  Serial.print("\t INPMUX:");
  Serial.print(args->INPMUX.reg);
  Serial.print("\t INPBIAS:");
  Serial.print(args->INPBIAS.reg);


  return true;
}

uint8_t ADS126X::writeAllRegisters(ADS126X_REGISTERS_Type* args) {
  digitalWrite(_chip_select, LOW);
  //writeRegister(ADS126X_ID,args->ID.reg );
  //writeRegister(ADS126X_STATUS);
  writeRegister(ADS126X_MODE0, args->MODE0.reg);
  writeRegister(ADS126X_MODE1, args->MODE1.reg);
  writeRegister(ADS126X_MODE2, args->MODE2.reg);
  writeRegister(ADS126X_MODE3, args->MODE3.reg);
  writeRegister(ADS126X_REF, args->REF.reg);
  writeRegister(ADS126X_OFCAL0, args->OFCAL.byte.OFC0);
  writeRegister(ADS126X_OFCAL1, args->OFCAL.byte.OFC1);
  writeRegister(ADS126X_OFCAL2, args->OFCAL.byte.OFC2);

  writeRegister(ADS126X_FSCAL0, args->FSCAL.byte.FSC0);
  writeRegister(ADS126X_FSCAL1, args->FSCAL.byte.FSC1);
  writeRegister(ADS126X_FSCAL2, args->FSCAL.byte.FSC2);

  writeRegister(ADS126X_IMUX, args->IMUX.reg);
  writeRegister(ADS126X_IMAG, args->IMAG.reg);
  writeRegister(ADS126X_PGA, args->PGA.reg);
  writeRegister(ADS126X_INPMUX, args->INPMUX.reg);
  writeRegister(ADS126X_INPBIAS, args->INPBIAS.reg);

  digitalWrite(_chip_select, HIGH);
}



uint8_t ADS126X ::readConfigRegister(uint8_t add) {
  digitalWrite(_chip_select, LOW);
  uint8_t data = readRegister(add);
  digitalWrite(_chip_select, HIGH);

  return data;
}

uint8_t ADS126X ::writeConfigRegister(uint8_t add, uint8_t val) {
  digitalWrite(_chip_select, LOW);
  uint8_t data = writeRegister(add, val);
  digitalWrite(_chip_select, HIGH);
  return data;
}
uint8_t ADS126X ::sendCommand(uint8_t add) {
  digitalWrite(_chip_select, LOW);
  uint8_t data = writeCommand(add);
  digitalWrite(_chip_select, HIGH);
  return data;
}



uint8_t ADS126X ::writeRegister(uint8_t reg_add, uint8_t reg_val) {
  uint8_t reg = ADS126X_COMMAND_WREG | reg_add;
  SPI.transfer(reg);
  byte echo_byte = SPI.transfer(reg_val);
  return echo_byte;
}

uint8_t ADS126X ::readRegister(uint8_t reg_add) {
  uint8_t reg_val = ADS126X_COMMAND_PREG | reg_add;
  SPI.transfer(reg_val);
  byte echo_byte = SPI.transfer(0x00);
  byte reg_data = SPI.transfer(0x00);
  return reg_data;
}

uint8_t ADS126X ::writeCommand(uint8_t command_add) {
  SPI.transfer(command_add);
  byte echo_byte = SPI.transfer(0x00);
  return echo_byte;
}
