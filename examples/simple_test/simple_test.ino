/*
created by juber choudhari 
test code of ads126x library

this example shows diffrent type to read channels 

*/
#include <ADS126X.h>

#define PWDN A2
#define RSTA A1
#define START A0

/********************************/
ADS126X adc;
ADS126X_REGISTERS_Type reg_map;
/********************************/

void setup() {
  Serial.begin(115200);
  /********* config ads other pins *********/
  pinMode(PWDN, OUTPUT);
  pinMode(RSTA, OUTPUT);
  pinMode(START, INPUT);
  digitalWrite(RSTA, HIGH);
  digitalWrite(PWDN, HIGH);
  digitalWrite(START, LOW);
  /*******************************************/
  adc.begin();
  adc.sendCommand(ADS126X_COMMAND_RESET);  // reset adc

  Serial.println("before config register");
  adc.readAllRegisters(&reg_map);  // this function read all ads126X register value and print in serial

  /****************** setting single short conversion mode *******/
  reg_map.MODE0.bit.FILTER = ADS126X_FILTER_FIR;
  reg_map.MODE0.bit.DR = ADS126X_DR_19200_SPS;


  reg_map.MODE1.bit.DELAY = ADS126X_DELAY_0_US;
  reg_map.MODE1.bit.CONVRT = ADS126X_CONVRT_PULSE_CONVERSION;
  reg_map.MODE1.bit.CHOP = ADS126X_CHOP_NORMAL_MODE;

  adc.writeConfigRegister(ADS126X_MODE0, reg_map.MODE0.reg);  // Mode0
  adc.writeConfigRegister(ADS126X_MODE1, reg_map.MODE1.reg);  // Mode1

  /****************************************************************/
  adc.readAllRegisters(&reg_map);  // this function read all ads126X register value and print in serial
}

void loop() {
 
  Serial.print(adc.readChannel(INPMUX_MUXP_AIN0, INPMUX_MUXN_AIN1));  // by deafult pga gain is 1
  delay(10);
  Serial.print(adc.readChannel(INPMUX_MUXP_AIN2, INPMUX_MUXN_AINCOM,PGA_GAIN_2));  // third parameter is pga gain 
  delay(10);
  Serial.print(adc.readChannel(INPMUX_MUXP_AIN3)); // 

}
