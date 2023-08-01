/*
 Author  : Juber Choudhari

*/

#ifndef ADS126X_H 
#define ADS126X_H

#include <Arduino.h>

////////////////////////// ADS126X  Constant  ///////////////
#define ADS126X_COMMAND_NOP (0x00)
#define ADS126X_COMMAND_RESET (0x06)
#define ADS126X_COMMAND_START (0x08)
#define ADS126X_COMMAND_STOP (0x0A)
#define ADS126X_COMMAND_RDATA (0x12)
#define ADS126X_COMMAND_SYOCAL (0x16)
#define ADS126X_COMMAND_GANCAL (0x17)
#define ADS126X_COMMAND_SFOCAL (0x19)
#define ADS126X_COMMAND_PREG (0x20)
#define ADS126X_COMMAND_WREG (0x40)
#define ADS126X_COMMAND_LOCK (0xF2)
#define ADS126X_COMMAND_UNLOCK (0xF5)
//////////////////////////// ADS126X Register////////////////
#define ADS126X_ID (0x00)
#define ADS126X_STATUS (0x01)
#define ADS126X_MODE0 (0x02)
#define ADS126X_MODE1 (0x03)
#define ADS126X_MODE2 (0x04)
#define ADS126X_MODE3 (0x05)
#define ADS126X_REF (0x06)
#define ADS126X_OFCAL0 (0x07)
#define ADS126X_OFCAL1 (0x08)
#define ADS126X_OFCAL2 (0x09)
#define ADS126X_FSCAL0 (0x0A)
#define ADS126X_FSCAL1 (0x0B)
#define ADS126X_FSCAL2 (0x0C)
#define ADS126X_IMUX (0x0D)
#define ADS126X_IMAG (0x0E)
#define ADS126X_RESERVED (0x0F)
#define ADS126X_PGA (0x10)
#define ADS126X_INPMUX (0x11)
#define ADS126X_INPBIAS (0x12)
/////////////////////////////////////////////////////
#define ADS126X_FILTER_SINC1 (0x00)
#define ADS126X_FILTER_SINC2 (0x01)
#define ADS126X_FILTER_SINC3 (0x02)
#define ADS126X_FILTER_SINC4 (0x03)
#define ADS126X_FILTER_FIR (0x04)  // default

#define ADS126X_DR_2_5_SPS (0x00)
#define ADS126X_DR_5_SPS (0x01)
#define ADS126X_DR_10_SPS (0x02)
#define ADS126X_DR_16_6_SPS (0x03)
#define ADS126X_DR_20_SPS (0x04)  //default
#define ADS126X_DR_50_SPS (0x05)
#define ADS126X_DR_60_SPS (0x06)
#define ADS126X_DR_100_SPS (0x07)
#define ADS126X_DR_400_SPS (0x08)
#define ADS126X_DR_1200_SPS (0x09)
#define ADS126X_DR_2400_SPS (0x0A)
#define ADS126X_DR_4800_SPS (0x0B)
#define ADS126X_DR_7200_SPS (0x0C)
#define ADS126X_DR_14400_SPS (0x0D)
#define ADS126X_DR_19200_SPS (0x0E)
#define ADS126X_DR_25600_SPS (0x0F)
#define ADS126X_DR_40000_SPS (0x10)

#define ADS126X_CHOP_NORMAL_MODE (0x00)  // default
#define ADS126X_CHOP_CHOP_MODE (0x01)
#define ADS126X_CHOP_2WIRE_MODE (0x02)
#define ADS126X_CHOP_3WIRE_MODE (0x03)

#define ADS126X_CONVRT_CONTINUOUS_CONVERSION (0x00)  // default
#define ADS126X_CONVRT_PULSE_CONVERSION (0x01)

#define ADS126X_DELAY_0_US (0x00)
#define ADS126X_DELAY_50_US (0x01)
#define ADS126X_DELAY_59_US (0x02)
#define ADS126X_DELAY_67_US (0x03)
#define ADS126X_DELAY_85_US (0x04)
#define ADS126X_DELAY_119_US (0x05)
#define ADS126X_DELAY_189_US (0x06)
#define ADS126X_DELAY_328_US (0x07)
#define ADS126X_DELAY_605_US (0x08)
#define ADS126X_DELAY_1_16_MS (0x09)
#define ADS126X_DELAY_2_27_MS (0x0A)
#define ADS126X_DELAY_4_49_MS (0x0B)
#define ADS126X_DELAY_8_93_MS (0x0C)
#define ADS126X_DELAY_17_8_MS (0x0D)


#define GPIO_DIR_0_INPUT (0x00)
#define GPIO_DIR_0_OUTPUT (0x01)

#define GPIO_DIR_1_INPUT (0x00)
#define GPIO_DIR_1_OUTPUT (0x01)

#define GPIO_DIR_2_INPUT (0x00)
#define GPIO_DIR_2_OUTPUT (0x01)

#define GPIO_DIR_3_INPUT (0x00)
#define GPIO_DIR_3_OUTPUT (0x01)

#define GPIO_DIR_3_INPUT (0x00)
#define GPIO_DIR_3_OUTPUT (0x01)

#define GPIO_CON_0_AIN2_NOT_CONNECTED (0x00)
#define GPIO_CON_0_AIN2_CONNECTED (0x01)

#define GPIO_CON_1_AIN3_NOT_CONNECTED (0x00)
#define GPIO_CON_1_AIN3_CONNECTED (0x01)

#define GPIO_CON_2_AIN4_NOT_CONNECTED (0x00)
#define GPIO_CON_2_AIN4_CONNECTED (0x01)

#define GPIO_CON_3_AIN5_NOT_CONNECTED (0x00)
#define GPIO_CON_3_AIN5_CONNECTED (0x01)

#define GPIO_DAT_0_AIN3_LOW (0x00)
#define GPIO_DAT_0_AIN3_HIGH (0x01)

#define GPIO_DAT_1_AIN3_LOW (0x00)
#define GPIO_DAT_1_AIN3_HIGH (0x01)

#define GPIO_DAT_2_AIN4_LOW (0x00)
#define GPIO_DAT_2_AIN4_HIGH (0x01)

#define GPIO_DAT_3_AIN5_LOW (0x00)
#define GPIO_DAT_3_AIN5_HIGH (0x01)

#define SPITIM_AUTO_DISENABLE (0x00)
#define SPITIM_AUTO_ENABLE (0x01)


#define CRCENB_DISENABLE (0x00)
#define CRCENB_ENABLE (0x01)

#define STATENB_DISENABLE (0x00)
#define STATENB_ENABLE (0x01)

#define PWDN_NORMAL_MODE (0x00)
#define PWDN_SOFTAWRE_MODE (0x01)



#define RMUXN_INTERNAL (0x00)
#define RMUXN_AVSS_INTERNAL (0x01)
#define RMUXN_AIN1_EXTERNAL (0x02)
#define RMUXN_AIN3_EXTERNAL (0x03)

#define RMUXP_INTERNAL (0x00)
#define RMUXP_AVSS_INTERNAL (0x01)
#define RMUXP_AIN0_EXTERNAL (0x02)
#define RMUXP_AIN2_EXTERNAL (0x03)

#define REFENB_ENTERNAL_DISENABLE (0x00)
#define REFENB_ENTERNAL_ENABLE (0x01)


#define IMUX1_AIN0 (0x00)
#define IMUX1_AIN1 (0x01)
#define IMUX1_AIN2 (0x02)
#define IMUX1_AIN3 (0x03)
#define IMUX1_AIN4 (0x04)
#define IMUX1_AIN5 (0x05)
#define IMUX1_AIN6 (0x06)
#define IMUX1_AIN7 (0x07)
#define IMUX1_AIN8 (0x08)
#define IMUX1_AIN9 (0x09)
#define IMUX1_AINCOM (0x0A)

#define IMUX1_NO_CONNECTION (0x0F)

#define IMUX2_AIN0 (0x00)
#define IMUX2_AIN1 (0x01)
#define IMUX2_AIN2 (0x02)
#define IMUX2_AIN3 (0x03)
#define IMUX2_AIN4 (0x04)
#define IMUX2_AIN5 (0x05)
#define IMUX2_AIN6 (0x06)
#define IMUX2_AIN7 (0x07)
#define IMUX2_AIN8 (0x08)
#define IMUX2_AIN9 (0x09)
#define IMUX2_AINCOM (0x0A)

#define IMUX2_NO_CONNECTION (0x0F)

#define IMAG1_OFF (0x00)
#define IMAG1_50UA (0x01)
#define IMAG1_100UA (0x02)
#define IMAG1_250UA (0x03)
#define IMAG1_500UA (0x04)
#define IMAG1_750UA (0x05)
#define IMAG1_1000UA (0x06)
#define IMAG1_1500UA (0x07)
#define IMAG1_2000UA (0x08)
#define IMAG1_2500UA (0x09)
#define IMAG1_3000UA (0x0A)



#define IMAG2_OFF (0x00)
#define IMAG2_50UA (0x01)
#define IMAG2_100UA (0x02)
#define IMAG2_250UA (0x03)
#define IMAG2_500UA (0x04)
#define IMAG2_750UA (0x05)
#define IMAG2_1000UA (0x06)
#define IMAG2_1500UA (0x07)
#define IMAG2_2000UA (0x08)
#define IMAG2_2500UA (0x09)
#define IMAG2_3000UA (0x0A)


#define PGA_GAIN_1 (0x00)
#define PGA_GAIN_2 (0x01)
#define PGA_GAIN_4 (0x02)
#define PGA_GAIN_8 (0x03)
#define PGA_GAIN_16 (0x04)
#define PGA_GAIN_32 (0x05)
#define PGA_GAIN_64 (0x06)
#define PGA_GAIN_128 (0x07)

#define PGA_MODE (0x00)  // default
#define PGA_BYPASS (0x01)

#define INPMUX_MUXP_AINCOM (0x00)
#define INPMUX_MUXP_AIN0 (0x01)
#define INPMUX_MUXP_AIN1 (0x02)
#define INPMUX_MUXP_AIN2 (0x03)
#define INPMUX_MUXP_AIN3 (0x04)
#define INPMUX_MUXP_AIN4 (0x05)
#define INPMUX_MUXP_AIN5 (0x06)
#define INPMUX_MUXP_AIN6 (0x07)
#define INPMUX_MUXP_AIN7 (0x08)
#define INPMUX_MUXP_AIN8 (0x09)
#define INPMUX_MUXP_AIN9 (0x0A)
#define INPMUX_MUXP_TEMP (0x0B)
#define INPMUX_MUXP_INTERNAL_AVDD_AVSS_4 (0x0C)
#define INPMUX_MUXP_INTERNAL_DVDD_4 (0x0D)
#define INPMUX_MUXP_OPEN (0x0E)
#define INPMUX_MUXP_INTERNAL_VCOM (0x0F)

#define INPMUX_MUXN_AINCOM (0x00)
#define INPMUX_MUXN_AIN0 (0x01)
#define INPMUX_MUXN_AIN1 (0x02)
#define INPMUX_MUXN_AIN2 (0x03)
#define INPMUX_MUXN_AIN3 (0x04)
#define INPMUX_MUXN_AIN4 (0x05)
#define INPMUX_MUXN_AIN5 (0x06)
#define INPMUX_MUXN_AIN6 (0x07)
#define INPMUX_MUXN_AIN7 (0x08)
#define INPMUX_MUXN_AIN8 (0x09)
#define INPMUX_MUXN_AIN9 (0x0A)
#define INPMUX_MUXN_TEMP (0x0B)
#define INPMUX_MUXN_INTERNAL_AVDD_AVSS_4 (0x0C)
#define INPMUX_MUXN_INTERNAL_DVDD_4 (0x0D)
#define INPMUX_MUXN_OPEN (0x0E)
#define INPMUX_MUXN_INTERNAL_VCOM (0x0F)

#define BOCS_OFF (0x00)
#define BOCS_50NA (0x01)
#define BOCS_200NA (0x02)
#define BOCS_1UA (0x03)
#define BOCS_10UA (0x04)


#define BOCSP_PULL_UP_MODE (0x00)
#define BOCP_PULL_DOWN_MODE (0x01)

#define VBIAS_DISABLE (0x00)
#define VBIAS_ENABLE (0x01)`

#define CONVERSION_TIMEOUT (1000)  // 1 sec for timeout

union ThreeBytesTo24Bit {
  unsigned char bytes[3];
  uint32_t value;
};



typedef union {

  struct {
    uint8_t REV_ID : 4;
    uint8_t DEV_ID : 4;
  } bit;
  uint8_t reg;
} ADS126X_ID_Type;

typedef union {
  struct
  {
    uint8_t RESET : 1;
    uint8_t CLOCK : 1;
    uint8_t DRDY : 1;
    uint8_t REFL_ALM : 1;
    uint8_t PGAH_ALM : 1;
    uint8_t PGAL_ALM : 1;
    uint8_t CRCERR : 1;
    uint8_t LOCK : 1;
  } bit;
  uint8_t reg;

} ADS126X_STATUS_Type;


typedef union {
  struct {
    uint8_t FILTER : 3;
    uint8_t DR : 5;
  } bit;
  uint8_t reg;
} ADS126X_MODE0_Type;


typedef union {
  struct
  {
    uint8_t DELAY : 4;
    uint8_t CONVRT : 1;
    uint8_t CHOP : 2;
    uint8_t RES : 1;
  } bit;
  uint8_t reg;
} ADS126X_MODE1_Type;


typedef union {
  struct {
    uint8_t GPIO0_DIR : 1;
    uint8_t GPIO1_DIR : 1;
    uint8_t GPIO2_DIR : 1;
    uint8_t GPIO3_DIR : 1;

    uint8_t GPIO0_CON : 1;
    uint8_t GPIO1_CON : 1;
    uint8_t GPIO2_CON : 1;
    uint8_t GPIO3_CON : 1;
  } bit;
  uint8_t reg;
} ADS126X_MODE2_Type;


typedef union {
  struct
  {
    uint8_t GPIO0_DAT : 1;
    uint8_t GPIO1_DAT : 1;
    uint8_t GPIO2_DAT : 1;
    uint8_t GPIO3_DAT : 1;

    uint8_t SPITIM : 1;
    uint8_t CRCENB : 1;
    uint8_t STATENB : 1;
    uint8_t PWDN : 1;
  } bit;
  uint8_t reg;
} ADS126X_MODE3_Type;

typedef union {
  struct
  {
    uint8_t RMUXN : 2;
    uint8_t RMUXP : 2;
    uint8_t REFENB : 1;
    uint8_t RESERVED : 3;
  } bit;

  uint8_t reg;
} ADS126X_REF_Type;

typedef union {
  struct
  {
    uint8_t OFC0;
    uint8_t OFC1;
    uint8_t OFC2;
  } byte;

  int32_t OFC;
} ADS126X_OFCAL_Type;


typedef union {
  struct
  {
    uint8_t FSC0;
    uint8_t FSC1;
    uint8_t FSC2;
  } byte;
  int32_t FSCAL;

} ADS126X_FSCAL_Type;


typedef union {
  struct
  {
    uint8_t IMUX1 : 4;
    uint8_t IMUX2 : 4;
  } bit;
  uint8_t reg;
} ADS126X_IMUX_Type;

typedef union {
  struct
  {
    uint8_t IMAG1 : 4;
    uint8_t IMAG2 : 4;
  } bit;

  uint8_t reg;
} ADS126X_IMAG_Type;

typedef union {
  struct
  {
    uint8_t GAIN : 3;
    uint8_t RESERVED : 4;
    uint8_t BYPASS : 1;
  } bit;
  uint8_t reg;

} ADS126X_PGA_Type;


typedef union {
  struct
  {
    uint8_t MUXN : 4;
    uint8_t MUXP : 4;
  } bit;
  uint8_t reg;
} ADS126X_INPMUX_Type;


typedef union {
  struct
  {
    uint8_t BOCS : 3;
    uint8_t BOCSP : 1;
    uint8_t VBIAS : 1;
    uint8_t RESERVED : 3;
  };
  uint8_t reg;
} ADS126X_INPBIAS_Type;

typedef struct
{
  volatile ADS126X_ID_Type ID;
  volatile ADS126X_STATUS_Type STATUS;
  volatile ADS126X_MODE0_Type MODE0;
  volatile ADS126X_MODE1_Type MODE1;
  volatile ADS126X_MODE2_Type MODE2;
  volatile ADS126X_MODE3_Type MODE3;
  volatile ADS126X_REF_Type REF;
  volatile ADS126X_OFCAL_Type OFCAL;
  volatile ADS126X_FSCAL_Type FSCAL;
  volatile ADS126X_IMUX_Type IMUX;
  volatile ADS126X_IMAG_Type IMAG;
  volatile ADS126X_PGA_Type PGA;
  volatile ADS126X_INPMUX_Type INPMUX;
  volatile ADS126X_INPBIAS_Type INPBIAS;

} ADS126X_REGISTERS_Type;



class ADS126X {
private:
  uint8_t _chip_select = 10;

public:
  void begin();
  void begin(uint8_t ss);
  uint8_t readConfigRegister(uint8_t add);
  uint8_t writeConfigRegister(uint8_t add, uint8_t val);
  uint8_t sendCommand(uint8_t add);
  int32_t readConversionData();
  int32_t readChannel(ADS126X_REGISTERS_Type *reg);
  int32_t readChannel(uint8_t pos, uint8_t neg);
  int32_t readChannel(uint8_t pos, uint8_t neg, uint8_t gain);
  int32_t readChannel(uint8_t pos);
  uint8_t readAllRegisters(ADS126X_REGISTERS_Type *reg);
  uint8_t writeAllRegisters(ADS126X_REGISTERS_Type *reg);
  uint8_t writeCommand(uint8_t command_add);
  uint8_t readRegister(uint8_t reg_add);
  uint8_t writeRegister(uint8_t reg_add, uint8_t reg_val);
};

#endif
