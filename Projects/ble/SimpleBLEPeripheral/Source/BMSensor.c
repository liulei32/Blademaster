#include "BMSensor.h"
#include "hal_adc.h"
#include "hal_archerled.h"
#include "hal_i2c.h"

const static uint8 red[6] =   {0,   255, 255, 255, 153, 100};
const static uint8 green[6] = {228, 255, 126, 0,   0,   0  };
const static uint8 blue[6] =  {0,   0,   0,   0,   76,  120};

void Wait4us(uint8 num)
{
  uint8 target = T3CNT + num;
  T3CTL |= 0xF0; // Run timer
  while (T3CNT != target)
    ;
  T3CTL &= !0x10; // Suspend timer
}

int16 tempRead(void)
{
  // Turn on Si7015 CSn
  P0_1 = 0;
  // Get Temperature data
  HalI2CInit(0x40, i2cClock_123KHZ);
  uint8 wdata_start[2] = {0x03, 0x11};
  uint8 res = HalI2CWrite(2, wdata_start);
  
  uint8 wdata_status = 0x00;
  uint8 wdata_result = 0x01;
  uint8 rdata = 0x01;
  while ((rdata&0x01) == 0x01)
  {
    HalI2CWriteNoStop(1, &wdata_status);
    uint8 res = HalI2CRead(1, &rdata);
    if ((res & 0x01) != 0x01)
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    Wait4us(200);
  }
  uint8 rdata_result[2];
  HalI2CWriteNoStop(1, &wdata_result);
  HalI2CRead(2, rdata_result);
  // Turn off Si7015 CSn
  P0_1 = 1;
  
  uint16 temperature = ((uint16)rdata_result[0]<<8) | (uint16)rdata_result[1];
  temperature = temperature >> 2;
  return temperature;
}

int16 tempReadADC(void)
{
  ATEST = 0x01;
  TR0 = 0x01;
  
  // Get PM2.5 data
  HalAdcInit(); // Set ADC reference to VDD
  static int16 adc;
  adc = HalAdcRead (14, HAL_ADC_RESOLUTION_8); // smkSENSOR, Resolution = 8 bits
  
  return adc;
}

uint8 humRead(void)
{
  // Turn on Si7015 CSn
  P0_1 = 0;
  // Get Humidity data
  HalI2CInit(0x40, i2cClock_123KHZ);
  uint8 wdata_start[2] = {0x03, 0x01};
  uint8 res = HalI2CWrite(2, wdata_start);
  
  uint8 wdata_status = 0x00;
  uint8 wdata_result = 0x01;
  uint8 rdata = 0x01;
  while ((rdata&0x01) == 0x01)
  {
    HalI2CWriteNoStop(1, &wdata_status);
    uint8 res = HalI2CRead(1, &rdata);
    if ((res & 0x01) != 0x01)
      HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    Wait4us(200);
  }
  uint8 rdata_result[2];
  HalI2CWriteNoStop(1, &wdata_result);
  HalI2CRead(2, rdata_result);
  // Turn off Si7015 CSn
  P0_1 = 1;
  
  uint16 humility = ((uint16)rdata_result[0]<<8) | (uint16)rdata_result[1];
  humility = humility + rdata_result[1];
  humility = humility >> 4;
  uint8 humToShow = humility/16-24;
  return humToShow;
}

uint8 pmRead( void )
{
  // Get PM2.5 data
  HalAdcInit(); // Set ADC reference to VDD
  
  static uint16 adc;
  P0_7 = 1;
  Wait4us(70);
  adc = HalAdcRead (6, HAL_ADC_RESOLUTION_8); // smkSENSOR, Resolution = 8 bits
  Wait4us(10);
  P0_7 = 0;
  
  return (uint8)(adc);
}

uint8 battRead( void )
{
  // Get battery voltage data
  HalAdcInit(); // Set ADC reference to VDD
  
  static uint16 adc;
  adc = HalAdcRead (5, HAL_ADC_RESOLUTION_8); // smkSENSOR, Resolution = 8 bits
  
  return (uint8)(adc);
}

void GetPMRGB(uint8 pmRaw, uint8 *color)
{
  uint8 pmIndex = pmRaw / 13;
  if (pmIndex > 5)
    pmIndex = 5;
  color[0] = red[pmIndex];
  color[1] = green[pmIndex];
  color[2] = blue[pmIndex];
}