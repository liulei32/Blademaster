#include "hal_types.h"


// Battery States
#define BATT_NORMAL     0
#define BATT_CHARGING   1
#define BATT_FULL       2
#define BATT_LOW        3
#define BATT_DEBUG      4

void Wait4us(uint8 num);
int16 tempRead(void);
int16 tempReadADC(void);
uint8 humRead(void);
uint8 pmRead( void );
uint8 battRead( void );
uint8 battUpdate( uint8 battVolt );
void GetPMRGB(uint8 pmRaw, uint8 *color);