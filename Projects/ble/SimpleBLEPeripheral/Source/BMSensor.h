#include "hal_types.h"

void Wait4us(uint8 num);
int16 tempRead(void);
int16 tempReadADC(void);
uint8 humRead(void);
uint8 pmRead( void );
void GetPMRGB(uint8 pmRaw, uint8 *color);