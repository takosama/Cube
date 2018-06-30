#include "CtoCpp.h"
CtoCppGPIO ControlerPinsArray[ControlerPins_Count];
CtoCppGPIO CtoCppGPIOInit(GPIO_TypeDef* pGPIO, uint16_t Pin)
{
	CtoCppGPIO rtn;
	rtn.pGPIO = pGPIO;
	rtn.Pin = Pin;
	return rtn;
}