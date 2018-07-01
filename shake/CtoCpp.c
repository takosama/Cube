#include "CtoCpp.h"
CtoCppGPIO ControlerPinsArray[ControlerPins_Count];
CtoCppGPIO MoterDirPinsArray[MoterDirPins_Count];
 CtoCppPwmGPIO MoterPwmPinsArray[MoterPwmPins_Count];


CtoCppGPIO CtoCppGPIOInit(GPIO_TypeDef* pGPIO, uint16_t Pin)
{
	CtoCppGPIO rtn;
	rtn.pGPIO = pGPIO;
	rtn.Pin = Pin;
	return rtn;
}

CtoCppPwmGPIO CtoCppPwmGPIOInit(TIM_HandleTypeDef* htim, int Channel)
{
	CtoCppPwmGPIO rtn;
	rtn.Channel = Channel;
	rtn.htim = htim;
	return rtn;
}
