#include "DigitalWritePin.h"
#include "stm32f4xx_hal.h"

DigitalWritePin::DigitalWritePin()
= default;

DigitalWritePin::DigitalWritePin(CtoCppGPIO* pGPIO)
{
	this->_pGPIO = pGPIO;
	this->IsInited = true;
}


DigitalWritePin::~DigitalWritePin()
= default;

void DigitalWritePin::Write(bool Value)
{
	HAL_GPIO_WritePin(_pGPIO->pGPIO, _pGPIO->Pin,Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void DigitalWritePin::Write(int Value)
{
	Write(Value != 0);
}

void DigitalWritePin::Write(float Value)
{
	Write(Value >= 0.5);
}
