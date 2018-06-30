#include "DigitalReadPin.h"



DigitalReadPin::DigitalReadPin()
= default;

DigitalReadPin::DigitalReadPin(CtoCppGPIO* pGPIO)
{
	this->_pCppGPIO = pGPIO;
	this->_isInited = true;
}


DigitalReadPin::~DigitalReadPin()
= default;

bool DigitalReadPin::ReadBool()
{
	return !HAL_GPIO_ReadPin(this->_pCppGPIO->pGPIO, this->_pCppGPIO->Pin) ;
}

int DigitalReadPin::ReadInt()
{
	return !this->ReadBool() ? 0 : 1;
}

float DigitalReadPin::ReadFloat()
{
	return this->ReadInt();
}
