#include "PwmWritePin.h"



PwmWritePin::PwmWritePin()
= default;

PwmWritePin::PwmWritePin(CtoCppPwmGPIO* PwmGPIO)
{
	this->_htim =PwmGPIO-> htim;
	this->_IsInited = true;
	this->_Channel =PwmGPIO-> Channel;

	if (_Channel == 0)
		this->_pCCR = &this->_htim->Instance->CCR1;
	if (_Channel == 0x4)
		this->_pCCR = &this->_htim->Instance->CCR2;
	if (_Channel == 0x8)
		this->_pCCR = &this->_htim->Instance->CCR3;
	if (_Channel == 0xC)
		this->_pCCR = &this->_htim->Instance->CCR4;

	this->_MaxValue = _htim->Init.Period;
	HAL_TIM_PWM_Start(this->_htim, this->_Channel);
}

PwmWritePin::~PwmWritePin()
= default;

void PwmWritePin::Write(bool Value)
{
	*this->_pCCR = Value ? this->_MaxValue : 3;
}

void PwmWritePin::Write(int Value)
{
	*this->_pCCR = Value == 0 ? 3 : this->_MaxValue;
}

void PwmWritePin::Write(float Value)
{
	float value = this->_MaxValue * Value;
	if (value < 3) value = 3;
	*this->_pCCR = value;
}
