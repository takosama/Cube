#pragma once
#include "IWritePin.h"
#include "ControllerManager.h"
#include "stm32f4xx_hal_tim.h"
class PwmWritePin :
	public IWritePin
{
 volatile	uint32_t * _pCCR{};
	bool _IsInited = false;
	TIM_HandleTypeDef* _htim{};
	int _Channel = 0;
	int _MaxValue = 0;
public:
	PwmWritePin();
	PwmWritePin(CtoCppPwmGPIO* PwmGPIO);
	~PwmWritePin();
	void Write(bool Value) override;
	void Write(int Value) override;
	void Write(float Value) override;
};

