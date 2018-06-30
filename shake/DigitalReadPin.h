#pragma once
#include "IReadPin.h"
#include "CtoCpp.h"

class DigitalReadPin :
	public IReadPin
{
	bool _isInited = false;
	CtoCppGPIO* _pCppGPIO{};
public:
	DigitalReadPin();
	DigitalReadPin(CtoCppGPIO* pGPIO);
	~DigitalReadPin();

	bool ReadBool() override;
	int ReadInt() override;
	float ReadFloat() override;
};

