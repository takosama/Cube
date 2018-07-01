#pragma once
#include "IWritePin.h"
#include "CtoCpp.h"

class DigitalWritePin :
	public IWritePin
{
	CtoCppGPIO* _pGPIO{};
	bool IsInited = false;
public:
	DigitalWritePin();
	DigitalWritePin(CtoCppGPIO* pGPIO);
	~DigitalWritePin();
	void Write(bool Value) override;
	void Write(int Value) override;
	void Write(float Value) override;
};

