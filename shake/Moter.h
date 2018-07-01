#pragma once
#include "CtoCpp.h"
#include "PwmWritePin.h"
#include "DigitalWritePin.h"

class Moter
{
	PwmWritePin	* _PwmPin;
	DigitalWritePin* _DirPin;
public:
	Moter(DigitalWritePin* DirPin, PwmWritePin* PwmPin);
	Moter();
	~Moter();
	void Move(float speed);
};

