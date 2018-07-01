#include "Moter.h"
#include <cstdlib>
#include "DigitalWritePin.h"
#include "PwmWritePin.h"


Moter::Moter(DigitalWritePin* DirPin, PwmWritePin* PwmPin)
{
	this->_DirPin = DirPin;
	this->_PwmPin = PwmPin;
}

Moter::Moter()
= default;


Moter::~Moter()
= default;

void Moter::Move(float speed)
{
	if (speed > 1 || speed < -1)
		return;
	float absSpeed = speed;
	if (absSpeed < 0) absSpeed *= -1;

	if (speed > 0)
	{
		this->_DirPin->Write(0);
		this->_PwmPin->Write(absSpeed);
	}
	else
	{
		this->_DirPin->Write(1);
		this->_PwmPin->Write(absSpeed);
	}
}
