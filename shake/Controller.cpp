#include "Controller.h"
#include "stdlib.h"

Controller::Controller()
= default;

Controller::Controller(int PinCount)
{
	this->_PinCount = PinCount;
	this->_ReadPinArray = static_cast<IReadPin**>(malloc(sizeof(IReadPin*) * this->_PinCount));
	this->_isInited = true;
}

void Controller::SetPin(IReadPin * ReadPin, int ID)
{
	this->_ReadPinArray[ID] = ReadPin;
}

bool Controller::ReadStatus(int ID)
{
	return this->_ReadPinArray[ID]->ReadBool();
}

Controller::~Controller()
= default;
