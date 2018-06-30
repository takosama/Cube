#pragma once
#include "IReadPin.h"
class Controller
{
	bool _isInited = false;
	int _PinCount = 0;
	IReadPin** _ReadPinArray;
public:
	Controller();
	Controller(int PinCount);
	void SetPin(IReadPin* ReadPin,int ID);
	bool ReadStatus(int ID);
	~Controller();
};

