#pragma once
#include "CppMain.h"

class IReadPin
{
public:
	virtual ~IReadPin() = default;
	virtual  bool ReadBool()=0;
	virtual  int ReadInt()=0;
	virtual float ReadFloat()=0;
};

