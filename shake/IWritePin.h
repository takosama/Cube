#pragma once
class IWritePin
{
public:
	virtual ~IWritePin()=default;
	virtual void Write(bool Value)=0;
	virtual void Write(int Value) = 0;
	virtual void Write(float Value) = 0;
};

