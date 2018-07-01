#include "ControllerManager.h"
#include "DigitalReadPin.h"


ControllerManager::ControllerManager()
= default;

Controller* ControllerManager::CreateController()
{
	auto* rtn = new Controller(ControlerPins::ControlerPins_Count);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_IsServo]), ControlerPins::ControlerPins_IsServo);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Speed1]), ControlerPins::ControlerPins_Speed1);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Speed2]), ControlerPins::ControlerPins_Speed2);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Speed4]), ControlerPins::ControlerPins_Speed4);

	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Up]), ControlerPins::ControlerPins_Up);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Down]), ControlerPins::ControlerPins_Down);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Left]), ControlerPins::ControlerPins_Left);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Right]), ControlerPins::ControlerPins_Right);

	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Hi]), ControlerPins::ControlerPins_Hi);
	rtn->SetPin(new DigitalReadPin(&ControlerPinsArray[ControlerPins::ControlerPins_Lo]), ControlerPins::ControlerPins_Lo);
	return rtn;
}


ControllerManager::~ControllerManager()
= default;
