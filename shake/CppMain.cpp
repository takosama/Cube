#include "CppMain.h"
#include "DigitalReadPin.h"
#include <cstdio>
#include "Controller.h"
#include "ControllerManager.h"
#include "Moter.h"
#include "ServoC.h"

Controller* controller;
Moter* moterLeftRight;
Moter* MoterFrontBack;
Moter* MoterHiLo;
void CppSetUp()
{
	ControllerManager manager;
	controller = manager.CreateController();

	moterLeftRight = new Moter(new DigitalWritePin(&MoterDirPinsArray[MoterDirPins_Dir1]),
	                   new PwmWritePin(&MoterPwmPinsArray[MoterPwmPins_Pwm1]));

	MoterFrontBack= new Moter(new DigitalWritePin(&MoterDirPinsArray[MoterDirPins_Dir2]),
		new PwmWritePin(&MoterPwmPinsArray[MoterPwmPins_Pwm2]));

	MoterHiLo = new Moter(new DigitalWritePin(&MoterDirPinsArray[MoterDirPins_Dir3]),
		new PwmWritePin(&MoterPwmPinsArray[MoterPwmPins_Pwm3]));

}

void CppLoop()
{
	const int speed = controller->ReadStatus(ControlerPins::ControlerPins_Speed1) +
		2 * controller->ReadStatus(ControlerPins::ControlerPins_Speed2) +
		4 * controller->ReadStatus(ControlerPins::ControlerPins_Speed4);

	if (!controller->ReadStatus(ControlerPins::ControlerPins_IsServo))
	{

		printf("%d %d\n", controller->ReadStatus(ControlerPins::ControlerPins_Up),
			controller->ReadStatus(ControlerPins::ControlerPins_Down));

		
		const auto speedf = 1.0f / 7.0f* speed;
	if (!controller->ReadStatus(ControlerPins::ControlerPins_Left) && !controller->ReadStatus(
		ControlerPins::ControlerPins_Right))
		moterLeftRight->Move(0);
	if (controller->ReadStatus(ControlerPins::ControlerPins_Left))
		moterLeftRight->Move(speedf);
	 if (controller->ReadStatus(ControlerPins::ControlerPins_Right))
		moterLeftRight->Move(-speedf);

		 if (!controller->ReadStatus(ControlerPins::ControlerPins_Up) && !controller->ReadStatus(
			 ControlerPins::ControlerPins_Down))
			 MoterFrontBack->Move(0);
		 if (controller->ReadStatus(ControlerPins::ControlerPins_Up))
			 MoterFrontBack->Move(speedf);
		 if (controller->ReadStatus(ControlerPins::ControlerPins_Down))
			 MoterFrontBack->Move(-speedf);
	
	
		 if (!controller->ReadStatus(ControlerPins::ControlerPins_Hi) && !controller->ReadStatus(
			 ControlerPins::ControlerPins_Lo))
			 MoterHiLo->Move(0);
		 if (controller->ReadStatus(ControlerPins::ControlerPins_Hi))
			 MoterHiLo->Move(speedf);
		 if (controller->ReadStatus(ControlerPins::ControlerPins_Lo))
			 MoterHiLo->Move(-speedf);
	}
	else
	{
		MoterHiLo->Move((0));
		MoterFrontBack->Move((0));
		moterLeftRight->Move((0));
		Servo1_Pow = ( 105-87)*(1.0f/7.0f)* speed+87;
	}
}
