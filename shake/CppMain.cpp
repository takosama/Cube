#include "CppMain.h"
#include "DigitalReadPin.h"
#include <cstdio>
#include "Controller.h"
#include "ControllerManager.h"

Controller* controller;
void CppSetUp()
{
	ControllerManager manager;
	controller = manager.CreateController();
}

void CppLoop()
{


}
