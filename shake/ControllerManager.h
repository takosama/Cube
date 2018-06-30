#pragma once
#include "Controller.h"

class ControllerManager
{
public:
	ControllerManager();
	Controller* CreateController();
	~ControllerManager();
};

