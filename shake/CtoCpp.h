#pragma once

#include "stm32f4xx_hal.h"


#ifdef _cpulsplus
extern "C" {
#endif // _cpulsplus

	typedef enum _ControlerPins
	{
		ControlerPins_Hi,
		ControlerPins_Lo,
		ControlerPins_Left,
		ControlerPins_Right,
		ControlerPins_Up,
		ControlerPins_Down,
		ControlerPins_IsServo,  
		ControlerPins_Speed1, 
		ControlerPins_Speed2, 
		ControlerPins_Speed4,
		ControlerPins_Count,
	}ControlerPins;

	typedef struct _CtpCppGPIO
	{
		GPIO_TypeDef* pGPIO;
		uint16_t Pin;
	}CtoCppGPIO;
      
	extern CtoCppGPIO ControlerPinsArray[ControlerPins_Count];

	CtoCppGPIO CtoCppGPIOInit(GPIO_TypeDef* pGPIO, uint16_t Pin);
	
	
	#ifdef _cpulsplus
}
#endif // _cpulsplus
