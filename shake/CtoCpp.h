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

	typedef enum _MoterDirPins
	{
		MoterDirPins_Dir1,
		MoterDirPins_Dir2,
		MoterDirPins_Dir3,
		MoterDirPins_Count
	}MoterDirPins;

	typedef enum _MoterPwmPins
	{
		MoterPwmPins_Pwm1,
		MoterPwmPins_Pwm2,
		MoterPwmPins_Pwm3,
		MoterPwmPins_Count
	}MoterPwmPins;
	
	typedef struct _CtpCppGPIO
	{
		GPIO_TypeDef* pGPIO;
		uint16_t Pin;
	}CtoCppGPIO;

	typedef struct _CtpCppPwmGPIO
	{
	TIM_HandleTypeDef* htim;
		int Channel;
	}CtoCppPwmGPIO;
      
	extern CtoCppGPIO ControlerPinsArray[ControlerPins_Count];
	extern CtoCppGPIO MoterDirPinsArray[MoterDirPins_Count];
	extern CtoCppPwmGPIO MoterPwmPinsArray[MoterPwmPins_Count];
	
	CtoCppGPIO CtoCppGPIOInit(GPIO_TypeDef* pGPIO, uint16_t Pin);
	CtoCppPwmGPIO CtoCppPwmGPIOInit(TIM_HandleTypeDef* htim, int Channel);
	
	
	#ifdef _cpulsplus
}
#endif // _cpulsplus
