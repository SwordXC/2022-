#include "menu.h"

key key_check()
{
	key key_value;
	if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==RESET)||(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==RESET)
		||(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==RESET)||(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)==RESET)||(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==RESET))
	{
		if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==RESET))
		{
			key_value=key_enter;
			while((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)==RESET));
			return key_value;
		}
		if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==RESET))
		{
			key_value=key_left;
			while((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==RESET));
			return key_value;
		}
		if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==RESET))
		{
			key_value=key_right;
			while((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==RESET));
			return key_value;
		}
		if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)==RESET))
		{
			key_value=key_up;
			while((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)==RESET));
			return key_value;
		}
		if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==RESET))
		{
			key_value=key_down;
			while((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==RESET));
			return key_value;
		}
	}
	return no_key;
}