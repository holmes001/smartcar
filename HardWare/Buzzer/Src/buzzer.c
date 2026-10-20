#include "buzzer.h"

#include "stdint.h"
#include "tim.h"

extern TIM_HandleTypeDef htim14;
#define buzzer htim14

void buzzer_init(void)
{
    HAL_TIM_PWM_Start(&buzzer, TIM_CHANNEL_1);
}

void buzzer_beep(uint8_t volume, uint8_t duration_ms)
{
    __HAL_TIM_SET_COMPARE(&buzzer, TIM_CHANNEL_1, volume);

    HAL_Delay(duration_ms); 

    __HAL_TIM_SET_COMPARE(&buzzer, TIM_CHANNEL_1, 0);

    HAL_Delay(duration_ms); 
}