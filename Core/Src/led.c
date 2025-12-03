#include "led.h"

// buffer: 24 bits per LED
static uint32_t ws2812_buffer[LED_COUNT * 24];


extern TIM_HandleTypeDef htim3;  // TIM4 handle
#define WS2812_DMA_CHANNEL TIM_CHANNEL_2  // Channel 1

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    }
}

void WS2812_SetLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index >= LED_COUNT) return;

    uint8_t colors[3] = {g, r, b}; // WS2812 expects GRB

    for(uint8_t c = 0; c < 3; c++)
    {
        for(int bit = 0; bit < 8; bit++)
        {
        	if(colors[c] & (1 << (7-bit)))
        	    ws2812_buffer[index*24 + c*8 + bit] = BIT_1_HIGH;
        	else
        	    ws2812_buffer[index*24 + c*8 + bit] = BIT_0_HIGH;
        }
    }
}

void WS2812_Send(void)
{
    // Start DMA, use proper type
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t*)ws2812_buffer, LED_COUNT*24);


    HAL_Delay(1); // >50us reset
}

void ClearLED(void){
	for(int i = 0; i < LED_COUNT; i++){
		WS2812_SetLED(i, 0, 0, 0);
	}
	WS2812_Send();
}

