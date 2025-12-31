#ifndef INC_KNOB_H_
#define INC_KNOB_H_

#include "main.h"
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
volatile uint32_t brightness = 0;
volatile uint32_t adc_val[1];

void knob_init(void);
void brightness_update(void);

void knob_init(void) {
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_val, 1);
	HAL_TIM_Base_Start(&htim3);
}

void brightness_update(void){
	brightness = (adc_val[0] * 31) / 4095;
}

#endif /* INC_KNOB_H_ */
