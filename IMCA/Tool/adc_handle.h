#pragma once
#include "main.h"
#include "adc.h"

#define HADC1 hadc3
// #define HADC2 hadc2

#define ADC1_VALUE_SIZE 1
// #define ADC2_VALUE_SIZE 3

extern uint16_t adc1_value[ADC1_VALUE_SIZE];
// extern uint16_t adc2_value[ADC2_VALUE_SIZE];

typedef struct
{
/***User_Core_Begin***/
	float Current;
	float Voltage;
	float Temper_mos;
	float Temper_Ls;
	float Temper_Cps;
	float Temper_MAX;
/***User_Core_End***/
} User_ADC_out_data_t;

extern User_ADC_out_data_t User_ADC_out_data;

void ADC_handle_Init(void);

void ADC_soft_start(void);


void NTC_B_Rtemp(float* Temp_out, uint16_t* R_out, uint16_t pull_R, uint16_t B_value, uint16_t adc_value, float Voltage, uint16_t R_ref);

void ADC_To_Voltage(float* voltage_out, uint16_t adc_value, float gain, float voltage_division);

void ADC_To_Current(float* current_out, uint16_t adc_value, uint8_t gain, float Rs);

void ADC_handle_task(User_ADC_out_data_t* adc_out_data);
