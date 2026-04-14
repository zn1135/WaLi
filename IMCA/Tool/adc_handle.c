#include "adc_handle.h"
#include "math.h"
#include "Ramp.h"
#include "adc.h"
uint16_t adc1_value[ADC1_VALUE_SIZE];
// uint16_t adc2_value[ADC2_VALUE_SIZE];

User_ADC_out_data_t User_ADC_out_data;
one_kalman_filter_init_t temper_max_filter;

void ADC_handle_Init(void)
{
	// HAL_ADCEx_Calibration_Start(&hadc3);
	// HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	// HAL_Delay(5);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc1_value, ADC1_VALUE_SIZE);
	// HAL_ADC_Start_DMA(&HADC2, (uint32_t*)adc2_value, ADC2_VALUE_SIZE);
}

void ADC_soft_start(void)
{
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc1_value, ADC1_VALUE_SIZE);
}
__weak void ADC_handle_GetValue(void)
{

}

/// @brief NTC计算，电路中采样电压两倍放大，使用3.3v上拉电阻，NTC作为下拉电阻
/// @param Temp_out 温度计算值，单位摄氏度
/// @param R_out NTC阻值
/// @param pull_R 上拉电阻阻值
/// @param B_value NTC特性值
/// @param adc_value ADC采样值，12Bit
/// @param R_ref NTC在25摄氏度下的标准阻值
void NTC_B_Rtemp(float* Temp_out, uint16_t* R_out, uint16_t pull_R, uint16_t B_value, uint16_t adc_value,float Voltage, uint16_t R_ref)
{
	float Vin=Voltage;
	if (Vin==0)
	{
		Vin=adc_value * 1.65f / 4095;
	}
	float ntc_R = pull_R / ((3.3f / (Vin)) - 1);//注意1.65V是由于运放放大
	float ntc_Temp = (1.0f / ((1.0f / B_value) * log(ntc_R / R_ref) + (1.0f / (25 + 273.15f))) - 273.15f);
	if (R_out != NULL)
	{
		*R_out = (uint16_t)ntc_R;
	}
	if (Temp_out != NULL)
	{
		*Temp_out = ntc_Temp;
	}
}
void ADC_To_Voltage(float* voltage_out, uint16_t adc_value, float gain, float voltage_division_Inverse)
{
	float voltage_temp = (adc_value * 3.3f / 4095)/ gain * voltage_division_Inverse;
	if (voltage_out != NULL)
	{
		*voltage_out = voltage_temp;
	}
	
}

void ADC_To_Current(float* current_out, uint16_t adc_value, uint8_t gain, float Rs)
{
	float current_out_temp = (adc_value * 3.3f / 4095) / gain / Rs;
	if ( current_out!= NULL)
	{
		*current_out = current_out_temp;
	}
}

void ADC_handle_task(User_ADC_out_data_t* adc_out_data)
{
	/***User_Core_Begin***/

	/***User_Core_End***/
}