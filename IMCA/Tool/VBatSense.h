// #ifndef IMCA_TOOL_VBATSENSE_H
// #define IMCA_TOOL_VBATSENSE_H
//
// #ifdef __cplusplus
// extern "C" {
// #endif
//
// #include <stdint.h>
// #include "adc.h"
//
// typedef struct {
//     uint32_t vref_mv;
//     uint32_t r_top_ohm;
//     uint32_t r_bottom_ohm;
// } VBatSense_Config_t;
//
// void VBatSense_Init(ADC_HandleTypeDef *hadc, const VBatSense_Config_t *config);
// HAL_StatusTypeDef VBatSense_Calibrate(void);
// HAL_StatusTypeDef VBatSense_ReadRaw(uint16_t *raw);
// HAL_StatusTypeDef VBatSense_ReadAdcMilliVolt(uint32_t *adc_mv);
// HAL_StatusTypeDef VBatSense_ReadBatteryMilliVolt(uint32_t *vbat_mv);
//
// uint32_t VBatSense_RawToAdcMilliVolt(uint16_t raw);
// uint32_t VBatSense_RawToBatteryMilliVolt(uint16_t raw);
//
// #ifdef __cplusplus
// }
// #endif
//
// #endif /* IMCA_TOOL_VBATSENSE_H */
//
