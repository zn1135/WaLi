// #include "VBatSense.h"
//
// /* 12bit ADC 满量程计数 */
// #define ADC_MAX_COUNT_12BIT 4095U
// /* 默认参数：3.3V参考、电阻分压10k/1k */
// #define VBAT_DEFAULT_VREF_MV 3300U
// #define VBAT_DEFAULT_R_TOP_OHM 10000U
// #define VBAT_DEFAULT_R_BOTTOM_OHM 1000U
//
// /* 模块内部保存的ADC句柄与参数 */
// static ADC_HandleTypeDef *s_hadc = 0;
// static VBatSense_Config_t s_cfg = {
//     VBAT_DEFAULT_VREF_MV,
//     VBAT_DEFAULT_R_TOP_OHM,
//     VBAT_DEFAULT_R_BOTTOM_OHM
// };
// /* 仅在首次读取前做一次校准 */
// static uint8_t s_is_calibrated = 0U;
//
// static uint32_t scale_to_mv(uint16_t raw)
// {
//     /* 原始计数换算为ADC引脚电压(mV) */
//     return ((uint32_t)raw * s_cfg.vref_mv) / ADC_MAX_COUNT_12BIT;
// }
//
// void VBatSense_Init(ADC_HandleTypeDef *hadc, const VBatSense_Config_t *config)
// {
//     s_hadc = hadc;
//     s_is_calibrated = 0U;
//
//     if (config != 0) {
//         s_cfg = *config;
//     }
//
//     /* 对非法配置做兜底，防止后续计算异常 */
//     if (s_cfg.vref_mv == 0U) {
//         s_cfg.vref_mv = VBAT_DEFAULT_VREF_MV;
//     }
//     if (s_cfg.r_top_ohm == 0U) {
//         s_cfg.r_top_ohm = VBAT_DEFAULT_R_TOP_OHM;
//     }
//     if (s_cfg.r_bottom_ohm == 0U) {
//         s_cfg.r_bottom_ohm = VBAT_DEFAULT_R_BOTTOM_OHM;
//     }
// }
//
// HAL_StatusTypeDef VBatSense_Calibrate(void)
// {
//     if (s_hadc == 0) {
//         return HAL_ERROR;
//     }
//
//     if (s_is_calibrated != 0U) {
//         return HAL_OK;
//     }
//
//     /* G4单端模式校准，提升绝对精度 */
//     if (HAL_ADCEx_Calibration_Start(s_hadc, ADC_SINGLE_ENDED) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     s_is_calibrated = 1U;
//     return HAL_OK;
// }
//
// HAL_StatusTypeDef VBatSense_ReadRaw(uint16_t *raw)
// {
//     if ((s_hadc == 0) || (raw == 0)) {
//         return HAL_ERROR;
//     }
//
//     if (VBatSense_Calibrate() != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     /* 单次采样流程：启动 -> 等待转换完成 -> 读值 -> 停止 */
//     if (HAL_ADC_Start(s_hadc) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     if (HAL_ADC_PollForConversion(s_hadc, 10U) != HAL_OK) {
//         (void)HAL_ADC_Stop(s_hadc);
//         return HAL_ERROR;
//     }
//
//     *raw = (uint16_t)HAL_ADC_GetValue(s_hadc);
//
//     if (HAL_ADC_Stop(s_hadc) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     return HAL_OK;
// }
//
// HAL_StatusTypeDef VBatSense_ReadAdcMilliVolt(uint32_t *adc_mv)
// {
//     uint16_t raw = 0U;
//
//     if (adc_mv == 0) {
//         return HAL_ERROR;
//     }
//
//     if (VBatSense_ReadRaw(&raw) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     *adc_mv = scale_to_mv(raw);
//     return HAL_OK;
// }
//
// HAL_StatusTypeDef VBatSense_ReadBatteryMilliVolt(uint32_t *vbat_mv)
// {
//     uint16_t raw = 0U;
//
//     if ((vbat_mv == 0) || (s_cfg.r_bottom_ohm == 0U)) {
//         return HAL_ERROR;
//     }
//
//     if (VBatSense_ReadRaw(&raw) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     /* 将ADC引脚电压按分压比例反推母线电压 */
//     *vbat_mv = VBatSense_RawToBatteryMilliVolt(raw);
//     return HAL_OK;
// }
//
// uint32_t VBatSense_RawToAdcMilliVolt(uint16_t raw)
// {
//     return scale_to_mv(raw);
// }
//
// uint32_t VBatSense_RawToBatteryMilliVolt(uint16_t raw)
// {
//     uint32_t adc_mv = scale_to_mv(raw);
//
//     /* 防止分母为0 */
//     if (s_cfg.r_bottom_ohm == 0U) {
//         return 0U;
//     }
//
//     /* Vbat = Vadc * (R_top + R_bottom) / R_bottom */
//     return (adc_mv * (s_cfg.r_top_ohm + s_cfg.r_bottom_ohm)) / s_cfg.r_bottom_ohm;
// }
//
