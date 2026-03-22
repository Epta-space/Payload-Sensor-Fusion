/**
  ******************************************************************************
  * @file    icp10100.h
  * @brief   Header for ICP-10100 barometric pressure sensor driver
  ******************************************************************************
  * @author
  * @date
  ******************************************************************************
  */

#ifndef __ICP10100_H__
#define __ICP10100_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"   // Inclui HAL e configurações globais

/* Defines -------------------------------------------------------------------*/
#define ICP10100_ADDR         (0x63 << 1)     // Endereço I2C do ICP-10100 com R/W = 0
#define MEAS_CMD_LP_P_FIRST   0x401A          // Comando de medição Low Power
#define SEA_LEVEL_PRESSURE    101325.0f       // Pressão ao nível do mar em Pascal

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief  Lê os dados brutos do ICP-10100
 * @param  p_raw: ponteiro para armazenar valor de pressão bruto
 * @param  t_raw: ponteiro para armazenar valor de temperatura bruto
 * @retval HAL_OK se sucesso, HAL_ERROR caso contrário
 */
HAL_StatusTypeDef ICP10100_ReadRaw(uint32_t *p_raw, uint16_t *t_raw);

/**
 * @brief  Converte o valor bruto de temperatura em graus Celsius
 * @param  t_raw: valor bruto de temperatura
 * @retval Temperatura em graus Celsius (°C)
 */
float ICP10100_ConvertTemperature(uint16_t t_raw);

/**
 * @brief  Calcula a altitude com base na pressão medida
 * @param  pressure_pa: pressão atmosférica atual em Pascal
 * @retval Altitude aproximada em metros
 */
float ICP10100_CalculateAltitude(float pressure_pa);

/**
 * @brief  Lê os coeficientes de calibração do sensor (memória OTP)
 * @param  otp: vetor de 4 posições para armazenar os coeficientes
 * @retval HAL_OK se sucesso, HAL_ERROR caso contrário
 */
HAL_StatusTypeDef ICP10100_ReadCalibration(int16_t otp[4]);

/**
 * @brief  Converte a pressão com base nos dados brutos e nos coeficientes OTP
 * @param  p_raw: valor bruto de pressão
 * @param  t_raw: valor bruto de temperatura
 * @param  otp: vetor com os coeficientes de calibração
 * @retval Pressão em Pascal (Pa)
 */
float ICP10100_ConvertPressureWithCalibration(uint32_t p_raw, uint16_t t_raw, int16_t otp[4]);

#endif /* __ICP10100_H__ */
