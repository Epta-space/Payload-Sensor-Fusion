/**
  ******************************************************************************
  * @file    icp10100.c
  * @brief   ICP-10100 barometric pressure sensor driver
  ******************************************************************************
  */

#include "icp10100.h"
#include "main.h"
#include <math.h> // para powf

/* I2C handle externo (definido no i2c.c gerado pelo STM32CubeIDE) */
extern I2C_HandleTypeDef hi2c2;

/* Comando de leitura OTP */
static HAL_StatusTypeDef send_otp_read_command(void) {
    uint8_t cmd[5] = { 0xC5, 0x95, 0x00, 0x66, 0x9C };
    return HAL_I2C_Master_Transmit(&hi2c2, ICP10100_ADDR, cmd, 5, HAL_MAX_DELAY);
}

/**
  * @brief Lê dados brutos de pressão e temperatura
  */
HAL_StatusTypeDef ICP10100_ReadRaw(uint32_t *p_raw, uint16_t *t_raw) {
    uint8_t cmd[2] = {MEAS_CMD_LP_P_FIRST >> 8, MEAS_CMD_LP_P_FIRST & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c2, ICP10100_ADDR, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(2); // tempo de conversão

    uint8_t rx[9];
    HAL_I2C_Master_Receive(&hi2c2, ICP10100_ADDR, rx, 9, HAL_MAX_DELAY);

    *p_raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    *t_raw = ((uint16_t)rx[6] << 8) | rx[7];
    return HAL_OK;
}

/**
  * @brief Converte temperatura bruta em graus Celsius
  */
float ICP10100_ConvertTemperature(uint16_t t_raw) {
    return -45.0f + (175.0f / 65536.0f) * (float)t_raw;
}

/**
  * @brief Calcula a altitude em metros com base na pressão
  */
float ICP10100_CalculateAltitude(float pressure_pa) {
    return 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE, 0.1903f));
}

/**
  * @brief Lê os 4 coeficientes da memória OTP
  */
HAL_StatusTypeDef ICP10100_ReadCalibration(int16_t otp[4]) {
    HAL_StatusTypeDef ret;
    uint8_t cmd[2] = { 0xC7, 0xF7 };
    uint8_t rx[3];

    ret = send_otp_read_command();
    if (ret != HAL_OK) return ret;

    for (int i = 0; i < 4; i++) {
        ret = HAL_I2C_Master_Transmit(&hi2c2, ICP10100_ADDR, cmd, 2, HAL_MAX_DELAY);
        if (ret != HAL_OK) return ret;

        ret = HAL_I2C_Master_Receive(&hi2c2, ICP10100_ADDR, rx, 3, HAL_MAX_DELAY);
        if (ret != HAL_OK) return ret;

        otp[i] = ((int16_t)rx[0] << 8) | rx[1];
    }
    return HAL_OK;
}

/**
  * @brief  Calcula os constantes de conversão A, B e C.
  * @note   Esta função é uma implementação direta do código de exemplo do datasheet.
  * @param  p_Pa: Vetor com 3 pontos de pressão de referência em Pascal.
  * @param  p_LUT: Vetor com 3 valores de sensor correspondentes aos pontos de pressão.
  * @param  out: Vetor de saída para armazenar [A, B, C].
  */
static void calculate_conversion_constants(float p_Pa[3], float p_LUT[3], float out[3])
{
    float A, B, C;

    C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
         p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
         p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
        (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
         p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
         p_LUT[1] * (p_Pa[2] - p_Pa[0]));

    A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);

    B = (p_Pa[0] - A) * (p_LUT[0] + C);

    out[0] = A;
    out[1] = B;
    out[2] = C;
}

/**
  * @brief Converte a pressão bruta usando coeficientes OTP (VERSÃO CORRIGIDA)
  */
float ICP10100_ConvertPressureWithCalibration(uint32_t p_raw, uint16_t t_raw, int16_t otp[4]) {
    // Constantes de calibração do datasheet [cite: 527, 528, 529]
    float p_Pa_calib[3] = {45000.0f, 80000.0f, 105000.0f};

    // Fatores de cálculo do datasheet [cite: 530, 531, 532]
    float LUT_lower = 3.5f * (1 << 20);      // 3.5 * 2^20
    float LUT_upper = 11.5f * (1 << 20);     // 11.5 * 2^20
    float quadr_factor = 1.0f / 16777216.0f; // 1 / 2^24
    float offst_factor = 2048.0f;

    // 1. Calcula os valores intermediários 's' dependentes da temperatura
    float t = (float)t_raw - 32768.0f;
    float s1 = LUT_lower + (float)(otp[0] * t * t) * quadr_factor;
    float s2 = offst_factor * otp[3] + (float)(otp[1] * t * t) * quadr_factor;
    float s3 = LUT_upper + (float)(otp[2] * t * t) * quadr_factor;

    // 2. Calcula os coeficientes A, B, C usando a função auxiliar correta
    float p_LUT[3] = {s1, s2, s3};
    float out_ABC[3];
    calculate_conversion_constants(p_Pa_calib, p_LUT, out_ABC);

    float A = out_ABC[0];
    float B = out_ABC[1];
    float C = out_ABC[2];

    // 3. Calcula a pressão final usando a fórmula do datasheet [cite: 555]
    return A + B / (C + (float)p_raw);
}
