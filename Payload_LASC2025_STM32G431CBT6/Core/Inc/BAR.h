/*
 * BAR.h
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Driver para barômetro ICP-10100
 */

#ifndef INC_BAR_H_
#define INC_BAR_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Defines */
#define ICP10100_ADDR           (0x63 << 1)
#define MEAS_CMD_LP_P_FIRST     0x401A
#define SEA_LEVEL_PRESSURE      101325.0f

/* Modo de operação */
typedef enum {
    BAR_MODE_LOW_POWER = 0x401A,      // 1.8ms, 3.2 Pa
    BAR_MODE_NORMAL = 0x48A3,          // 6.3ms, 1.6 Pa
    BAR_MODE_LOW_NOISE = 0x5059,       // 24ms, 0.8 Pa
    BAR_MODE_ULTRA_LOW_NOISE = 0x58E0  // 94ms, 0.4 Pa
} BAR_Mode;

/* Status do barômetro */
typedef enum {
    BAR_STATUS_NOT_INIT = 0,
    BAR_STATUS_READY,
    BAR_STATUS_CALIBRATED,
    BAR_STATUS_ERROR
} BAR_Status;

typedef struct {
    /* Handler I2C */
    I2C_HandleTypeDef *hi2c;

    /* Medição */
    BAR_Mode measure_mode;
    uint8_t conversion_delay_ms;

    /* Status */
    BAR_Status status;
    uint8_t is_calibrated;
    uint8_t is_valid;
    uint32_t last_update;

    /* Dados brutos */
    uint32_t pressure_raw;
    uint16_t temperature_raw;

    /* Coeficientes de calibração OTP */
    int16_t otp[4];

    /* Cpmpensação */
    float temperature_ref;     // Temperatura de referência (calibração)
    float pressure_ref;        // Pressão na temp. de referência
    float tco_compensation;    // Fator TCO (±0.5 Pa/°C)

    /* Constantes de conversão */
    float conversion_A;
    float conversion_B;
    float conversion_C;

    /* Dados processados */
    float pressure_pa;          /* Pressão em Pascal */
    float pressure_hpa;         /* Pressão em hPa (mbar) */
    float temperature_c;        /* Temperatura em °C */
    float altitude_m;           /* Altitude em metros */

    /* Referência de pressão ao nível do mar */
    float sea_level_pressure;   /* Pressão de referência (default: 101325 Pa) */

    /* Posição inicial (após primeira leitura válida) */
    uint8_t has_initial_alt;
    float altitude_inicial;
    float pressure_inicial;

    /* Deslocamento vertical */
    float deslocamento_z;       /* Deslocamento vertical em metros */

} BAR;

/* Funções principais */
void BAR_Init(BAR *bar);
HAL_StatusTypeDef BAR_Begin(BAR *bar, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BAR_Update(BAR *bar);

/* Funções de calibração */
HAL_StatusTypeDef BAR_ReadCalibration(BAR *bar);
void BAR_CalculateConversionConstants(BAR *bar);

/* Funções de leitura */
HAL_StatusTypeDef BAR_ReadRaw(BAR *bar);
void BAR_ConvertData(BAR *bar);

/* Funções de configuração */
void BAR_SetSeaLevelPressure(BAR *bar, float pressure_pa);
void BAR_SetSeaLevelPressureFromAltitude(BAR *bar, float known_altitude_m);
void BAR_SaveInitialAltitude(BAR *bar);
void BAR_CalculateDisplacement(BAR *bar);

/* Funções auxiliares */
float BAR_ConvertTemperature(uint16_t t_raw);
float BAR_CalculateAltitude(float pressure_pa, float sea_level_pa);
float BAR_ConvertPressureWithCalibration(BAR *bar);

/* Função de impressão */
void BAR_PrintInfo(BAR *bar);

#endif /* INC_BAR_H_ */
