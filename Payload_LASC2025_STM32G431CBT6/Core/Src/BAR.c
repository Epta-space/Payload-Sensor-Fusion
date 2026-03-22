/*
 * BAR.c
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Driver para barômetro ICP-10100
 */

#include "BAR.h"

/* ========================================================================
 * FUNÇÕES DE INICIALIZAÇÃO
 * ======================================================================== */

void BAR_Init(BAR *bar)
{
    memset(bar, 0, sizeof(BAR));
    bar->status = BAR_STATUS_NOT_INIT;
    bar->is_calibrated = 0;
    bar->is_valid = 0;
    bar->has_initial_alt = 0;
    bar->sea_level_pressure = SEA_LEVEL_PRESSURE;
}

HAL_StatusTypeDef BAR_Begin(BAR *bar, I2C_HandleTypeDef *hi2c)
{
    if (bar == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    bar->hi2c = hi2c;
    bar->status = BAR_STATUS_READY;

    /* Lê os coeficientes de calibração */
    if (BAR_ReadCalibration(bar) != HAL_OK) {
        bar->status = BAR_STATUS_ERROR;
        return HAL_ERROR;
    }

    /* Calcula as constantes de conversão */
    BAR_CalculateConversionConstants(bar);

    bar->status = BAR_STATUS_CALIBRATED;
    bar->is_calibrated = 1;

    return HAL_OK;
}

HAL_StatusTypeDef BAR_Update(BAR *bar)
{
    if (bar->status != BAR_STATUS_CALIBRATED) {
        return HAL_ERROR;
    }

    /* Lê dados brutos */
    if (BAR_ReadRaw(bar) != HAL_OK) {
        bar->is_valid = 0;
        return HAL_ERROR;
    }

    /* Converte os dados */
    BAR_ConvertData(bar);

    bar->is_valid = 1;
    bar->last_update = HAL_GetTick();

    /* Salva altitude inicial na primeira leitura válida */
    if (!bar->has_initial_alt) {
        BAR_SaveInitialAltitude(bar);
    } else {
        BAR_CalculateDisplacement(bar);
    }

    return HAL_OK;
}

/* ========================================================================
 * FUNÇÕES DE CALIBRAÇÃO
 * ======================================================================== */

static HAL_StatusTypeDef send_otp_read_command(BAR *bar)
{
    uint8_t cmd[5] = { 0xC5, 0x95, 0x00, 0x66, 0x9C };
    return HAL_I2C_Master_Transmit(bar->hi2c, ICP10100_ADDR, cmd, 5, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BAR_ReadCalibration(BAR *bar)
{
    HAL_StatusTypeDef ret;
    uint8_t cmd[2] = { 0xC7, 0xF7 };
    uint8_t rx[3];

    ret = send_otp_read_command(bar);
    if (ret != HAL_OK) return ret;

    for (int i = 0; i < 4; i++) {
        ret = HAL_I2C_Master_Transmit(bar->hi2c, ICP10100_ADDR, cmd, 2, HAL_MAX_DELAY);
        if (ret != HAL_OK) return ret;

        ret = HAL_I2C_Master_Receive(bar->hi2c, ICP10100_ADDR, rx, 3, HAL_MAX_DELAY);
        if (ret != HAL_OK) return ret;

        bar->otp[i] = ((int16_t)rx[0] << 8) | rx[1];
    }

    return HAL_OK;
}

static void calculate_conversion_constants(float p_Pa[3], float p_LUT[3], float *A, float *B, float *C)
{
    *C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
          p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
          p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
         (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
          p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
          p_LUT[1] * (p_Pa[2] - p_Pa[0]));

    *A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * (*C)) / (p_LUT[0] - p_LUT[1]);

    *B = (p_Pa[0] - (*A)) * (p_LUT[0] + (*C));
}

void BAR_CalculateConversionConstants(BAR *bar)
{
    /* Constantes de calibração do datasheet */
    float p_Pa_calib[3] = {45000.0f, 80000.0f, 105000.0f};

    /* Fatores de cálculo do datasheet */
    float LUT_lower = 3.5f * (1 << 20);
    float LUT_upper = 11.5f * (1 << 20);
    float quadr_factor = 1.0f / 16777216.0f;
    float offst_factor = 2048.0f;

    /* Calcula valores intermediários no ponto médio de temperatura */
    float t = 0.0f; /* t_raw = 32768 corresponde a t = 0 */
    float s1 = LUT_lower + (float)(bar->otp[0] * t * t) * quadr_factor;
    float s2 = offst_factor * bar->otp[3] + (float)(bar->otp[1] * t * t) * quadr_factor;
    float s3 = LUT_upper + (float)(bar->otp[2] * t * t) * quadr_factor;

    float p_LUT[3] = {s1, s2, s3};

    /* Calcula as constantes A, B, C */
    calculate_conversion_constants(p_Pa_calib, p_LUT,
                                   &bar->conversion_A,
                                   &bar->conversion_B,
                                   &bar->conversion_C);
}

/* ========================================================================
 * FUNÇÕES DE LEITURA
 * ======================================================================== */

//HAL_StatusTypeDef BAR_ReadRaw(BAR *bar)
//{
//    uint8_t cmd[2] = {MEAS_CMD_LP_P_FIRST >> 8, MEAS_CMD_LP_P_FIRST & 0xFF};
//
//    if (HAL_I2C_Master_Transmit(bar->hi2c, ICP10100_ADDR, cmd, 2, HAL_MAX_DELAY) != HAL_OK) {
//        return HAL_ERROR;
//    }
//
//    HAL_Delay(2); /* Tempo de conversão */
//
//    uint8_t rx[9];
//    if (HAL_I2C_Master_Receive(bar->hi2c, ICP10100_ADDR, rx, 9, HAL_MAX_DELAY) != HAL_OK) {
//        return HAL_ERROR;
//    }
//
//    bar->pressure_raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
//    bar->temperature_raw = ((uint16_t)rx[6] << 8) | rx[7];
//
//    return HAL_OK;
//}

HAL_StatusTypeDef BAR_ReadRaw(BAR *bar)
{
    // Comando adaptativo baseado na fase
    uint16_t cmd = BAR_MODE_LOW_NOISE; // padrão

    // Determinar modo pela fase (se tiver acesso ao FSM)
    // Durante apogeu: Ultra Low Noise
    // Durante boost: Normal (barômetro não confiável mesmo)
    // Resto: Low Noise

    uint8_t cmd_bytes[2] = {cmd >> 8, cmd & 0xFF};

    if (HAL_I2C_Master_Transmit(bar->hi2c, ICP10100_ADDR, cmd_bytes, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // IMPORTANTE: Delay depende do modo!
    HAL_Delay(25); // Para Low Noise mode (24ms típico)

    uint8_t rx[9];
    if (HAL_I2C_Master_Receive(bar->hi2c, ICP10100_ADDR, rx, 9, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    bar->pressure_raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    bar->temperature_raw = ((uint16_t)rx[6] << 8) | rx[7];

    return HAL_OK;
}

//void BAR_ConvertData(BAR *bar)
//{
//    /* Converte temperatura */
//    bar->temperature_c = BAR_ConvertTemperature(bar->temperature_raw);
//
//    /* Converte pressão usando calibração */
//    bar->pressure_pa = BAR_ConvertPressureWithCalibration(bar);
//    bar->pressure_hpa = bar->pressure_pa / 100.0f;
//
//    /* Calcula altitude */
//    bar->altitude_m = BAR_CalculateAltitude(bar->pressure_pa, bar->sea_level_pressure);
//}

void BAR_ConvertData(BAR *bar)
{
    // Conversão existente
    bar->temperature_c = BAR_ConvertTemperature(bar->temperature_raw);
    bar->pressure_pa = BAR_ConvertPressureWithCalibration(bar);

    // NOVO: Compensação TCO
    if (bar->has_initial_alt) {
        float temp_delta = bar->temperature_c - bar->temperature_ref;
        float tco_correction = temp_delta * bar->tco_compensation;

        // Aplica correção
        bar->pressure_pa -= tco_correction;
    }

    bar->pressure_hpa = bar->pressure_pa / 100.0f;
    bar->altitude_m = BAR_CalculateAltitude(bar->pressure_pa, bar->sea_level_pressure);
}

/* ========================================================================
 * FUNÇÕES DE CONVERSÃO
 * ======================================================================== */

float BAR_ConvertTemperature(uint16_t t_raw)
{
    return -45.0f + (175.0f / 65536.0f) * (float)t_raw;
}

float BAR_CalculateAltitude(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

float BAR_ConvertPressureWithCalibration(BAR *bar)
{
    /* Fatores de cálculo do datasheet */
    float LUT_lower = 3.5f * (1 << 20);
    float LUT_upper = 11.5f * (1 << 20);
    float quadr_factor = 1.0f / 16777216.0f;
    float offst_factor = 2048.0f;

    /* Calcula os valores intermediários 's' dependentes da temperatura */
    float t = (float)bar->temperature_raw - 32768.0f;
    float s1 = LUT_lower + (float)(bar->otp[0] * t * t) * quadr_factor;
    float s2 = offst_factor * bar->otp[3] + (float)(bar->otp[1] * t * t) * quadr_factor;
    float s3 = LUT_upper + (float)(bar->otp[2] * t * t) * quadr_factor;

    /* Calcula constantes A, B, C para esta temperatura específica */
    float p_Pa_calib[3] = {45000.0f, 80000.0f, 105000.0f};
    float p_LUT[3] = {s1, s2, s3};
    float A, B, C;

    calculate_conversion_constants(p_Pa_calib, p_LUT, &A, &B, &C);

    /* Calcula a pressão final usando a fórmula do datasheet */
    return A + B / (C + (float)bar->pressure_raw);
}

/* ========================================================================
 * FUNÇÕES DE CONFIGURAÇÃO E DESLOCAMENTO
 * ======================================================================== */

void BAR_SetSeaLevelPressure(BAR *bar, float pressure_pa)
{
    bar->sea_level_pressure = pressure_pa;
}

void BAR_SetSeaLevelPressureFromAltitude(BAR *bar, float known_altitude_m)
{
    /* Fórmula inversa da altitude barométrica */
    bar->sea_level_pressure = bar->pressure_pa / powf(1.0f - (known_altitude_m / 44330.0f), 5.255f);
}

void BAR_CalculateDisplacement(BAR *bar)
{
    if (!bar->has_initial_alt || !bar->is_valid) return;

    bar->deslocamento_z = bar->altitude_m - bar->altitude_inicial;
}

void BAR_SaveInitialAltitude(BAR *bar)
{
    if (!bar->is_valid) return;

    bar->altitude_inicial = bar->altitude_m;
    bar->pressure_inicial = bar->pressure_pa;
    bar->temperature_ref = bar->temperature_c;  // NOVO
    bar->tco_compensation = 0.5f;  // ±0.5 Pa/°C do datasheet
    bar->has_initial_alt = 1;
    bar->deslocamento_z = 0.0f;
}

/* ========================================================================
 * FUNÇÃO DE IMPRESSÃO
 * ======================================================================== */

void BAR_PrintInfo(BAR *bar)
{
    printf("\n========== INFORMACOES DO BAROMETRO ==========\n");

    printf("\n--- STATUS ---\n");
    printf("Status: ");
    switch(bar->status) {
        case BAR_STATUS_NOT_INIT:   printf("Nao Inicializado\n"); break;
        case BAR_STATUS_READY:      printf("Pronto\n"); break;
        case BAR_STATUS_CALIBRATED: printf("Calibrado\n"); break;
        case BAR_STATUS_ERROR:      printf("Erro\n"); break;
        default:                    printf("Desconhecido\n"); break;
    }

    printf("Calibrado: %s\n", bar->is_calibrated ? "Sim" : "Nao");
    printf("Dados Validos: %s\n", bar->is_valid ? "Sim" : "Nao");
    printf("Ultima Atualizacao: %lu ms\n", bar->last_update);

    if (bar->is_calibrated) {
        printf("\n--- COEFICIENTES OTP ---\n");
        printf("OTP[0]: %d\n", bar->otp[0]);
        printf("OTP[1]: %d\n", bar->otp[1]);
        printf("OTP[2]: %d\n", bar->otp[2]);
        printf("OTP[3]: %d\n", bar->otp[3]);

        printf("\n--- CONSTANTES DE CONVERSAO ---\n");
        printf("A: %.6f\n", bar->conversion_A);
        printf("B: %.6f\n", bar->conversion_B);
        printf("C: %.6f\n", bar->conversion_C);
    }

    if (bar->is_valid) {
        printf("\n--- DADOS BRUTOS ---\n");
        printf("Pressao Raw: %lu\n", bar->pressure_raw);
        printf("Temperatura Raw: %u\n", bar->temperature_raw);

        printf("\n--- DADOS PROCESSADOS ---\n");
        printf("Temperatura: %.2f C\n", bar->temperature_c);
        printf("Pressao: %.2f Pa (%.2f hPa)\n", bar->pressure_pa, bar->pressure_hpa);
        printf("Altitude: %.2f m\n", bar->altitude_m);
        printf("Pressao ao Nivel do Mar (ref): %.2f Pa\n", bar->sea_level_pressure);

        if (bar->has_initial_alt) {
            printf("\n--- ALTITUDE INICIAL ---\n");
            printf("Altitude inicial: %.2f m\n", bar->altitude_inicial);
            printf("Pressao inicial: %.2f Pa\n", bar->pressure_inicial);

            printf("\n--- DESLOCAMENTO ---\n");
            printf("Deslocamento Z (Vertical): %.2f m %s\n",
                   fabs(bar->deslocamento_z),
                   bar->deslocamento_z >= 0 ? "(Subiu)" : "(Desceu)");
        } else {
            printf("\n--- ALTITUDE INICIAL ---\n");
            printf("Aguardando primeira leitura valida...\n");
        }
    }

    printf("\n==============================================\n\n");
}
