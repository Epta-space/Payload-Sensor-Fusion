/*
 * IMU.c
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Driver para IMU ICM-20948 (Accel + Gyro + Mag)
 */

#include "IMU.h"
#include "ROCKET.h"
#include "IMU_FUSION.h"

/* Registradores ICM-20948 - Bank 0 */
#define ICM20948_WHO_AM_I       0x00
#define ICM20948_USER_CTRL      0x03
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07
#define ICM20948_ACCEL_OUT      0x2D
#define ICM20948_GYRO_OUT       0x33
#define ICM20948_TEMP_OUT       0x39
#define ICM20948_EXT_SLV_DATA_00 0x3B
#define ICM20948_REG_BANK_SEL   0x7F

/* Registradores ICM-20948 - Bank 2 */
#define ICM20948_GYRO_CONFIG_1  0x01
#define ICM20948_ACCEL_CONFIG   0x14

/* Registradores ICM-20948 - Bank 3 */
#define ICM20948_I2C_MST_CTRL   0x01
#define ICM20948_I2C_SLV0_ADDR  0x03
#define ICM20948_I2C_SLV0_REG   0x04
#define ICM20948_I2C_SLV0_CTRL  0x05
#define ICM20948_I2C_SLV0_DO    0x06

/* Magnetômetro AK09916 */
#define AK09916_ADDRESS         0x0C
#define AK09916_WIA2            0x01
#define AK09916_ST1             0x10
#define AK09916_HXL             0x11
#define AK09916_CNTL2           0x31
#define AK09916_CNTL3           0x32

/* Constantes */
#define ICM20948_WHO_AM_I_VAL   0xEA
#define AK09916_WHO_AM_I_VAL    0x09

/* ========================================================================
 * FUNÇÕES DE INICIALIZAÇÃO
 * ======================================================================== */

void IMU_Init(IMU *imu)
{
    memset(imu, 0, sizeof(IMU));
    imu->status = IMU_STATUS_NOT_INIT;
    imu->is_calibrated = 0;
    imu->is_valid = 0;
    imu->has_initial_pos = 0;
    
    imu->sample_time_ms = 5.0f;
    imu->filter_type = IMU_FILTER_COMPLEMENTARY;
    imu->filter_alpha = 0.98f;
    
    imu->mag_sens = 0.15f;
    
    /* Matriz identidade para mag_scale */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            imu->mag_scale[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

HAL_StatusTypeDef IMU_Begin(IMU *imu, SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *cs_gpio, uint16_t cs_pin)
{
    if (imu == NULL || hspi == NULL || cs_gpio == NULL) {
        return HAL_ERROR;
    }
    
    imu->hspi = hspi;
    imu->cs_gpio = cs_gpio;
    imu->cs_pin = cs_pin;
    
    HAL_Delay(100);
    
    /* Verifica WHO_AM_I */
    uint8_t who_am_i = 0;
    if (IMU_SelectBank(imu, 0) != HAL_OK) return HAL_ERROR;
    if (IMU_ReadReg(imu, ICM20948_WHO_AM_I, &who_am_i, 1) != HAL_OK) return HAL_ERROR;
    
    if (who_am_i != ICM20948_WHO_AM_I_VAL) {
        imu->status = IMU_STATUS_NO_DEVICE;
        return HAL_ERROR;
    }
    
    /* Reset do dispositivo */
    IMU_WriteReg(imu, ICM20948_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    
    /* Configura clock interno */
    IMU_WriteReg(imu, ICM20948_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    
    /* Liga sensores */
    IMU_WriteReg(imu, ICM20948_PWR_MGMT_2, 0x00);
    HAL_Delay(10);
    
    /* Configura ranges padrão */
    if (IMU_SetRange(imu, ACC_RANGE_16G, GYRO_RANGE_2000DPS) != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* Inicializa magnetômetro */
    if (MAG_Init(imu) != HAL_OK) {
        return HAL_ERROR;
    }
    
    imu->status = IMU_STATUS_READY;
    imu->prev_time = HAL_GetTick();
    
    return HAL_OK;
}

HAL_StatusTypeDef IMU_Update_(IMU *imu)
{
    if (imu->status != IMU_STATUS_READY) {
        return HAL_ERROR;
    }

    imu->curr_time = HAL_GetTick();
    imu->dt = (imu->curr_time - imu->prev_time) / 1000.0f;

    /* Verifica se passou o tempo de amostragem */
    if ((imu->curr_time - imu->prev_time) < imu->sample_time_ms) {
        return HAL_OK;
    }

    /* Lê dados brutos */
    if (IMU_ReadRawData(imu) != HAL_OK) {
        imu->is_valid = 0;
        return HAL_ERROR;
    }

    /* Converte para unidades SI */
    IMU_ConvertData(imu);

    /* Filtra dados */
    IMU_FilterData(imu);

    /* Calcula atitude */
    //IMU_CalculateAttitude(imu); -----------------------------> Descomentar se for usar!!!!!!!

    /* Calcula referencial NED */
    IMU_CalculateNED(imu);

    imu->is_valid = 1;
    imu->last_update = HAL_GetTick();

    /* Salva posição inicial se necessário */
    if (!imu->has_initial_pos && imu->is_calibrated) {
        IMU_SaveInitialPosition(imu);
    } else if (imu->has_initial_pos) {
        IMU_CalculateDisplacement(imu);
    }

    imu->prev_time = imu->curr_time;

    return HAL_OK;
}

HAL_StatusTypeDef IMU_Update(IMU *imu, IMU_Fusion *fusion, ROCKET_FSM_t *fsm)
{
    if (imu->status != IMU_STATUS_READY) {
        return HAL_ERROR;
    }
    
    imu->curr_time = HAL_GetTick();
    imu->dt = (imu->curr_time - imu->prev_time) / 1000.0f;
    
    if ((imu->curr_time - imu->prev_time) < imu->sample_time_ms) {
        return HAL_OK;
    }
    
    if (IMU_ReadRawData(imu) != HAL_OK) {
        imu->is_valid = 0;
        return HAL_ERROR;
    }
    
    IMU_ConvertData(imu);
    IMU_FilterData(imu);
    
    IMU_CalculateAttitude_Adaptive(imu, fusion, fsm);
    
    IMU_CalculateNED(imu);
    
    imu->is_valid = 1;
    imu->last_update = HAL_GetTick();
    
    if (!imu->has_initial_pos && imu->is_calibrated) {
        IMU_SaveInitialPosition(imu);
    } else if (imu->has_initial_pos) {
        IMU_CalculateDisplacement(imu);
    }
    
    imu->prev_time = imu->curr_time;
    
    return HAL_OK;
}

/* ========================================================================
 * CONFIGURAÇÃO
 * ======================================================================== */

HAL_StatusTypeDef IMU_SetRange(IMU *imu, IMU_AccRange acc_range, IMU_GyroRange gyro_range)
{
    if (IMU_SelectBank(imu, 2) != HAL_OK) return HAL_ERROR;
    
    /* Configura acelerômetro */
    if (IMU_WriteReg(imu, ICM20948_ACCEL_CONFIG, (uint8_t)acc_range) != HAL_OK) {
        return HAL_ERROR;
    }
    
    imu->acc_range = acc_range;
    switch (acc_range) {
        case ACC_RANGE_2G:  imu->acc_sens = 16384.0f; break;
        case ACC_RANGE_4G:  imu->acc_sens = 8192.0f; break;
        case ACC_RANGE_8G:  imu->acc_sens = 4096.0f; break;
        case ACC_RANGE_16G: imu->acc_sens = 2048.0f; break;
    }
    
    /* Configura giroscópio */
    if (IMU_WriteReg(imu, ICM20948_GYRO_CONFIG_1, (uint8_t)gyro_range) != HAL_OK) {
        return HAL_ERROR;
    }
    
    imu->gyro_range = gyro_range;
    switch (gyro_range) {
        case GYRO_RANGE_250DPS:  imu->gyro_sens = 131.0f; break;
        case GYRO_RANGE_500DPS:  imu->gyro_sens = 65.5f; break;
        case GYRO_RANGE_1000DPS: imu->gyro_sens = 32.8f; break;
        case GYRO_RANGE_2000DPS: imu->gyro_sens = 16.4f; break;
    }
    
    IMU_SelectBank(imu, 0);
    
    return HAL_OK;
}

void IMU_SetFilterType(IMU *imu, IMU_FilterType filter_type)
{
    imu->filter_type = filter_type;
}

void IMU_SetFilterAlpha(IMU *imu, float alpha)
{
    if (alpha >= 0.0f && alpha <= 1.0f) {
        imu->filter_alpha = alpha;
    }
}

/* ========================================================================
 * CALIBRAÇÃO
 * ======================================================================== */

HAL_StatusTypeDef IMU_Calibrate(IMU *imu, uint16_t samples)
{
    float acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        if (IMU_ReadRawData(imu) != HAL_OK) {
            return HAL_ERROR;
        }
        
        IMU_ConvertData(imu);
        
        gyro_sum_x += imu->gyro_x;
        gyro_sum_y += imu->gyro_y;
        gyro_sum_z += imu->gyro_z;
        
        acc_sum_x += imu->acc_x;
        acc_sum_y += imu->acc_y;
        acc_sum_z += imu->acc_z;
        
        HAL_Delay(10);
    }
    
    /* Calcula offsets do giroscópio */
    imu->gyro_offset_x = gyro_sum_x / samples;
    imu->gyro_offset_y = gyro_sum_y / samples;
    imu->gyro_offset_z = gyro_sum_z / samples;
    
    /* Offset NED do acelerômetro (assume estático) */
    imu->acc_ned_offset_x = acc_sum_x / samples;
    imu->acc_ned_offset_y = acc_sum_y / samples;
    imu->acc_ned_offset_z = acc_sum_z / samples;
    
    imu->is_calibrated = 1;
    
    return HAL_OK;
}

void IMU_SetAccOffset(IMU *imu, float x, float y, float z)
{
    imu->acc_offset_x = x;
    imu->acc_offset_y = y;
    imu->acc_offset_z = z;
}

void IMU_SetGyroOffset(IMU *imu, float x, float y, float z)
{
    imu->gyro_offset_x = x;
    imu->gyro_offset_y = y;
    imu->gyro_offset_z = z;
}

void IMU_SetMagOffset(IMU *imu, float x, float y, float z)
{
    imu->mag_offset_x = x;
    imu->mag_offset_y = y;
    imu->mag_offset_z = z;
}

void IMU_SetMagScale(IMU *imu, float scale[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            imu->mag_scale[i][j] = scale[i][j];
        }
    }
}

/* ========================================================================
 * LEITURA E CONVERSÃO DE DADOS
 * ======================================================================== */

HAL_StatusTypeDef IMU_ReadRawData(IMU *imu)
{
    if (IMU_SelectBank(imu, 0) != HAL_OK) return HAL_ERROR;
    
    /* Lê acelerômetro, giroscópio, temperatura e magnetômetro */
    if (IMU_ReadReg(imu, ICM20948_ACCEL_OUT, imu->data_buffer, 20) != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* Acelerômetro */
    imu->raw_acc_x = (int16_t)((imu->data_buffer[0] << 8) | imu->data_buffer[1]);
    imu->raw_acc_y = (int16_t)((imu->data_buffer[2] << 8) | imu->data_buffer[3]);
    imu->raw_acc_z = (int16_t)((imu->data_buffer[4] << 8) | imu->data_buffer[5]);
    
    /* Giroscópio */
    imu->raw_gyro_x = (int16_t)((imu->data_buffer[6] << 8) | imu->data_buffer[7]);
    imu->raw_gyro_y = (int16_t)((imu->data_buffer[8] << 8) | imu->data_buffer[9]);
    imu->raw_gyro_z = (int16_t)((imu->data_buffer[10] << 8) | imu->data_buffer[11]);
    
    /* Temperatura */
    imu->raw_temp = (int16_t)((imu->data_buffer[12] << 8) | imu->data_buffer[13]);
    
    /* Magnetômetro (lido via I2C slave) */
    imu->raw_mag_x = (int16_t)((imu->data_buffer[15] << 8) | imu->data_buffer[14]);
    imu->raw_mag_y = -((int16_t)((imu->data_buffer[17] << 8) | imu->data_buffer[16]));
    imu->raw_mag_z = -((int16_t)((imu->data_buffer[19] << 8) | imu->data_buffer[18]));
    
    // INVERTENDO EIXOS

    imu->raw_acc_y = -imu->raw_acc_y;
    imu->raw_acc_z = -imu->raw_acc_z;

    /* Giroscópio */
    imu->raw_gyro_y = -imu->raw_gyro_y;
    imu->raw_gyro_z = -imu->raw_gyro_z;

    /* Magnetômetro */
    imu->raw_mag_y = -imu->raw_mag_y;
    imu->raw_mag_z = -imu->raw_mag_z;

    return HAL_OK;
}

void IMU_ConvertData(IMU *imu)
{
    /* Acelerômetro [g] */
    imu->acc_x = (imu->raw_acc_x / imu->acc_sens) - imu->acc_offset_x;
    imu->acc_y = (imu->raw_acc_y / imu->acc_sens) - imu->acc_offset_y;
    imu->acc_z = (imu->raw_acc_z / imu->acc_sens) - imu->acc_offset_z;
    
    /* Giroscópio [deg/s] */
    imu->gyro_x = (imu->raw_gyro_x / imu->gyro_sens) - imu->gyro_offset_x;
    imu->gyro_y = (imu->raw_gyro_y / imu->gyro_sens) - imu->gyro_offset_y;
    imu->gyro_z = (imu->raw_gyro_z / imu->gyro_sens) - imu->gyro_offset_z;
    
    /* Magnetômetro [uT] com calibração */
    float mag_raw_x = (imu->raw_mag_x * imu->mag_sens) - imu->mag_offset_x;
    float mag_raw_y = (imu->raw_mag_y * imu->mag_sens) - imu->mag_offset_y;
    float mag_raw_z = (imu->raw_mag_z * imu->mag_sens) - imu->mag_offset_z;
    
    imu->mag_x = imu->mag_scale[0][0] * mag_raw_x + 
                 imu->mag_scale[0][1] * mag_raw_y + 
                 imu->mag_scale[0][2] * mag_raw_z;
    imu->mag_y = imu->mag_scale[1][0] * mag_raw_x + 
                 imu->mag_scale[1][1] * mag_raw_y + 
                 imu->mag_scale[1][2] * mag_raw_z;
    imu->mag_z = imu->mag_scale[2][0] * mag_raw_x + 
                 imu->mag_scale[2][1] * mag_raw_y + 
                 imu->mag_scale[2][2] * mag_raw_z;
    
    /* Temperatura [°C] */
    imu->temperature = (imu->raw_temp / 333.87f) + 21.0f;
}

void IMU_FilterData(IMU *imu)
{
    /* Filtro passa-baixa simples (média móvel exponencial) */
    float alpha = 0.3f;
    
    imu->acc_x_f = alpha * imu->acc_x + (1.0f - alpha) * imu->acc_x_f;
    imu->acc_y_f = alpha * imu->acc_y + (1.0f - alpha) * imu->acc_y_f;
    imu->acc_z_f = alpha * imu->acc_z + (1.0f - alpha) * imu->acc_z_f;
    
    imu->gyro_x_f = alpha * imu->gyro_x + (1.0f - alpha) * imu->gyro_x_f;
    imu->gyro_y_f = alpha * imu->gyro_y + (1.0f - alpha) * imu->gyro_y_f;
    imu->gyro_z_f = alpha * imu->gyro_z + (1.0f - alpha) * imu->gyro_z_f;
    
    imu->mag_x_f = alpha * imu->mag_x + (1.0f - alpha) * imu->mag_x_f;
    imu->mag_y_f = alpha * imu->mag_y + (1.0f - alpha) * imu->mag_y_f;
    imu->mag_z_f = alpha * imu->mag_z + (1.0f - alpha) * imu->mag_z_f;
}

//void IMU_CalculateAttitude(IMU *imu)
//{
//    /* Roll e Pitch do acelerômetro */
//    imu->roll_acc = atan2f(imu->acc_y_f, imu->acc_z_f);
//    imu->pitch_acc = atan2f(-imu->acc_x_f,
//                           sqrtf(imu->acc_y_f * imu->acc_y_f +
//                                imu->acc_z_f * imu->acc_z_f));
//
//    /* Yaw do magnetômetro (compensado por roll e pitch) */
//    float mag_x_comp = imu->mag_x_f * cosf(imu->pitch) +
//                       imu->mag_z_f * sinf(imu->pitch);
//    float mag_y_comp = imu->mag_x_f * sinf(imu->roll) * sinf(imu->pitch) +
//                       imu->mag_y_f * cosf(imu->roll) -
//                       imu->mag_z_f * sinf(imu->roll) * cosf(imu->pitch);
//
//    imu->yaw_mag = atan2f(-mag_y_comp, mag_x_comp);
//
//    /* Integração do giroscópio */
//    float gyro_roll = imu->roll + (imu->gyro_x_f * imu->dt * M_PI / 180.0f);
//    float gyro_pitch = imu->pitch + (imu->gyro_y_f * imu->dt * M_PI / 180.0f);
//    float gyro_yaw = imu->yaw + (imu->gyro_z_f * imu->dt * M_PI / 180.0f);
//
//    /* Filtro complementar */
//    float alpha = imu->filter_alpha;
//    imu->roll = alpha * gyro_roll + (1.0f - alpha) * imu->roll_acc;
//    imu->pitch = alpha * gyro_pitch + (1.0f - alpha) * imu->pitch_acc;
//    imu->yaw = alpha * gyro_yaw + (1.0f - alpha) * imu->yaw_mag;
//}

void IMU_CalculateNED(IMU *imu)
{
    /* Matriz de rotação Body -> NED */
    float cr = cosf(imu->roll);
    float sr = sinf(imu->roll);
    float cp = cosf(imu->pitch);
    float sp = sinf(imu->pitch);
    float cy = cosf(imu->yaw);
    float sy = sinf(imu->yaw);

    imu->rot_matrix[0][0] = cy * cp;
    imu->rot_matrix[0][1] = cy * sp * sr - sy * cr;
    imu->rot_matrix[0][2] = cy * sp * cr + sy * sr;

    imu->rot_matrix[1][0] = sy * cp;
    imu->rot_matrix[1][1] = sy * sp * sr + cy * cr;
    imu->rot_matrix[1][2] = sy * sp * cr - cy * sr;

    imu->rot_matrix[2][0] = -sp;
    imu->rot_matrix[2][1] = cp * sr;
    imu->rot_matrix[2][2] = cp * cr;

    /* Transforma aceleração para NED */
    imu->acc_ned_x = (imu->rot_matrix[0][0] * imu->acc_x_f +
                      imu->rot_matrix[0][1] * imu->acc_y_f +
                      imu->rot_matrix[0][2] * imu->acc_z_f) * 9.81f;

    imu->acc_ned_y = (imu->rot_matrix[1][0] * imu->acc_x_f +
                      imu->rot_matrix[1][1] * imu->acc_y_f +
                      imu->rot_matrix[1][2] * imu->acc_z_f) * 9.81f;

    imu->acc_ned_z = (imu->rot_matrix[2][0] * imu->acc_x_f +
                      imu->rot_matrix[2][1] * imu->acc_y_f +
                      imu->rot_matrix[2][2] * imu->acc_z_f) * 9.81f;

    /* Remove offset (gravidade) */
    imu->acc_ned_x -= imu->acc_ned_offset_x;
    imu->acc_ned_y -= imu->acc_ned_offset_y;
    imu->acc_ned_z -= imu->acc_ned_offset_z;

    /* Threshold para ruído */
    if (fabsf(imu->acc_ned_x) < 0.1f * 9.81f) imu->acc_ned_x = 0.0f;
    if (fabsf(imu->acc_ned_y) < 0.1f * 9.81f) imu->acc_ned_y = 0.0f;
    if (fabsf(imu->acc_ned_z) < 0.1f * 9.81f) imu->acc_ned_z = 0.0f;

    /* Integração da velocidade */
    imu->vel_ned_x += imu->acc_ned_x * imu->dt;
    imu->vel_ned_y += imu->acc_ned_y * imu->dt;
    imu->vel_ned_z += imu->acc_ned_z * imu->dt;

    /* Threshold para velocidade */
    if (fabsf(imu->vel_ned_x) < 0.1f) imu->vel_ned_x = 0.0f;
    if (fabsf(imu->vel_ned_y) < 0.1f) imu->vel_ned_y = 0.0f;
    if (fabsf(imu->vel_ned_z) < 0.1f) imu->vel_ned_z = 0.0f;

    /* Integração da posição */
    imu->pos_ned_x += imu->vel_ned_x * imu->dt;
    imu->pos_ned_y += imu->vel_ned_y * imu->dt;
    imu->pos_ned_z += imu->vel_ned_z * imu->dt;
}

/* ========================================================================
 * POSIÇÃO E DESLOCAMENTO
 * ======================================================================== */

void IMU_SaveInitialPosition(IMU *imu)
{
    if (!imu->is_valid) return;

    imu->roll_initial = imu->roll;
    imu->pitch_initial = imu->pitch;
    imu->yaw_initial = imu->yaw;

    imu->pos_x_initial = imu->pos_ned_x;
    imu->pos_y_initial = imu->pos_ned_y;
    imu->pos_z_initial = imu->pos_ned_z;

    imu->has_initial_pos = 1;

    /* Reset deslocamentos */
    imu->displacement_x = 0.0f;
    imu->displacement_y = 0.0f;
    imu->displacement_z = 0.0f;
    imu->displacement_roll = 0.0f;
    imu->displacement_pitch = 0.0f;
    imu->displacement_yaw = 0.0f;
}

void IMU_CalculateDisplacement(IMU *imu)
{
    if (!imu->has_initial_pos || !imu->is_valid) return;

    /* Deslocamento em posição */
    imu->displacement_x = imu->pos_ned_x - imu->pos_x_initial;
    imu->displacement_y = imu->pos_ned_y - imu->pos_y_initial;
    imu->displacement_z = imu->pos_ned_z - imu->pos_z_initial;

    /* Deslocamento em atitude */
    imu->displacement_roll = imu->roll - imu->roll_initial;
    imu->displacement_pitch = imu->pitch - imu->pitch_initial;
    imu->displacement_yaw = imu->yaw - imu->yaw_initial;
}

void IMU_ResetPosition(IMU *imu)
{
    imu->vel_ned_x = 0.0f;
    imu->vel_ned_y = 0.0f;
    imu->vel_ned_z = 0.0f;

    imu->pos_ned_x = 0.0f;
    imu->pos_ned_y = 0.0f;
    imu->pos_ned_z = 0.0f;

    imu->has_initial_pos = 0;
}

/* ========================================================================
 * FUNÇÕES DE IMPRESSÃO
 * ======================================================================== */

void IMU_PrintInfo(IMU *imu)
{
    printf("\r\n========== INFORMACOES DO IMU ==========\r\n");

    printf("\r\n--- STATUS ---\r\n");
    printf("Status: ");
    switch(imu->status) {
        case IMU_STATUS_NOT_INIT:  printf("Nao Inicializado\r\n"); break;
        case IMU_STATUS_ERROR:     printf("Erro\r\n"); break;
        case IMU_STATUS_NO_DEVICE: printf("Dispositivo Nao Encontrado\r\n"); break;
        case IMU_STATUS_READY:     printf("Pronto\r\n"); break;
        default:                   printf("Desconhecido\r\n"); break;
    }

    printf("Calibrado: %s\r\n", imu->is_calibrated ? "Sim" : "Nao");
    printf("Dados Validos: %s\r\n", imu->is_valid ? "Sim" : "Nao");
    printf("Ultima Atualizacao: %lu ms\r\n", imu->last_update);
    printf("Sample Time: %.2f ms\r\n", imu->sample_time_ms);

    if (imu->is_valid) {
        printf("\r\n--- ACELEROMETRO [g] ---\r\n");
        printf("X: %.4f  Y: %.4f  Z: %.4f\r\n",
               imu->acc_x, imu->acc_y, imu->acc_z);
        printf("Filtrado - X: %.4f  Y: %.4f  Z: %.4f\r\n",
               imu->acc_x_f, imu->acc_y_f, imu->acc_z_f);

        printf("\r\n--- GIROSCOPIO [deg/s] ---\r\n");
        printf("X: %.4f  Y: %.4f  Z: %.4f\r\n",
               imu->gyro_x, imu->gyro_y, imu->gyro_z);
        printf("Filtrado - X: %.4f  Y: %.4f  Z: %.4f\r\n",
               imu->gyro_x_f, imu->gyro_y_f, imu->gyro_z_f);

        printf("\r\n--- MAGNETOMETRO [uT] ---\r\n");
        printf("X: %.4f  Y: %.4f  Z: %.4f\r\n",
               imu->mag_x, imu->mag_y, imu->mag_z);

        printf("\r\n--- TEMPERATURA ---\r\n");
        printf("%.2f C\r\n", imu->temperature);

        printf("\r\n--- ATITUDE [graus] ---\r\n");
        printf("Roll:  %.2f\r\n", imu->roll * 180.0f / M_PI);
        printf("Pitch: %.2f\r\n", imu->pitch * 180.0f / M_PI);
        printf("Yaw:   %.2f\r\n", imu->yaw * 180.0f / M_PI);

        printf("\r\n--- REFERENCIAL NED ---\r\n");
        printf("Aceleracao - X: %.2f  Y: %.2f  Z: %.2f m/s2\r\n",
               imu->acc_ned_x, imu->acc_ned_y, imu->acc_ned_z);
        printf("Velocidade - X: %.2f  Y: %.2f  Z: %.2f m/s\r\n",
               imu->vel_ned_x, imu->vel_ned_y, imu->vel_ned_z);
        printf("Posicao    - X: %.2f  Y: %.2f  Z: %.2f m\r\n",
               imu->pos_ned_x, imu->pos_ned_y, imu->pos_ned_z);

        if (imu->has_initial_pos) {
            printf("\r\n--- DESLOCAMENTO ---\r\n");
            printf("Posicao - X: %.2f m  Y: %.2f m  Z: %.2f m\r\n",
                   imu->displacement_x, imu->displacement_y, imu->displacement_z);
            printf("Atitude - Roll: %.2f deg  Pitch: %.2f deg  Yaw: %.2f deg\r\n",
                   imu->displacement_roll * 180.0f / M_PI,
                   imu->displacement_pitch * 180.0f / M_PI,
                   imu->displacement_yaw * 180.0f / M_PI);
        }
    }

    printf("\r\n========================================\r\n\r\n");
}

void IMU_PrintCalibration(IMU *imu)
{
    printf("\n--- CALIBRACAO DO IMU ---\n");
    printf("Acelerometro Offset - X: %.4f  Y: %.4f  Z: %.4f\n",
           imu->acc_offset_x, imu->acc_offset_y, imu->acc_offset_z);
    printf("Giroscopio Offset   - X: %.4f  Y: %.4f  Z: %.4f\n",
           imu->gyro_offset_x, imu->gyro_offset_y, imu->gyro_offset_z);
    printf("Magnetometro Offset - X: %.4f  Y: %.4f  Z: %.4f\n",
           imu->mag_offset_x, imu->mag_offset_y, imu->mag_offset_z);
    printf("Magnetometro Scale:\n");
    printf("  [%.4f  %.4f  %.4f]\n",
           imu->mag_scale[0][0], imu->mag_scale[0][1], imu->mag_scale[0][2]);
    printf("  [%.4f  %.4f  %.4f]\n",
           imu->mag_scale[1][0], imu->mag_scale[1][1], imu->mag_scale[1][2]);
    printf("  [%.4f  %.4f  %.4f]\n",
           imu->mag_scale[2][0], imu->mag_scale[2][1], imu->mag_scale[2][2]);
}

/* ========================================================================
 * FUNÇÕES DE BAIXO NÍVEL - SPI
 * ======================================================================== */

HAL_StatusTypeDef IMU_SelectBank(IMU *imu, uint8_t bank)
{
    if (bank > 3) return HAL_ERROR;

    uint8_t bank_value = bank << 4;

    if (bank_value != imu->current_bank) {
        if (IMU_WriteReg(imu, ICM20948_REG_BANK_SEL, bank_value) != HAL_OK) {
            return HAL_ERROR;
        }
        imu->current_bank = bank_value;
    }

    return HAL_OK;
}

HAL_StatusTypeDef IMU_WriteReg(IMU *imu, uint8_t reg, uint8_t data)
{
    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(imu->hspi, &reg, 1, 100);
    if (status == HAL_OK) {
        status = HAL_SPI_Transmit(imu->hspi, &data, 1, 100);
    }

    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs_pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef IMU_ReadReg(IMU *imu, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t reg_read = 0x80 | reg;

    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs_pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(imu->hspi, &reg_read, 1, 100);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(imu->hspi, data, len, 100);
    }

    HAL_GPIO_WritePin(imu->cs_gpio, imu->cs_pin, GPIO_PIN_SET);

    return status;
}

/* ========================================================================
 * MAGNETÔMETRO AK09916
 * ======================================================================== */

HAL_StatusTypeDef MAG_Init(IMU *imu)
{
    /* Reset I2C Master */
    IMU_SelectBank(imu, 0);
    uint8_t val;
    IMU_ReadReg(imu, ICM20948_USER_CTRL, &val, 1);
    val |= 0x02;
    IMU_WriteReg(imu, ICM20948_USER_CTRL, val);
    HAL_Delay(100);

    /* Habilita I2C Master */
    IMU_ReadReg(imu, ICM20948_USER_CTRL, &val, 1);
    val |= 0x20;
    IMU_WriteReg(imu, ICM20948_USER_CTRL, val);
    HAL_Delay(10);

    /* Configura clock I2C */
    IMU_SelectBank(imu, 3);
    IMU_WriteReg(imu, ICM20948_I2C_MST_CTRL, 0x07);
    HAL_Delay(10);

    /* Reset AK09916 */
    MAG_WriteReg(imu, AK09916_CNTL3, 0x01);
    HAL_Delay(100);

    /* Modo de medição contínua (100Hz) */
    MAG_WriteReg(imu, AK09916_CNTL2, 0x08);
    HAL_Delay(100);

    /* Configura leitura automática */
    MAG_ReadReg(imu, AK09916_HXL, 9);

    IMU_SelectBank(imu, 0);

    return HAL_OK;
}

HAL_StatusTypeDef IMU_CollectMagDataToSD(IMU *imu, SD *sd, uint32_t duration_ms, const char *filename)
{
    if (!imu->is_valid) {
        printf("[MAG] IMU nao inicializado\r\n");
        return HAL_ERROR;
    }

    if (!sd->is_mounted) {
        printf("[MAG] SD nao montado\r\n");
        return HAL_ERROR;
    }

    printf("\r\n========== COLETA DE DADOS DO MAGNETOMETRO ==========\r\n");
    printf("Duracao: %lu segundos\r\n", duration_ms / 1000);
    printf("Arquivo: %s\r\n", filename);
    printf("\r\nINSTRUCOES:\r\n");
    printf("1. Gire o modulo LENTAMENTE em todas as direcoes\r\n");
    printf("2. Faca movimentos em forma de '8' no ar\r\n");
    printf("3. Cubra todas as orientacoes possiveis\r\n");
    printf("4. IMPORTANTE: Afaste-se de metais e imas!\r\n");
    printf("\r\nIniciando em 3 segundos...\r\n");
    HAL_Delay(3000);

    // Abre arquivo
    if (SD_OpenFile(sd, filename, SD_MODE_OVERWRITE) != HAL_OK) {
        printf("[ERRO] Falha ao abrir arquivo\r\n");
        return HAL_ERROR;
    }

    // Escreve cabeçalho (formato compatível com magcal do MATLAB)
    SD_WriteLine(sd, "Mag_X Mag_Y Mag_Z");

    uint32_t start_time = HAL_GetTick();
    uint32_t last_print = start_time;
    uint32_t sample_count = 0;

    printf("\r\nColeta iniciada...\r\n");

    // Loop de coleta
    while ((HAL_GetTick() - start_time) < duration_ms) {
        if (IMU_Update_(imu) == HAL_OK) {
            // Grava diretamente no SD (formato: X Y Z)
            char line[64];
            snprintf(line, sizeof(line), "%.4f %.4f %.4f",
                    imu->mag_x, imu->mag_y, imu->mag_z);

            if (SD_WriteLine(sd, line) == HAL_OK) {
                sample_count++;
            }

            // Flush a cada 50 amostras para não perder dados
            if (sample_count % 50 == 0) {
                SD_Sync(sd);
            }
        }

        // Progresso
        if ((HAL_GetTick() - last_print) >= 1000) {
            uint32_t elapsed = (HAL_GetTick() - start_time) / 1000;
            uint32_t remaining = (duration_ms / 1000) - elapsed;
            printf("[%lu/%lu s] Amostras: %lu\r\n",
                   elapsed, duration_ms / 1000, sample_count);
            last_print = HAL_GetTick();
        }

        HAL_Delay(10); // ~100Hz de amostragem
    }

    // Fecha arquivo
    SD_CloseFile(sd);

    printf("\r\n========== COLETA CONCLUIDA ==========\r\n");
    printf("Amostras coletadas: %lu\r\n", sample_count);
    printf("Arquivo: %s\r\n", filename);
    printf("\r\n--- Proximos passos ---\r\n");
    printf("1. Remova o cartao SD\r\n");
    printf("2. Execute o script MATLAB: MagCalibration.m\r\n");
    printf("3. Copie os valores A e b gerados\r\n");
    printf("4. Use IMU_SetMagCalibration() no codigo\r\n");
    printf("======================================\r\n");

    return HAL_OK;
}

void IMU_SetMagCalibration(IMU *imu, float offset[3], float scale[3][3])
{
    IMU_SetMagOffset(imu, offset[0], offset[1], offset[2]);
    IMU_SetMagScale(imu, scale);
}

HAL_StatusTypeDef MAG_WriteReg(IMU *imu, uint8_t reg, uint8_t data)
{
    IMU_SelectBank(imu, 3);

    IMU_WriteReg(imu, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);
    HAL_Delay(10);
    IMU_WriteReg(imu, ICM20948_I2C_SLV0_REG, reg);
    HAL_Delay(10);
    IMU_WriteReg(imu, ICM20948_I2C_SLV0_DO, data);
    HAL_Delay(10);
    IMU_WriteReg(imu, ICM20948_I2C_SLV0_CTRL, 0x80 | 0x01);
    HAL_Delay(100);

    IMU_SelectBank(imu, 0);

    return HAL_OK;
}

HAL_StatusTypeDef MAG_ReadReg(IMU *imu, uint8_t reg, uint8_t len)
{
    IMU_SelectBank(imu, 3);

    IMU_WriteReg(imu, ICM20948_I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
    HAL_Delay(10);
    IMU_WriteReg(imu, ICM20948_I2C_SLV0_REG, reg);
    HAL_Delay(10);
    IMU_WriteReg(imu, ICM20948_I2C_SLV0_CTRL, 0x80 | len);
    HAL_Delay(100);

    IMU_SelectBank(imu, 0);

    return HAL_OK;
}
