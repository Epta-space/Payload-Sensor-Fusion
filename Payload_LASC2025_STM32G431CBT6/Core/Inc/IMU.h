/*
 * IMU.h
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Driver para IMU ICM-20948 (Accel + Gyro + Mag)
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "SD.h"

typedef struct ROCKET_FSM_s ROCKET_FSM_t;
typedef struct IMU_FUSION_s IMU_Fusion;

/* Ranges do Acelerômetro */
typedef enum {
    ACC_RANGE_2G = 0b00 << 1,
    ACC_RANGE_4G = 0b01 << 1,
    ACC_RANGE_8G = 0b10 << 1,
    ACC_RANGE_16G = 0b11 << 1
} IMU_AccRange;

/* Ranges do Giroscópio */
typedef enum {
    GYRO_RANGE_250DPS = 0b00 << 1,
    GYRO_RANGE_500DPS = 0b01 << 1,
    GYRO_RANGE_1000DPS = 0b10 << 1,
    GYRO_RANGE_2000DPS = 0b11 << 1
} IMU_GyroRange;

/* Status do IMU */
typedef enum {
    IMU_STATUS_NOT_INIT = 0,
    IMU_STATUS_ERROR = -1,
    IMU_STATUS_NO_DEVICE = -2,
    IMU_STATUS_READY = 7
} IMU_Status;

/* Tipo de filtro de atitude */
typedef enum {
    IMU_FILTER_NONE = 0,
    IMU_FILTER_COMPLEMENTARY,
    IMU_FILTER_KALMAN,
    IMU_FILTER_MADGWICK
} IMU_FilterType;

typedef struct {
    /* Handler SPI */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_gpio;
    uint16_t cs_pin;
    
    /* Confiabilidade da atitude */
    uint8_t attitude_reliable;      // 1 = confiável, 0 = não confiar

    /* Estado anterior para detecção de transição */
    uint8_t last_boost_state;       // Detecta saída de BOOST

    /* Status */
    IMU_Status status;
    uint8_t is_calibrated;
    uint8_t is_valid;
    uint32_t last_update;
    float sample_time_ms;
    
    /* Dados brutos */
    int16_t raw_acc_x, raw_acc_y, raw_acc_z;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    int16_t raw_mag_x, raw_mag_y, raw_mag_z;
    int16_t raw_temp;
    
    /* Dados convertidos [unidades SI] */
    float acc_x, acc_y, acc_z;           /* Aceleração em g */
    float gyro_x, gyro_y, gyro_z;        /* Velocidade angular em deg/s */
    float mag_x, mag_y, mag_z;           /* Campo magnético em uT */
    float temperature;                    /* Temperatura em °C */
    
    /* Dados filtrados */
    float acc_x_f, acc_y_f, acc_z_f;
    float gyro_x_f, gyro_y_f, gyro_z_f;
    float mag_x_f, mag_y_f, mag_z_f;

    /* Ângulos de Euler [radianos] */
    float roll, pitch, yaw;              /* Phi, Theta, Gamma */
    float roll_acc, pitch_acc;           /* Do acelerômetro */
    float yaw_mag;                       /* Do magnetômetro */
    
    /* Referencial NED (North-East-Down) */
    float acc_ned_x, acc_ned_y, acc_ned_z;
    float vel_ned_x, vel_ned_y, vel_ned_z;
    float pos_ned_x, pos_ned_y, pos_ned_z;
    
    /* Matriz de rotação Body -> NED */
    float rot_matrix[3][3];
    
    /* Calibração */
    float acc_offset_x, acc_offset_y, acc_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    float mag_offset_x, mag_offset_y, mag_offset_z;
    float mag_scale[3][3];
    
    float acc_ned_offset_x, acc_ned_offset_y, acc_ned_offset_z;
    
    /* Sensibilidades */
    float acc_sens;
    float gyro_sens;
    float mag_sens;
    
    /* Configuração */
    IMU_AccRange acc_range;
    IMU_GyroRange gyro_range;
    IMU_FilterType filter_type;
    float filter_alpha;                   /* Para filtro complementar */
    
    /* Tempo */
    float curr_time;
    float prev_time;
    float dt;
    
    /* Buffer interno */
    uint8_t data_buffer[22];
    uint8_t current_bank;
    
    /* Posição inicial */
    uint8_t has_initial_pos;
    float roll_initial, pitch_initial, yaw_initial;
    float pos_x_initial, pos_y_initial, pos_z_initial;
    
    /* Deslocamento */
    float displacement_x, displacement_y, displacement_z;
    float displacement_roll, displacement_pitch, displacement_yaw;
    
} IMU;

/* Funções principais */
void IMU_Init(IMU *imu);
HAL_StatusTypeDef IMU_Begin(IMU *imu, SPI_HandleTypeDef *hspi, 
                            GPIO_TypeDef *cs_gpio, uint16_t cs_pin);
HAL_StatusTypeDef IMU_Update_(IMU *imu);
HAL_StatusTypeDef IMU_Update(IMU *imu, IMU_Fusion *fusion, ROCKET_FSM_t *fsm);

/* Configuração */
HAL_StatusTypeDef IMU_SetRange(IMU *imu, IMU_AccRange acc_range, IMU_GyroRange gyro_range);
void IMU_SetFilterType(IMU *imu, IMU_FilterType filter_type);
void IMU_SetFilterAlpha(IMU *imu, float alpha);

/* Calibração */
HAL_StatusTypeDef IMU_Calibrate(IMU *imu, uint16_t samples);
void IMU_SetAccOffset(IMU *imu, float x, float y, float z);
void IMU_SetGyroOffset(IMU *imu, float x, float y, float z);
void IMU_SetMagOffset(IMU *imu, float x, float y, float z);
void IMU_SetMagScale(IMU *imu, float scale[3][3]);

/* Leitura de dados */
HAL_StatusTypeDef IMU_ReadRawData(IMU *imu);
void IMU_ConvertData(IMU *imu);
void IMU_FilterData(IMU *imu);
//void IMU_CalculateAttitude(IMU *imu);
void IMU_CalculateNED(IMU *imu);

/* Posição e deslocamento */
void IMU_SaveInitialPosition(IMU *imu);
void IMU_CalculateDisplacement(IMU *imu);
void IMU_ResetPosition(IMU *imu);

/* Utilidades */
void IMU_PrintInfo(IMU *imu);
void IMU_PrintCalibration(IMU *imu);

/* Funções internas de baixo nível */
HAL_StatusTypeDef IMU_WriteReg(IMU *imu, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMU_ReadReg(IMU *imu, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef IMU_SelectBank(IMU *imu, uint8_t bank);

/* Magnetômetro */
HAL_StatusTypeDef MAG_Init(IMU *imu);
HAL_StatusTypeDef IMU_CollectMagDataToSD(IMU *imu, SD *sd, uint32_t duration_ms, const char *filename);
void IMU_SetMagCalibration(IMU *imu, float offset[3], float scale[3][3]);
HAL_StatusTypeDef MAG_WriteReg(IMU *imu, uint8_t reg, uint8_t data);
HAL_StatusTypeDef MAG_ReadReg(IMU *imu, uint8_t reg, uint8_t len);

#endif /* INC_IMU_H_ */
