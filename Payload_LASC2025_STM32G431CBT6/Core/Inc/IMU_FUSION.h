/*
 * IMU_FUSION.h
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Biblioteca de fusão sensorial e filtros para IMU
 */

#ifndef INC_IMU_FUSION_H_
#define INC_IMU_FUSION_H_

#include "main.h"
#include "IMU.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//#include "ROCKET_FSM.h"

typedef struct ROCKET_FSM_s ROCKET_FSM_t;

/* Quaternion */
typedef struct {
    float w, x, y, z;
} Quaternion;

/* Filtro Digital (IIR) */
typedef struct {
    float a[4];  /* Coeficientes do numerador */
    float b[4];  /* Coeficientes do denominador */
    float x[4];  /* Valores passados da entrada */
    float y[4];  /* Valores passados da saída */
} IIR_Filter;

/* Filtro de Kalman para atitude */
typedef struct {
    /* Estado: [ângulo, bias_gyro] */
    float x[2];
    
    /* Matriz de covariância */
    float P[2][2];
    
    /* Ganho de Kalman */
    float K[2];
    
    /* Ruído do processo e medição */
    float Q_angle;
    float Q_bias;
    float R_measure;
    
} Kalman_Filter;

/* Filtro Madgwick */
typedef struct {
    Quaternion q;
    float beta;              /* Ganho do filtro */
    float sample_freq;       /* Frequência de amostragem */
} Madgwick_Filter;

/* Estrutura principal de fusão */
typedef struct IMU_FUSION_s {
    /* Filtros digitais para cada eixo */
    IIR_Filter acc_lpf[3];      /* Passa-baixa acelerômetro */
    IIR_Filter gyro_lpf[3];     /* Passa-baixa giroscópio */
    IIR_Filter gyro_hpf[3];     /* Passa-alta giroscópio */
    IIR_Filter mag_lpf[3];      /* Passa-baixa magnetômetro */
    
    /* Filtros de Kalman */
    Kalman_Filter kf_roll;
    Kalman_Filter kf_pitch;
    Kalman_Filter kf_yaw;
    
    /* Filtro Madgwick */
    Madgwick_Filter madgwick;
    
    /* Dados filtrados */
    float acc_x_f, acc_y_f, acc_z_f;
    float gyro_x_f, gyro_y_f, gyro_z_f;
    float mag_x_f, mag_y_f, mag_z_f;
    
    /* Atitude calculada por diferentes métodos */
    float roll_cf, pitch_cf, yaw_cf;        /* Complementar */
    float roll_kf, pitch_kf, yaw_kf;        /* Kalman */
    float roll_mw, pitch_mw, yaw_mw;        /* Madgwick */
    
    /* Configuração */
    float alpha_complementary;
    uint8_t use_lowpass;
    uint8_t use_highpass;
    
} IMU_Fusion;

/* Funções principais */
void FUSION_Init(IMU_Fusion *fusion, float sample_freq);
void FUSION_Update(IMU_Fusion *fusion, IMU *imu);

/* Filtros digitais IIR */
void FILTER_InitLowPass(IIR_Filter *filter, float cutoff_freq, float sample_freq);
void FILTER_InitHighPass(IIR_Filter *filter, float cutoff_freq, float sample_freq);
float FILTER_Apply(IIR_Filter *filter, float input);
void FILTER_Reset(IIR_Filter *filter);

/* Filtro Complementar */
void FUSION_ComplementaryFilter(IMU_Fusion *fusion, IMU *imu);

/* Filtro de Kalman */
void KALMAN_Init(Kalman_Filter *kf, float q_angle, float q_bias, float r_measure);
float KALMAN_Update(Kalman_Filter *kf, float measurement, float rate, float dt);
void KALMAN_Reset(Kalman_Filter *kf);

/* Filtro Madgwick */
void MADGWICK_Init(Madgwick_Filter *mw, float beta, float sample_freq);
void MADGWICK_Update(Madgwick_Filter *mw, float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz, float dt);
void MADGWICK_UpdateIMU(Madgwick_Filter *mw, float gx, float gy, float gz,
                        float ax, float ay, float az, float dt);

/* Quaternion */
void QUAT_Init(Quaternion *q, float w, float x, float y, float z);
void QUAT_Normalize(Quaternion *q);
void QUAT_ToEuler(Quaternion *q, float *roll, float *pitch, float *yaw);
void QUAT_FromEuler(Quaternion *q, float roll, float pitch, float yaw);
float QUAT_InvSqrt(float x);

/* Utilidades */
void FUSION_ApplyFilters(IMU_Fusion *fusion, IMU *imu);
void FUSION_SetComplementaryAlpha(IMU_Fusion *fusion, float alpha);
void FUSION_SetMadgwickBeta(IMU_Fusion *fusion, float beta);
void FUSION_PrintAngles(IMU_Fusion *fusion);

/* ========================================================================
 * FUSÃO SENSORIAL ADAPTATIVA
 * ======================================================================== */
void FUSION_Update_Adaptive(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm);
void FUSION_Update_GyroOnly(IMU_Fusion *fusion, IMU *imu);
void FUSION_Recover_From_Boost(IMU_Fusion *fusion, IMU *imu);
void FUSION_Kalman_Adaptive(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm);
void FUSION_ComplementaryFilter_Adaptive(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm);
uint8_t FUSION_IsAttitudeReliable(IMU_Fusion *fusion, IMU *imu, ROCKET_FSM_t *fsm);
void IMU_CalculateAttitude_Adaptive(IMU *imu, IMU_Fusion *fusion, ROCKET_FSM_t *fsm);

#endif /* INC_IMU_FUSION_H_ */
