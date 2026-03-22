/*
 * ROCKET.h
 *
 *  Created on: Feb 9, 2025
 *      Author: de4lerr
 *  Description: Biblioteca de alto nível para gerenciamento completo de sistemas de foguete
 */

#ifndef INC_ROCKET_H_
#define INC_ROCKET_H_

#include "main.h"
#include "GNSS.h"
#include "SD.h"
#include "LORA.h"
#include "BAR.h"
#include "IMU.h"
#include "IMU_FUSION.h"
#include "ROCKET_FSM.h"
#include <stdio.h>
#include <math.h>

/* ========================================================================
 * CONFIGURAÇÕES DE GRAVAÇÃO
 * ======================================================================== */

#define FLIGHT_BUFFER_SIZE 60      // Aumentado de 100 para 200
#define SD_BUFFER_SIZE 1024          // Buffer maior para escritas em bloco
#define CRITICAL_FLUSH_INTERVAL 100  // Flush a cada 100ms em voo crítico

/* ========================================================================
 * ESTRUTURAS DE DADOS PARA LOGGING OTIMIZADO
 * ======================================================================== */

/* Dados completos para gravação (otimizado) */
typedef struct __attribute__((__packed__)) {
    uint32_t timestamp_ms;      // Timestamp absoluto

    // GPS
    float lat;
    float lon;
    float alt_gps;
    uint8_t gnss_fix;
    uint8_t gnss_sats;
    char gnss_time[9];

    // Barômetro
    float alt_bar;
    float pressure;
    float temperature;

    // EKF - Posição NED
    float pos_n;
    float pos_e;
    float pos_d;

    // EKF - Velocidade NED
    float vel_n;
    float vel_e;
    float vel_d;

    // IMU - Aceleração NED (sem gravidade)
    float acc_n;
    float acc_e;
    float acc_d;

    // IMU - Aceleração Body (raw)
    float acc_x_raw;
    float acc_y_raw;
    float acc_z_raw;

    // IMU - Giroscópio
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // NOVO: IMU - Magnetômetro
    float mag_x;
    float mag_y;
    float mag_z;

    // Atitude (Euler em graus)
    float roll;
    float pitch;
    float yaw;

    // Estado da FSM
    uint8_t flight_state;

    // Flags de validade
    uint8_t imu_valid;
    uint8_t ekf_valid;
    uint8_t bar_valid;
    uint8_t attitude_reliable;

} ROCKET_LogData_t;

/* Buffer circular inteligente */
typedef struct {
    ROCKET_LogData_t samples[FLIGHT_BUFFER_SIZE];
    uint16_t write_idx;
    uint16_t count;
    uint8_t overflow;
    uint32_t lost_samples;
} ROCKET_DataBuffer_t;

/* Sistema de gravação */
typedef struct {
    ROCKET_DataBuffer_t buffer;

    char log_buffer[SD_BUFFER_SIZE];
    uint16_t log_buffer_pos;

    uint32_t last_flush;
    uint32_t samples_written;
    uint32_t flush_count;

    uint8_t file_open;
    uint8_t header_written;

    char current_filename[32];

} ROCKET_Logger_t;

/* ========================================================================
 * ESTRUTURAS DE FUSÃO SENSORIAL (EKF)
 * ======================================================================== */

/* Filtro de Kalman Estendido para posição 3D */
typedef struct {
    float dt;           // Intervalo de tempo (s)
    float x[6];         // Estado: [x, y, z, vx, vy, vz]
    float P[6][6];      // Covariância do estado
    float Q[6][6];      // Ruído do processo
    float R_gps[3][3];  // Ruído de medição GPS (x, y, z)
    float R_baro;       // Ruído de medição barômetro (z)
} ROCKET_EKF;

/* Estado completo do foguete */
typedef struct {
    float dt;           // Intervalo de tempo (s)

    /* Atitude (Quaternion e Euler) */
    float q[4];         // Quaternion [w, x, y, z]
    float roll;         // Phi (rad)
    float pitch;        // Theta (rad)
    float yaw;          // Psi (rad)

    /* Posição e velocidade (NED) */
    float pos[3];       // [x, y, z] em metros
    float vel[3];       // [vx, vy, vz] em m/s
    float acc[3];       // [ax, ay, az] em m/s² (sem gravidade)

    /* Dados brutos dos sensores */
    float gps_pos[3];   // Posição GPS
    float baro_alt;     // Altitude barômetro
    float imu_acc[3];   // Aceleração IMU (body frame)
    float imu_gyro[3];  // Velocidade angular

    /* Flags de validade */
    uint8_t gps_valid;
    uint8_t baro_valid;
    uint8_t imu_valid;

} ROCKET_State;

/* ========================================================================
 * ESTRUTURAS DE TELEMETRIA
 * ======================================================================== */

/* Payload de telemetria compacto */
typedef struct __attribute__((__packed__)) ROCKET_Payload_s {
    float time;          // Timestamp em segundos
    float lat;           // Latitude (graus)
    float lon;           // Longitude (graus)
    float alt;           // Altitude GNSS (m)
    float down;          // Deslocamento vertical (m)
    float north;         // Deslocamento norte (m)
    float east;          // Deslocamento leste (m)
    float veld;          // Velocidade vertical (m/s)
    float veln;          // Velocidade norte (m/s)
    float vele;          // Velocidade leste (m/s)
    float yaw;           // Guinada (graus)
    float pitch;         // Arfagem (graus)
    float roll;          // Rolagem (graus)
    float bar_alt;       // Altitude barômetro (m)
    float acc_x;         // Aceleração X NED (m/s²)
    float acc_y;         // Aceleração Y NED (m/s²)
    float acc_z;         // Aceleração Z NED (m/s²)
    uint8_t gnss_fix;    // Status do fix GNSS
    uint8_t imu_valid;   // Status IMU
    uint8_t parachute;   // Estado do paraquedas
} ROCKET_Payload_t;

/* Timers do sistema */
typedef struct {
    uint32_t lora_tx;
    uint32_t sd_write;
    uint32_t gnss_check;
    uint32_t bar_update;
    uint32_t imu_update;
    uint32_t ekf_update;
} ROCKET_Timers_t;

/* Status do sistema */
typedef struct {
    uint32_t transmission_count;
    uint32_t sd_write_count;
    uint8_t system_ready;
    uint8_t gnss_initialized;
    uint8_t sd_initialized;
    uint8_t lora_initialized;
    uint8_t bar_initialized;
    uint8_t imu_initialized;
    uint8_t fusion_initialized;
    uint8_t ekf_initialized;
} ROCKET_Status_t;

/* Estrutura principal do sistema */
typedef struct ROCKET_System_s {
    /* Periféricos */
    GNSS *gnss;
    SD *sd;
    LORA *lora;
    BAR *bar;
    IMU *imu;
    IMU_Fusion *fusion;

    /* Fusão sensorial */
    ROCKET_EKF ekf;
    ROCKET_State state;

    /* Sistema */
    ROCKET_Timers_t timers;
    ROCKET_Status_t status;
    ROCKET_Payload_t last_payload;
    ROCKET_FSM_t *fsm;

    /* Configuração EKF */
    uint8_t use_ekf;
    float ekf_update_rate;

    /* Sistema de logging unificado */
    ROCKET_Logger_t logger;

} ROCKET_System_t;

/* ========================================================================
 * AUXILIARES
 * ======================================================================== */
typedef struct {
    float readings[10];
    uint8_t index;
    uint8_t count;
    float last_valid;
} BAR_Filter_t;

typedef struct {
    ROCKET_Payload_t samples[FLIGHT_BUFFER_SIZE];
    uint16_t write_idx;
    uint16_t read_idx;
    uint16_t count;
} FlightDataBuffer_t;

/* ========================================================================
 * FUNÇÕES PRINCIPAIS
 * ======================================================================== */

void ROCKET_Init(ROCKET_System_t *rocket, GNSS *gnss, SD *sd, LORA *lora,
                 BAR *bar, IMU *imu, IMU_Fusion *fusion, ROCKET_FSM_t *fsm);

HAL_StatusTypeDef ROCKET_BeginPeripherals(ROCKET_System_t *rocket,
                                          UART_HandleTypeDef *huart_lora,
                                          UART_HandleTypeDef *huart_gnss,
                                          I2C_HandleTypeDef *hi2c_bar,
                                          SPI_HandleTypeDef *hspi_imu,
                                          GPIO_TypeDef *m0_port, uint16_t m0_pin,
                                          GPIO_TypeDef *m1_port, uint16_t m1_pin,
                                          GPIO_TypeDef *imu_cs_port, uint16_t imu_cs_pin);

void ROCKET_Manager(ROCKET_System_t *rocket);

/* ========================================================================
 * FUNÇÕES DE FUSÃO SENSORIAL (EKF)
 * ======================================================================== */

void ROCKET_InitEKF_PreFlight(ROCKET_System_t *rocket);

/**
 * @brief Inicializa o EKF com parâmetros iniciais
 */
void ROCKET_EKF_Init(ROCKET_EKF *ekf, float dt);


/**
 * @brief Força a matriz de covariância P a ser simétrica.
 * P = (P + P^T) / 2
 * Isso previne a divergência numérica do filtro devido a erros de
 * arredondamento de ponto flutuante.
 */
void ROCKET_EKF_EnforceSymmetry(ROCKET_EKF *ekf);

/**
 * @brief Passo de predição usando acelerações do IMU
 * @param ekf Ponteiro para estrutura EKF
 * @param ax, ay, az Acelerações em m/s² (referencial NED, sem gravidade)
 */
void ROCKET_EKF_Predict(ROCKET_EKF *ekf, float ax, float ay, float az);

/**
 * @brief Atualização com medição do GPS
 * @param ekf Ponteiro para estrutura EKF
 * @param gps_x, gps_y, gps_z Posição GPS em metros (NED)
 */
void ROCKET_EKF_UpdateGPS(ROCKET_EKF *ekf, float gps_x, float gps_y, float gps_z);

/**
 * @brief Atualização com medição do barômetro
 * @param ekf Ponteiro para estrutura EKF
 * @param baro_z Altitude do barômetro em metros
 */
void ROCKET_EKF_UpdateBaro(ROCKET_EKF *ekf, float baro_z);

/**
 * @brief Atualiza o estado completo do foguete com fusão sensorial
 */
void ROCKET_UpdateState(ROCKET_System_t *rocket);

/**
 * @brief Converte coordenadas GPS para NED relativo à posição inicial
 */
void ROCKET_GPS_ToNED(ROCKET_System_t *rocket, double lat, double lon, float alt,
                      float *ned_x, float *ned_y, float *ned_z);

/* ========================================================================
 * FUNÇÕES DE TELEMETRIA
 * ======================================================================== */

HAL_StatusTypeDef ROCKET_SendTelemetry(ROCKET_System_t *rocket);
void ROCKET_BuildPayload(ROCKET_System_t *rocket, ROCKET_Payload_t *payload);

/* ========================================================================
 * FUNÇÕES DE GRAVAÇÃO
 * ======================================================================== */

//HAL_StatusTypeDef ROCKET_LogToSD(ROCKET_System_t *rocket);
//HAL_StatusTypeDef ROCKET_CreateLogFile(ROCKET_System_t *rocket, const char *filename);

/* ========================================================================
 * FUNÇÕES DE ATUALIZAÇÃO DE SENSORES
 * ======================================================================== */

void ROCKET_UpdateBarometer(ROCKET_System_t *rocket);
void ROCKET_UpdateIMU(ROCKET_System_t *rocket);
void ROCKET_CheckGNSS(ROCKET_System_t *rocket);

/* ========================================================================
 * FUNÇÕES DE CALIBRAÇÃO
 * ======================================================================== */

HAL_StatusTypeDef ROCKET_CalibrateIMU(ROCKET_System_t *rocket, uint16_t samples);
void ROCKET_SetEKFNoiseParameters(ROCKET_System_t *rocket,
                                  float q_pos, float q_vel,
                                  float r_gps, float r_baro);
void ROCKET_AutoCalibrateOnPad(ROCKET_System_t *rocket);

/* ========================================================================
 * SISTEMA DE GRAVAÇÃO
 * ======================================================================== */

/**
 * @brief Inicializa o sistema de logging
 */
HAL_StatusTypeDef ROCKET_Logger_Init(ROCKET_System_t *rocket);

/**
 * @brief Cria arquivo de log com cabeçalho completo
 * @param filename Nome do arquivo (NULL = nome automático com timestamp)
 */
HAL_StatusTypeDef ROCKET_Logger_CreateFile(ROCKET_System_t *rocket, const char *filename);

/**
 * @brief Adiciona amostra ao buffer (chamada a cada ciclo de aquisição)
 */
HAL_StatusTypeDef ROCKET_Logger_AddSample(ROCKET_System_t *rocket);

/**
 * @brief Grava buffer no SD (inteligente - só grava quando necessário)
 */
HAL_StatusTypeDef ROCKET_Logger_Flush(ROCKET_System_t *rocket);

/**
 * @brief Força flush imediato (usar em momentos críticos)
 */
HAL_StatusTypeDef ROCKET_Logger_ForceFlush(ROCKET_System_t *rocket);

/**
 * @brief Fecha arquivo e salva estatísticas
 */
HAL_StatusTypeDef ROCKET_Logger_Close(ROCKET_System_t *rocket);

/**
 * @brief Imprime estatísticas do logger
 */
void ROCKET_Logger_PrintStats(ROCKET_System_t *rocket);

/**
 * @brief Converte LogData para formato CSV
 */
void ROCKET_Logger_DataToCSV(ROCKET_LogData_t *data, char *buffer, size_t size);


/* ========================================================================
 * FUNÇÕES AUXILIARES
 * ======================================================================== */
void ROCKET_AdjustTimingsForPhase(ROCKET_System_t *rocket, FlightState_t phase);
uint8_t ROCKET_DetectApogee_Redundant(ROCKET_System_t *rocket, ROCKET_FSM_t *fsm);
void ROCKET_PrintInfo(ROCKET_System_t *rocket);
void ROCKET_PrintState(ROCKET_System_t *rocket);
uint8_t ROCKET_CheckPeripherals(ROCKET_System_t *rocket);
void ROCKET_SetTransmissionInterval(ROCKET_System_t *rocket, uint32_t interval_ms);
void ROCKET_SetSDWriteInterval(ROCKET_System_t *rocket, uint32_t interval_ms);
void ROCKET_EnableEKF(ROCKET_System_t *rocket, uint8_t enable);
HAL_StatusTypeDef ROCKET_InjectGPSAidData(ROCKET_System_t *rocket,
                                          double lat, double lon, float alt,
                                          uint16_t year, uint8_t month, uint8_t day,
                                          uint8_t hour, uint8_t minute, uint8_t second);
HAL_StatusTypeDef ROCKET_CollectMagCalibrationData(ROCKET_System_t *rocket,
                                                    uint32_t duration_ms);

/**
 * @brief Grava dados do EKF no SD para análise offline
 */
//HAL_StatusTypeDef ROCKET_LogEKFToSD(ROCKET_System_t *rocket);

/**
 * @brief Cria arquivo de log EKF com cabeçalho
 */
HAL_StatusTypeDef ROCKET_CreateEKFLogFile(ROCKET_System_t *rocket, const char *filename);

/**
 * @brief Envia dados do EKF via printf para análise em tempo real
 */
void ROCKET_PrintEKFForMatlab(ROCKET_System_t *rocket);

/* ========================================================================
 * AUXILIARES
 * ======================================================================== */
void GNSS_SetFlightMode(GNSS *gnss, FlightState_t phase);
void BAR_SetModeForPhase(BAR *bar, FlightState_t phase);
float BAR_GetFilteredAltitude(BAR *bar, BAR_Filter_t *filter, ROCKET_FSM_t *fsm);

/* ========================================================================
 * FUNÇÕES DEPRECIADAS (manter compatibilidade)
 * ======================================================================== */

// Estas funções agora apenas chamam o novo sistema
HAL_StatusTypeDef ROCKET_LogToSD(ROCKET_System_t *rocket) __attribute__((deprecated));
HAL_StatusTypeDef ROCKET_CreateLogFile(ROCKET_System_t *rocket, const char *filename) __attribute__((deprecated));
HAL_StatusTypeDef ROCKET_LogEKFToSD(ROCKET_System_t *rocket) __attribute__((deprecated));

#endif /* INC_ROCKET_H_ */
