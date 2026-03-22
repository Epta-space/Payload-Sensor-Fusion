/*
 * GNSS.h
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Updated: Suporte completo para GNSS (GPS, GLONASS, Galileo, BeiDou)
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Tipos de mensagens NMEA suportadas */
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_RMC,
    NMEA_GGA,
    NMEA_GSA,
    NMEA_GSV,
    NMEA_VTG,
    NMEA_GLL
} NMEA_MessageType;

/* Tipos de sistemas GNSS */
typedef enum {
    GNSS_GPS = 0,
    GNSS_GLONASS,
    GNSS_GALILEO,
    GNSS_BEIDOU,
    GNSS_COMBINED,
    GNSS_UNKNOWN
} GNSS_System;

/* Modo de fix */
typedef enum {
    FIX_NONE = 0,
    FIX_2D,
    FIX_3D
} GNSS_FixMode;

/* Qualidade do fix */
typedef enum {
    QUALITY_INVALID = 0,
    QUALITY_GPS_FIX,
    QUALITY_DGPS_FIX,
    QUALITY_PPS_FIX,
    QUALITY_RTK_FIX,
    QUALITY_RTK_FLOAT,
    QUALITY_ESTIMATED,
    QUALITY_MANUAL,
    QUALITY_SIMULATION
} GNSS_Quality;

/* Estrutura para dados de auxílio */
typedef struct {
    double lat;
    double lon;
    float alt;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} GNSS_AidData_t;

typedef struct {
    char GNSS_Data[256];

    /* Buffer de recepção DMA */
    uint8_t rx_buffer[512];
    uint16_t rx_size;

    /* Parser de linha */
    char line_buffer[512];
    uint16_t line_pos;
    uint8_t line_ready;

    /* Informações do sistema */
    GNSS_System system;
    NMEA_MessageType msg_type;
    char talker_id[3];

    /* RMC - Recommended Minimum */
    char tipo[7];
    char hora[11];
    char status;
    double lat;
    char lat_dir;
    double lon;
    char lon_dir;
    double velocidade;
    double angulo;
    char data[7];
    double variacao_magnetica;
    char direcao_variacao;

    /* GGA - Fix Data */
    GNSS_Quality quality;
    uint8_t satelites_usados;
    double hdop;
    double altitude;
    double geoidal_sep;

    /* GSA - DOP and Active Satellites */
    char modo_fixo;
    GNSS_FixMode fix_mode;
    uint8_t satelites_ids[12];
    double pdop;
    double vdop;

    /* Conversões úteis */
    double lat_decimal;
    double lon_decimal;
    double velocidade_kmh;

    /* Informações de validade */
    uint8_t is_valid;
    uint8_t is_fixed;
    uint32_t last_update;

    /* Posição inicial (após primeiro fix) */
    uint8_t has_initial_pos;
    double lat_inicial;
    double lon_inicial;
    double alt_inicial;

    /* Deslocamento em relação à posição inicial */
    double distancia_total;
    double deslocamento_x;
    double deslocamento_y;
    double deslocamento_z;

} GNSS;

/* Funções principais */
void GNSS_Init(UART_HandleTypeDef uart, GNSS *gnss);
HAL_StatusTypeDef GNSS_Configure(GNSS *gnss);
HAL_StatusTypeDef GNSS_Begin(GNSS *gnss, UART_HandleTypeDef *uart);
HAL_StatusTypeDef GNSS_StartDMA(GNSS *gnss);
void GNSS_ProcessData(GNSS *gnss, uint16_t size);
void GNSS_ParseLine(GNSS *gnss);

/* Funções de parsing */
void parseNMEA(char *msg, GNSS *gnss);
void parseRMC(char *msg, GNSS *gnss);
void parseGGA(char *msg, GNSS *gnss);
void parseGSA(char *msg, GNSS *gnss);

/* Funções auxiliares */
GNSS_System GNSS_GetSystemFromTalker(const char *talker);
NMEA_MessageType GNSS_GetMessageType(const char *sentence);
uint8_t GNSS_ValidateChecksum(const char *sentence);
void GNSS_ConvertToDecimal(GNSS *gnss);
double GNSS_ConvertCoordinate(double nmea_coord);

HAL_StatusTypeDef GNSS_InjectAidData(GNSS *gnss, double lat, double lon, float alt,
                                     uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t minute, uint8_t second);

HAL_StatusTypeDef GNSS_SetHotStart(GNSS *gnss);
HAL_StatusTypeDef GNSS_SetWarmStart(GNSS *gnss);
HAL_StatusTypeDef GNSS_SetColdStart(GNSS *gnss);
HAL_StatusTypeDef GNSS_SetFullColdStart(GNSS *gnss);
HAL_StatusTypeDef GNSS_SendCommand(UART_HandleTypeDef *uart, const char *cmd);

/* Funções de posição e deslocamento */
void GNSS_SaveInitialPosition(GNSS *gnss);
void GNSS_CalculateDisplacement(GNSS *gnss);
void GNSS_CheckFixStatus(GNSS *gnss);

/* Funções úteis */
double GNSS_CalculateDistance(double lat1, double lon1, double lat2, double lon2);
double GNSS_CalculateBearing(double lat1, double lon1, double lat2, double lon2);
void GNSS_PrintInfo(GNSS *gnss);
void GNSS_PrintDiagnostics(GNSS *gnss);

#endif /* INC_GNSS_H_ */
