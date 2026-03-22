/*
 * SD.h
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Driver para gravação em cartão SD via SPI usando FatFS
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "main.h"
#include "app_fatfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* Status do SD Card */
typedef enum {
    SD_STATUS_NOT_INIT = 0,
    SD_STATUS_NO_CARD,
    SD_STATUS_MOUNTED,
    SD_STATUS_ERROR
} SD_Status;

/* Formato de arquivo */
typedef enum {
    SD_FORMAT_TXT = 0,
    SD_FORMAT_CSV,
    SD_FORMAT_JSON
} SD_Format;

/* Modo de escrita */
typedef enum {
    SD_MODE_APPEND = 0,     /* Adiciona ao final do arquivo */
    SD_MODE_OVERWRITE       /* Sobrescreve o arquivo */
} SD_WriteMode;

typedef struct SD_s {
    /* Sistema de arquivos FatFS */
    FATFS fatfs;
    FIL file;

    /* Status */
    SD_Status status;
    uint8_t is_mounted;
    uint8_t is_file_open;
    uint32_t last_write;

    /* Informações do cartão */
    uint32_t total_space_kb;
    uint32_t free_space_kb;

    /* Configurações */
    char current_filename[64];
    SD_Format format;
    SD_WriteMode write_mode;

    /* Buffer de escrita */
    char write_buffer[512];
    uint16_t buffer_size;

    /* Contadores */
    uint32_t write_count;
    uint32_t error_count;

    /* JSON */
    uint8_t json_first_entry;
    uint8_t json_object_open;

} SD;

/* Funções principais */
void SD_Init(SD *sd);
HAL_StatusTypeDef SD_Mount(SD *sd);
HAL_StatusTypeDef SD_Unmount(SD *sd);
void SD_GetCardInfo(SD *sd);

/* Funções de arquivo */
HAL_StatusTypeDef SD_OpenFile(SD *sd, const char *filename, SD_WriteMode mode);
HAL_StatusTypeDef SD_CloseFile(SD *sd);
HAL_StatusTypeDef SD_DeleteFile(SD *sd, const char *filename);
HAL_StatusTypeDef SD_FileExists(SD *sd, const char *filename);

/* Funções de escrita - Texto */
HAL_StatusTypeDef SD_WriteLine(SD *sd, const char *line);
HAL_StatusTypeDef SD_WriteFormatted(SD *sd, const char *format, ...);
HAL_StatusTypeDef SD_WriteData(SD *sd, const char *data, uint16_t length);

/* Funções de escrita - CSV */
HAL_StatusTypeDef SD_WriteCSVHeader(SD *sd, const char *header);
HAL_StatusTypeDef SD_WriteCSVLine(SD *sd, const char *format, ...);

/* Funções de escrita - JSON */
HAL_StatusTypeDef SD_JSONStart(SD *sd);
HAL_StatusTypeDef SD_JSONEnd(SD *sd);
HAL_StatusTypeDef SD_JSONStartObject(SD *sd);
HAL_StatusTypeDef SD_JSONEndObject(SD *sd);
HAL_StatusTypeDef SD_JSONAddString(SD *sd, const char *key, const char *value);
HAL_StatusTypeDef SD_JSONAddNumber(SD *sd, const char *key, double value);
HAL_StatusTypeDef SD_JSONAddInt(SD *sd, const char *key, int value);
HAL_StatusTypeDef SD_JSONAddBool(SD *sd, const char *key, uint8_t value);

/* Funções de leitura */
HAL_StatusTypeDef SD_ReadLine(SD *sd, char *buffer, uint16_t buffer_size);
HAL_StatusTypeDef SD_ReadFile(SD *sd, const char *filename, char *buffer, uint32_t buffer_size);

/* Funções auxiliares */
void SD_Sync(SD *sd);
void SD_PrintInfo(SD *sd);

/* Funções de alto nível - Facilita uso */
HAL_StatusTypeDef SD_QuickWriteLine(SD *sd, const char *filename, const char *line);
HAL_StatusTypeDef SD_QuickWriteFormatted(SD *sd, const char *filename, const char *format, ...);

#endif /* INC_SD_H_ */
