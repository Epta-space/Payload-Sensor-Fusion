/*
 * SD.c
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Description: Driver para gravação em cartão SD via SPI usando FatFS
 */

#include "SD.h"

/* ========================================================================
 * FUNÇÕES DE INICIALIZAÇÃO E MONTAGEM
 * ======================================================================== */

void SD_Init(SD *sd)
{
    memset(sd, 0, sizeof(SD));
    sd->status = SD_STATUS_NOT_INIT;
    sd->is_mounted = 0;
    sd->is_file_open = 0;
    sd->format = SD_FORMAT_TXT;
    sd->write_mode = SD_MODE_APPEND;
    sd->json_first_entry = 1;
    sd->json_object_open = 0;
}

HAL_StatusTypeDef SD_Mount(SD *sd)
{
    FRESULT fres;

    /* Tenta montar o cartão SD */
    fres = f_mount(&sd->fatfs, "", 1);

    if (fres != FR_OK) {
        sd->status = SD_STATUS_NO_CARD;
        sd->is_mounted = 0;
        return HAL_ERROR;
    }

    sd->status = SD_STATUS_MOUNTED;
    sd->is_mounted = 1;

    /* Obtém informações do cartão */
    SD_GetCardInfo(sd);

    return HAL_OK;
}

HAL_StatusTypeDef SD_Unmount(SD *sd)
{
    /* Fecha arquivo se estiver aberto */
    if (sd->is_file_open) {
        SD_CloseFile(sd);
    }

    /* Desmonta o cartão */
    f_mount(NULL, "", 0);

    sd->status = SD_STATUS_NOT_INIT;
    sd->is_mounted = 0;

    return HAL_OK;
}

void SD_GetCardInfo(SD *sd)
{
    if (!sd->is_mounted) return;

    FATFS *pfs;
    DWORD fre_clust;

    if (f_getfree("", &fre_clust, &pfs) == FR_OK) {
        sd->total_space_kb = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
        sd->free_space_kb = (uint32_t)(fre_clust * pfs->csize * 0.5);
    }
}

/* ========================================================================
 * FUNÇÕES DE GERENCIAMENTO DE ARQUIVOS
 * ======================================================================== */

HAL_StatusTypeDef SD_OpenFile(SD *sd, const char *filename, SD_WriteMode mode)
{
    FRESULT fres;
    BYTE flags;

    if (!sd->is_mounted) {
        return HAL_ERROR;
    }

    /* Fecha arquivo anterior se estiver aberto */
    if (sd->is_file_open) {
        SD_CloseFile(sd);
    }

    /* Define flags de abertura */
    if (mode == SD_MODE_APPEND) {
        flags = FA_WRITE | FA_OPEN_APPEND;
    } else {
        flags = FA_WRITE | FA_CREATE_ALWAYS;
    }

    /* Abre o arquivo */
    fres = f_open(&sd->file, filename, flags);

    if (fres != FR_OK) {
        sd->error_count++;
        return HAL_ERROR;
    }

    sd->is_file_open = 1;
    strncpy(sd->current_filename, filename, sizeof(sd->current_filename) - 1);
    sd->current_filename[sizeof(sd->current_filename) - 1] = '\0';

    return HAL_OK;
}

HAL_StatusTypeDef SD_CloseFile(SD *sd)
{
    if (!sd->is_file_open) {
        return HAL_OK;
    }

    f_close(&sd->file);
    sd->is_file_open = 0;
    sd->current_filename[0] = '\0';

    return HAL_OK;
}

HAL_StatusTypeDef SD_DeleteFile(SD *sd, const char *filename)
{
    if (!sd->is_mounted) {
        return HAL_ERROR;
    }

    FRESULT fres = f_unlink(filename);

    return (fres == FR_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef SD_FileExists(SD *sd, const char *filename)
{
    if (!sd->is_mounted) {
        return HAL_ERROR;
    }

    FILINFO fno;
    FRESULT fres = f_stat(filename, &fno);

    return (fres == FR_OK) ? HAL_OK : HAL_ERROR;
}

/* ========================================================================
 * FUNÇÕES DE ESCRITA - TEXTO
 * ======================================================================== */

HAL_StatusTypeDef SD_WriteLine(SD *sd, const char *line)
{
    if (!sd->is_file_open || line == NULL) {
        return HAL_ERROR;
    }

    /* Escreve a linha */
    UINT bytes_written;
    size_t len = strlen(line);

    FRESULT res = f_write(&sd->file, line, len, &bytes_written);

    if (res != FR_OK || bytes_written != len) {
        sd->error_count++;
        return HAL_ERROR;
    }

    /* Adiciona quebra de linha se não houver */
    if (len == 0 || line[len - 1] != '\n') {
        f_putc('\n', &sd->file);
    }

    sd->write_count++;
    sd->last_write = HAL_GetTick();

    return HAL_OK;
}

HAL_StatusTypeDef SD_WriteFormatted(SD *sd, const char *format, ...)
{
    if (!sd->is_file_open || format == NULL) {
        return HAL_ERROR;
    }

    va_list args;
    va_start(args, format);

    vsnprintf(sd->write_buffer, sizeof(sd->write_buffer), format, args);

    va_end(args);

    return SD_WriteLine(sd, sd->write_buffer);
}

HAL_StatusTypeDef SD_WriteData(SD *sd, const char *data, uint16_t length)
{
    if (!sd->is_file_open || data == NULL) {
        return HAL_ERROR;
    }

    UINT bytes_written;
    FRESULT fres = f_write(&sd->file, data, length, &bytes_written);

    if (fres != FR_OK || bytes_written != length) {
        sd->error_count++;
        return HAL_ERROR;
    }

    sd->write_count++;
    sd->last_write = HAL_GetTick();

    return HAL_OK;
}

/* ========================================================================
 * FUNÇÕES DE ESCRITA - CSV
 * ======================================================================== */

HAL_StatusTypeDef SD_WriteCSVHeader(SD *sd, const char *header)
{
    if (!sd->is_file_open) {
        return HAL_ERROR;
    }

    sd->format = SD_FORMAT_CSV;
    return SD_WriteLine(sd, header);
}

HAL_StatusTypeDef SD_WriteCSVLine(SD *sd, const char *format, ...)
{
    if (!sd->is_file_open) {
        return HAL_ERROR;
    }

    va_list args;
    va_start(args, format);

    vsnprintf(sd->write_buffer, sizeof(sd->write_buffer), format, args);

    va_end(args);

    /* Remove quebra de linha se houver */
    size_t len = strlen(sd->write_buffer);
    if (len > 0 && sd->write_buffer[len - 1] == '\n') {
        sd->write_buffer[len - 1] = '\0';
    }

    return SD_WriteLine(sd, sd->write_buffer);
}

/* ========================================================================
 * FUNÇÕES DE ESCRITA - JSON
 * ======================================================================== */

HAL_StatusTypeDef SD_JSONStart(SD *sd)
{
    if (!sd->is_file_open) {
        return HAL_ERROR;
    }

    sd->format = SD_FORMAT_JSON;
    sd->json_first_entry = 1;
    sd->json_object_open = 0;

    f_puts("[\n", &sd->file);

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONEnd(SD *sd)
{
    if (!sd->is_file_open) {
        return HAL_ERROR;
    }

    /* Fecha objeto se estiver aberto */
    if (sd->json_object_open) {
        SD_JSONEndObject(sd);
    }

    f_puts("\n]\n", &sd->file);

    sd->json_first_entry = 1;
    sd->json_object_open = 0;

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONStartObject(SD *sd)
{
    if (!sd->is_file_open) {
        return HAL_ERROR;
    }

    /* Adiciona vírgula se não for o primeiro objeto */
    if (!sd->json_first_entry) {
        f_puts(",\n", &sd->file);
    } else {
        sd->json_first_entry = 0;
    }

    f_puts("  {\n", &sd->file);
    sd->json_object_open = 1;
    sd->json_first_entry = 1; /* Reset para campos do objeto */

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONEndObject(SD *sd)
{
    if (!sd->is_file_open || !sd->json_object_open) {
        return HAL_ERROR;
    }

    f_puts("\n  }", &sd->file);
    sd->json_object_open = 0;
    sd->json_first_entry = 0; /* Próximo objeto precisará de vírgula */

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONAddString(SD *sd, const char *key, const char *value)
{
    if (!sd->is_file_open || !sd->json_object_open) {
        return HAL_ERROR;
    }

    /* Adiciona vírgula se não for o primeiro campo */
    if (!sd->json_first_entry) {
        f_puts(",\n", &sd->file);
    } else {
        sd->json_first_entry = 0;
    }

    snprintf(sd->write_buffer, sizeof(sd->write_buffer),
             "    \"%s\": \"%s\"", key, value);
    f_puts(sd->write_buffer, &sd->file);

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONAddNumber(SD *sd, const char *key, double value)
{
    if (!sd->is_file_open || !sd->json_object_open) {
        return HAL_ERROR;
    }

    if (!sd->json_first_entry) {
        f_puts(",\n", &sd->file);
    } else {
        sd->json_first_entry = 0;
    }

    snprintf(sd->write_buffer, sizeof(sd->write_buffer),
             "    \"%s\": %.6f", key, value);
    f_puts(sd->write_buffer, &sd->file);

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONAddInt(SD *sd, const char *key, int value)
{
    if (!sd->is_file_open || !sd->json_object_open) {
        return HAL_ERROR;
    }

    if (!sd->json_first_entry) {
        f_puts(",\n", &sd->file);
    } else {
        sd->json_first_entry = 0;
    }

    snprintf(sd->write_buffer, sizeof(sd->write_buffer),
             "    \"%s\": %d", key, value);
    f_puts(sd->write_buffer, &sd->file);

    return HAL_OK;
}

HAL_StatusTypeDef SD_JSONAddBool(SD *sd, const char *key, uint8_t value)
{
    if (!sd->is_file_open || !sd->json_object_open) {
        return HAL_ERROR;
    }

    if (!sd->json_first_entry) {
        f_puts(",\n", &sd->file);
    } else {
        sd->json_first_entry = 0;
    }

    snprintf(sd->write_buffer, sizeof(sd->write_buffer),
             "    \"%s\": %s", key, value ? "true" : "false");
    f_puts(sd->write_buffer, &sd->file);

    return HAL_OK;
}

/* ========================================================================
 * FUNÇÕES DE LEITURA
 * ======================================================================== */

HAL_StatusTypeDef SD_ReadLine(SD *sd, char *buffer, uint16_t buffer_size)
{
    if (!sd->is_file_open || buffer == NULL) {
        return HAL_ERROR;
    }

    if (f_gets(buffer, buffer_size, &sd->file) == NULL) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef SD_ReadFile(SD *sd, const char *filename, char *buffer, uint32_t buffer_size)
{
    if (!sd->is_mounted || buffer == NULL) {
        return HAL_ERROR;
    }

    FIL temp_file;
    FRESULT fres;
    UINT bytes_read;

    fres = f_open(&temp_file, filename, FA_READ);
    if (fres != FR_OK) {
        return HAL_ERROR;
    }

    fres = f_read(&temp_file, buffer, buffer_size - 1, &bytes_read);
    buffer[bytes_read] = '\0';

    f_close(&temp_file);

    return (fres == FR_OK) ? HAL_OK : HAL_ERROR;
}

/* ========================================================================
 * FUNÇÕES AUXILIARES
 * ======================================================================== */

void SD_Sync(SD *sd)
{
    if (sd->is_file_open) {
        f_sync(&sd->file);
    }
}

void SD_PrintInfo(SD *sd)
{
    printf("\n========== INFORMACOES DO SD CARD ==========\n");

    printf("\n--- STATUS ---\n");
    printf("Status: ");
    switch(sd->status) {
        case SD_STATUS_NOT_INIT: printf("Nao Inicializado\n"); break;
        case SD_STATUS_NO_CARD:  printf("Cartao Nao Encontrado\n"); break;
        case SD_STATUS_MOUNTED:  printf("Montado\n"); break;
        case SD_STATUS_ERROR:    printf("Erro\n"); break;
        default:                 printf("Desconhecido\n"); break;
    }

    printf("Montado: %s\n", sd->is_mounted ? "Sim" : "Nao");
    printf("Arquivo Aberto: %s\n", sd->is_file_open ? "Sim" : "Nao");

    if (sd->is_file_open) {
        printf("Arquivo Atual: %s\n", sd->current_filename);
    }

    if (sd->is_mounted) {
        printf("\n--- INFORMACOES DO CARTAO ---\n");
        printf("Espaco Total: %lu KB\n", sd->total_space_kb);
        printf("Espaco Livre: %lu KB\n", sd->free_space_kb);
        printf("Espaco Usado: %lu KB (%.1f%%)\n",
               sd->total_space_kb - sd->free_space_kb,
               (float)(sd->total_space_kb - sd->free_space_kb) * 100.0f / sd->total_space_kb);
    }

    printf("\n--- ESTATISTICAS ---\n");
    printf("Total de Escritas: %lu\n", sd->write_count);
    printf("Total de Erros: %lu\n", sd->error_count);
    printf("Ultima Escrita: %lu ms\n", sd->last_write);

    printf("\n--- CONFIGURACAO ---\n");
    printf("Formato: ");
    switch(sd->format) {
        case SD_FORMAT_TXT:  printf("TXT\n"); break;
        case SD_FORMAT_CSV:  printf("CSV\n"); break;
        case SD_FORMAT_JSON: printf("JSON\n"); break;
        default:             printf("Desconhecido\n"); break;
    }

    printf("Modo de Escrita: %s\n",
           sd->write_mode == SD_MODE_APPEND ? "Append" : "Overwrite");

    printf("\n============================================\n\n");
}

/* ========================================================================
 * FUNÇÕES DE ALTO NÍVEL
 * ======================================================================== */

HAL_StatusTypeDef SD_QuickWriteLine(SD *sd, const char *filename, const char *line)
{
    if (!sd->is_mounted) {
        return HAL_ERROR;
    }

    if (SD_OpenFile(sd, filename, SD_MODE_APPEND) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef result = SD_WriteLine(sd, line);

    SD_CloseFile(sd);

    return result;
}

HAL_StatusTypeDef SD_QuickWriteFormatted(SD *sd, const char *filename, const char *format, ...)
{
    if (!sd->is_mounted) {
        return HAL_ERROR;
    }

    va_list args;
    va_start(args, format);

    vsnprintf(sd->write_buffer, sizeof(sd->write_buffer), format, args);

    va_end(args);

    return SD_QuickWriteLine(sd, filename, sd->write_buffer);
}
