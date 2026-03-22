/*
 * LORA.h
 *
 *  Created on: Dec 30, 2024
 *      Author: de4lerr
 *  Description: Driver para módulo LoRa E22-900T22D
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"
#include "e22900t22d.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Defines */
#define LORA_RX_BUFFER_SIZE     256
#define LORA_MAX_PACKET_SIZE    240

/* Status do LoRa */
typedef enum {
    LORA_STATUS_NOT_INIT = 0,
    LORA_STATUS_READY,
    LORA_STATUS_TRANSMITTING,
    LORA_STATUS_RECEIVING,
    LORA_STATUS_ERROR
} LORA_Status;

/* Modo de operação */
typedef enum {
    LORA_MODE_TRANSCEIVER = 0,
    LORA_MODE_CONFIG
} LORA_Mode;

typedef struct {
    /* Handler UART */
    UART_HandleTypeDef *huart;

    /* Pinos de controle */
    GPIO_TypeDef *m0_port;
    uint16_t m0_pin;
    GPIO_TypeDef *m1_port;
    uint16_t m1_pin;

    /* Status */
    LORA_Status status;
    LORA_Mode mode;
    uint8_t is_initialized;

    /* Configuração */
    uint16_t local_address;
    uint16_t destination_address;
    uint8_t channel;

    /* Buffers DMA para recepção contínua */
    uint8_t rx_dma_buffer[LORA_RX_BUFFER_SIZE];
    uint8_t rx_process_buffer[LORA_RX_BUFFER_SIZE];
    volatile uint16_t rx_old_pos;
    volatile uint16_t rx_process_size;
    volatile uint8_t rx_data_ready;

    /* Controle assíncrono de TX */
    volatile uint8_t tx_in_progress;  // Flag: transmissão em andamento
    volatile uint8_t tx_complete;     // Flag: transmissão completa
    uint32_t tx_timeout_ms;           // Timeout de segurança

    /* Estatísticas */
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint32_t last_tx_time;
    uint32_t last_rx_time;

    /* Callback para pacotes recebidos */
    void (*packet_received_callback)(uint8_t *data, uint8_t size);

} LORA;

/* Funções principais */
void LORA_Init(LORA *lora);
HAL_StatusTypeDef LORA_Begin(LORA *lora,
                             UART_HandleTypeDef *huart,
                             GPIO_TypeDef *m0_port, uint16_t m0_pin,
                             GPIO_TypeDef *m1_port, uint16_t m1_pin);
void LORA_SetAddress(LORA *lora, uint16_t local_addr, uint16_t dest_addr);
void LORA_SetChannel(LORA *lora, uint8_t channel);
void LORA_SetPacketCallback(LORA *lora, void (*callback)(uint8_t*, uint8_t));

/* Funções de controle de modo */
void LORA_SetTransceiverMode(LORA *lora);
void LORA_SetConfigMode(LORA *lora);

/* Funções de transmissão */
HAL_StatusTypeDef LORA_SendString(LORA *lora, const char *str);
HAL_StatusTypeDef LORA_SendData(LORA *lora, uint8_t *data, uint16_t size);
HAL_StatusTypeDef LORA_SendFormatted(LORA *lora, const char *format, ...);
HAL_StatusTypeDef LORA_SendStruct(LORA *lora, void *struct_ptr, uint16_t struct_size);
HAL_StatusTypeDef LORA_SendInt(LORA *lora, int value);
HAL_StatusTypeDef LORA_SendFloat(LORA *lora, float value);
HAL_StatusTypeDef LORA_SendDouble(LORA *lora, double value);

/* Funções de transmissão com destino específico */
HAL_StatusTypeDef LORA_SendStringTo(LORA *lora, const char *str, uint16_t dest_addr, uint8_t channel);
HAL_StatusTypeDef LORA_SendDataTo(LORA *lora, uint8_t *data, uint16_t size, uint16_t dest_addr, uint8_t channel);
HAL_StatusTypeDef LORA_SendStructTo(LORA *lora, void *struct_ptr, uint16_t struct_size, uint16_t dest_addr, uint8_t channel);

/* Funções de recepção */
void LORA_StartReception(LORA *lora);
void LORA_ProcessReceived(LORA *lora);
void LORA_HandleIdleInterrupt(LORA *lora);

/* Função de gerenciamento (deve ser chamada no loop principal) */
void LORA_Manager(LORA *lora);

/* Funções auxiliares */
void LORA_MakeReady(LORA *lora);
void LORA_PrintInfo(LORA *lora);

/* Callbacks internos (não chamar diretamente) */
void LORA_InternalTransmitCallback(void *huart, const uint8_t *pData, uint16_t size);
void LORA_InternalStartReceptionCallback(void *huart, uint8_t *pData, uint16_t size);
void LORA_InternalPacketHandlerCallback(uint8_t *pData, uint8_t size);
void LORA_InternalSetConfigModeCallback(void);
void LORA_InternalSetTransceiverModeCallback(void);

/* Funções de controle assíncrono */
int8_t LORA_IsTxComplete(LORA *lora);
HAL_StatusTypeDef LORA_WaitTxComplete(LORA *lora, uint32_t timeout_ms);
void LORA_SetTxTimeout(LORA *lora, uint32_t timeout_ms);

/* Ponteiro global para instância ativa (usado nos callbacks) */
extern LORA *g_active_lora;

#endif /* INC_LORA_H_ */
