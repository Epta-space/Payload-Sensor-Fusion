/*
 * LORA.c
 *
 *  Created on: Dec 30, 2024
 *      Author: de4lerr
 *  Description: Driver para módulo LoRa E22-900T22D
 */

#include "LORA.h"
#include <stdarg.h>

/* Ponteiro global para instância ativa */
LORA *g_active_lora = NULL;

/* ========================================================================
 * FUNÇÕES DE INICIALIZAÇÃO
 * ======================================================================== */

void LORA_Init(LORA *lora)
{
    memset(lora, 0, sizeof(LORA));
    lora->status = LORA_STATUS_NOT_INIT;
    lora->mode = LORA_MODE_TRANSCEIVER;
    lora->is_initialized = 0;
    lora->rx_old_pos = 0;
    lora->rx_data_ready = 0;
    lora->packet_received_callback = NULL;

    lora->tx_in_progress = 0;
    lora->tx_complete = 1;
    lora->tx_timeout_ms = 2000;  // Timeout padrão: 1/2 segundo
}

HAL_StatusTypeDef LORA_Begin(LORA *lora,
                             UART_HandleTypeDef *huart,
                             GPIO_TypeDef *m0_port, uint16_t m0_pin,
                             GPIO_TypeDef *m1_port, uint16_t m1_pin)
{
    if (lora == NULL || huart == NULL) {
        return HAL_ERROR;
    }

    /* Salva configuração */
    lora->huart = huart;
    lora->m0_port = m0_port;
    lora->m0_pin = m0_pin;
    lora->m1_port = m1_port;
    lora->m1_pin = m1_pin;

    /* Define instância global para callbacks */
    g_active_lora = lora;

    /* Configura modo transceiver */
    LORA_SetTransceiverMode(lora);
    HAL_Delay(300);

    /* Inicializa biblioteca E22 */
    e22_lora_init(huart,
                  LORA_InternalTransmitCallback,
                  LORA_InternalStartReceptionCallback,
                  LORA_InternalPacketHandlerCallback,
                  LORA_InternalSetConfigModeCallback,
                  LORA_InternalSetTransceiverModeCallback);

    /* Inicia recepção contínua */
    LORA_StartReception(lora);

    lora->status = LORA_STATUS_READY;
    lora->is_initialized = 1;

    return HAL_OK;
}

void LORA_SetAddress(LORA *lora, uint16_t local_addr, uint16_t dest_addr)
{
    lora->local_address = local_addr;
    lora->destination_address = dest_addr;
}

void LORA_SetChannel(LORA *lora, uint8_t channel)
{
    lora->channel = channel;
}

void LORA_SetPacketCallback(LORA *lora, void (*callback)(uint8_t*, uint8_t))
{
    lora->packet_received_callback = callback;
}

/* ========================================================================
 * FUNÇÕES DE CONTROLE DE MODO
 * ======================================================================== */

void LORA_SetTransceiverMode(LORA *lora)
{
    HAL_GPIO_WritePin(lora->m0_port, lora->m0_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lora->m1_port, lora->m1_pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    lora->mode = LORA_MODE_TRANSCEIVER;
}

void LORA_SetConfigMode(LORA *lora)
{
    HAL_GPIO_WritePin(lora->m0_port, lora->m0_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lora->m1_port, lora->m1_pin, GPIO_PIN_SET);
    HAL_Delay(50);
    lora->mode = LORA_MODE_CONFIG;
}

/* ========================================================================
 * FUNÇÕES DE TRANSMISSÃO
 * ======================================================================== */

HAL_StatusTypeDef LORA_SendString(LORA *lora, const char *str)
{
    if (!lora->is_initialized || str == NULL) {
        return HAL_ERROR;
    }

    return LORA_SendStringTo(lora, str, lora->destination_address, lora->channel);
}

HAL_StatusTypeDef LORA_SendData(LORA *lora, uint8_t *data, uint16_t size)
{
    if (!lora->is_initialized || data == NULL) {
        return HAL_ERROR;
    }

    return LORA_SendDataTo(lora, data, size, lora->destination_address, lora->channel);
}

HAL_StatusTypeDef LORA_SendFormatted(LORA *lora, const char *format, ...)
{
    if (!lora->is_initialized || format == NULL) {
        return HAL_ERROR;
    }

    char buffer[LORA_MAX_PACKET_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    return LORA_SendString(lora, buffer);
}

HAL_StatusTypeDef LORA_SendStruct(LORA *lora, void *struct_ptr, uint16_t struct_size)
{
    if (!lora->is_initialized || struct_ptr == NULL) {
        return HAL_ERROR;
    }

    return LORA_SendStructTo(lora, struct_ptr, struct_size,
                            lora->destination_address, lora->channel);
}

HAL_StatusTypeDef LORA_SendInt(LORA *lora, int value)
{
    if (!lora->is_initialized) {
        return HAL_ERROR;
    }

    return LORA_SendFormatted(lora, "%d", value);
}

HAL_StatusTypeDef LORA_SendFloat(LORA *lora, float value)
{
    if (!lora->is_initialized) {
        return HAL_ERROR;
    }

    return LORA_SendFormatted(lora, "%.6f", value);
}

HAL_StatusTypeDef LORA_SendDouble(LORA *lora, double value)
{
    if (!lora->is_initialized) {
        return HAL_ERROR;
    }

    return LORA_SendFormatted(lora, "%.6lf", value);
}

/* ========================================================================
 * FUNÇÕES DE TRANSMISSÃO COM DESTINO ESPECÍFICO
 * ======================================================================== */

HAL_StatusTypeDef LORA_SendStringTo(LORA *lora, const char *str,
                                    uint16_t dest_addr, uint8_t channel)
{
    if (!lora->is_initialized || str == NULL) {
        return HAL_ERROR;
    }

    size_t len = strlen(str);
    if (len > LORA_MAX_PACKET_SIZE) {
        len = LORA_MAX_PACKET_SIZE;
    }

    return LORA_SendDataTo(lora, (uint8_t*)str, len, dest_addr, channel);
}

HAL_StatusTypeDef LORA_SendDataTo(LORA *lora, uint8_t *data, uint16_t size,
                                  uint16_t dest_addr, uint8_t channel)
{
    if (!lora->is_initialized || data == NULL || size == 0) {
        return HAL_ERROR;
    }

    if (size > LORA_MAX_PACKET_SIZE) {
        size = LORA_MAX_PACKET_SIZE;
        printf("⚠️ Pacote truncado para %d bytes\r\n", LORA_MAX_PACKET_SIZE);
    }

    // Verifica se há transmissão em andamento
    if (lora->tx_in_progress) {
        lora->error_count++;
        return HAL_BUSY;  // Retorna BUSY em vez de bloquear
    }

    // Marca transmissão como iniciada
    lora->tx_in_progress = 1;
    lora->tx_complete = 0;
    lora->status = LORA_STATUS_TRANSMITTING;

    // Garante que o sistema está pronto
    e22_lora_make_ready();

    // Enfileira o pacote para transmissão
    e22_lora_transnit(data, size, dest_addr, channel);

    // Atualiza estatísticas
    lora->tx_count++;
    lora->last_tx_time = HAL_GetTick();

    // apenas chama o manager uma vez
    e22_lora_manager();

    return HAL_OK;
}

HAL_StatusTypeDef LORA_SendStructTo(LORA *lora, void *struct_ptr,
                                    uint16_t struct_size,
                                    uint16_t dest_addr, uint8_t channel)
{
    if (!lora->is_initialized || struct_ptr == NULL || struct_size == 0) {
        return HAL_ERROR;
    }

    if (struct_size > LORA_MAX_PACKET_SIZE) {
        struct_size = LORA_MAX_PACKET_SIZE;
        printf("Mensagem maior que o suportado!!!!!!\r\n");
    }

    return LORA_SendDataTo(lora, (uint8_t*)struct_ptr, struct_size,
                          dest_addr, channel);
}

/* ========================================================================
 * FUNÇÕES DE RECEPÇÃO
 * ======================================================================== */

void LORA_StartReception(LORA *lora)
{
    if (!lora->is_initialized) {
        return;
    }

    /* Inicia recepção DMA contínua em modo circular */
    lora->rx_old_pos = 0;
    HAL_UART_Receive_DMA(lora->huart, lora->rx_dma_buffer, LORA_RX_BUFFER_SIZE);

    /* Habilita interrupção IDLE */
    __HAL_UART_ENABLE_IT(lora->huart, UART_IT_IDLE);

    lora->status = LORA_STATUS_RECEIVING;
}

void LORA_HandleIdleInterrupt(LORA *lora)
{
    if (!lora->is_initialized) {
        return;
    }

    /* Calcula quantos bytes foram recebidos */
    uint16_t current_pos = LORA_RX_BUFFER_SIZE -
                          __HAL_DMA_GET_COUNTER(lora->huart->hdmarx);

    if (current_pos != lora->rx_old_pos) {
        uint16_t data_size;

        if (current_pos > lora->rx_old_pos) {
            /* Dados não deram wrap */
            data_size = current_pos - lora->rx_old_pos;
            memcpy(lora->rx_process_buffer,
                   &lora->rx_dma_buffer[lora->rx_old_pos],
                   data_size);
        } else {
            /* Dados deram wrap no buffer circular */
            uint16_t first_part = LORA_RX_BUFFER_SIZE - lora->rx_old_pos;
            memcpy(lora->rx_process_buffer,
                   &lora->rx_dma_buffer[lora->rx_old_pos],
                   first_part);
            memcpy(&lora->rx_process_buffer[first_part],
                   lora->rx_dma_buffer,
                   current_pos);
            data_size = first_part + current_pos;
        }

        lora->rx_process_size = data_size;
        lora->rx_old_pos = current_pos;
        lora->rx_data_ready = 1;

        /* Atualiza estatísticas */
        lora->rx_count++;
        lora->last_rx_time = HAL_GetTick();
    }
}

void LORA_ProcessReceived(LORA *lora)
{
    if (!lora->is_initialized) {
        return;
    }

    if (lora->rx_data_ready) {
        lora->rx_data_ready = 0;

        /* Envia dados para a biblioteca E22 */
        e22_lora_receive(lora->rx_process_buffer, lora->rx_process_size);
    }
}

/* ========================================================================
 * FUNÇÃO DE GERENCIAMENTO
 * ======================================================================== */

void LORA_Manager(LORA *lora)
{
    if (!lora->is_initialized) {
        return;
    }

    /* Processa dados recebidos */
    LORA_ProcessReceived(lora);

    /* Gerencia a biblioteca E22 */
    e22_lora_manager();
}

/* ========================================================================
 * FUNÇÕES AUXILIARES
 * ======================================================================== */

void LORA_MakeReady(LORA *lora)
{
    if (!lora->is_initialized) {
        return;
    }

    e22_lora_make_ready();

    // Marca TX como completa
    lora->tx_complete = 1;
    lora->tx_in_progress = 0;
    lora->status = LORA_STATUS_READY;
}

void LORA_PrintInfo(LORA *lora)
{
    printf("\n========== INFORMACOES DO LORA ==========\n");

    printf("\n--- STATUS ---\n");
    printf("Status: ");
    switch(lora->status) {
        case LORA_STATUS_NOT_INIT:     printf("Nao Inicializado\n"); break;
        case LORA_STATUS_READY:        printf("Pronto\n"); break;
        case LORA_STATUS_TRANSMITTING: printf("Transmitindo\n"); break;
        case LORA_STATUS_RECEIVING:    printf("Recebendo\n"); break;
        case LORA_STATUS_ERROR:        printf("Erro\n"); break;
        default:                       printf("Desconhecido\n"); break;
    }

    printf("Inicializado: %s\n", lora->is_initialized ? "Sim" : "Nao");
    printf("Modo: %s\n", lora->mode == LORA_MODE_TRANSCEIVER ?
           "Transceiver" : "Config");

    printf("\n--- CONFIGURACAO ---\n");
    printf("Endereco Local: 0x%04X\n", lora->local_address);
    printf("Endereco Destino: 0x%04X\n", lora->destination_address);
    printf("Canal: %d\n", lora->channel);

    printf("\n--- ESTATISTICAS ---\n");
    printf("Pacotes Transmitidos: %lu\n", lora->tx_count);
    printf("Pacotes Recebidos: %lu\n", lora->rx_count);
    printf("Erros: %lu\n", lora->error_count);
    printf("Ultima Transmissao: %lu ms\n", lora->last_tx_time);
    printf("Ultima Recepcao: %lu ms\n", lora->last_rx_time);

    printf("\n--- RECEPCAO ---\n");
    printf("Buffer RX Ocupado: %d/%d bytes\n",
           lora->rx_old_pos, LORA_RX_BUFFER_SIZE);
    printf("Dados Prontos: %s\n", lora->rx_data_ready ? "Sim" : "Nao");
    printf("Callback Registrado: %s\n",
           lora->packet_received_callback ? "Sim" : "Nao");

    printf("\n=========================================\n\n");
}

/* ========================================================================
 * Funções de controle assíncrono
 * ======================================================================== */

/**
 * @brief Aguarda conclusão da TX processando outras tarefas
 * @param timeout_ms Timeout máximo em ms
 * @return HAL_OK se completo, HAL_TIMEOUT se expirou, HAL_ERROR se erro
 */
HAL_StatusTypeDef LORA_WaitTxComplete(LORA *lora, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (lora->tx_in_progress) {
        // Processa manager (não bloqueante)
        e22_lora_manager();

        // Verifica timeout
        if ((HAL_GetTick() - start) > timeout_ms) {
            lora->tx_in_progress = 0;
            lora->status = LORA_STATUS_ERROR;
            return HAL_TIMEOUT;
        }

        // Verifica se completou
        if (lora->tx_complete) {
            lora->tx_complete = 0;
            lora->tx_in_progress = 0;
            lora->status = LORA_STATUS_READY;
            return HAL_OK;
        }

        // ✅ Yield mínimo - permite outras interrupções
        __NOP();
    }

    return HAL_OK;
}

/**
 * @brief Verifica se a transmissão foi concluída
 * @return 1 se completa, 0 se ainda em andamento, -1 se timeout
 */
int8_t LORA_IsTxComplete(LORA *lora)
{
    if (!lora->is_initialized) {
        return -1;
        //printf("caso 1");
    }

    // Verifica timeout
    if (lora->tx_in_progress) {
        uint32_t elapsed = HAL_GetTick() - lora->last_tx_time;
        if (elapsed > lora->tx_timeout_ms) {
            lora->tx_in_progress = 0;
            lora->status = LORA_STATUS_ERROR;
            lora->error_count++;
            //printf("caso 2");
            return -1;  // Timeout
        }
    }

    // Retorna status
    if (lora->tx_complete) {
        lora->tx_complete = 0;
        lora->tx_in_progress = 0;
        lora->status = LORA_STATUS_READY;
        //printf("caso 3");
        return 1;  // Completo
    }

    //printf("caso 4");

    return 0;  // Ainda em andamento
}

void LORA_SetTxTimeout(LORA *lora, uint32_t timeout_ms)
{
    if(lora && lora->is_initialized) {
        lora->tx_timeout_ms = timeout_ms;
    }
}

/* ========================================================================
 * CALLBACKS INTERNOS
 * ======================================================================== */

void LORA_InternalTransmitCallback(void *huart, const uint8_t *pData, uint16_t size)
{
    HAL_UART_Transmit_DMA((UART_HandleTypeDef*)huart, pData, size);
}

void LORA_InternalStartReceptionCallback(void *huart, uint8_t *pData, uint16_t size)
{
    /* Não usado - recepção é contínua via DMA + IDLE */
}

void LORA_InternalPacketHandlerCallback(uint8_t *pData, uint8_t size)
{
    if (g_active_lora && g_active_lora->packet_received_callback) {
        g_active_lora->packet_received_callback(pData, size);
    }
}

void LORA_InternalSetConfigModeCallback(void)
{
    if (g_active_lora) {
        LORA_SetConfigMode(g_active_lora);
    }
}

void LORA_InternalSetTransceiverModeCallback(void)
{
    if (g_active_lora) {
        LORA_SetTransceiverMode(g_active_lora);
    }
}
