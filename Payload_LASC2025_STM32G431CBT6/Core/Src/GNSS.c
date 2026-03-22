/*
 * GNSS.c
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 *  Updated: Suporte completo para GNSS (GPS, GLONASS, Galileo, BeiDou)
 */

#include "main.h"
#include "GNSS.h"

UART_HandleTypeDef GNSSuart;

/* ========================================================================
 * FUNÇÕES DE INICIALIZAÇÃO E COMUNICAÇÃO
 * ======================================================================== */

void GNSS_Init(UART_HandleTypeDef uart, GNSS *gnss)
{
    GNSSuart = uart;
    memset(gnss, 0, sizeof(GNSS));
    gnss->system = GNSS_UNKNOWN;
    gnss->quality = QUALITY_INVALID;
    gnss->fix_mode = FIX_NONE;
    gnss->is_valid = 0;
    gnss->is_fixed = 0;
    gnss->has_initial_pos = 0;
    gnss->rx_size = sizeof(gnss->rx_buffer);
}

/**
 * @brief Configura GPS para melhor desempenho
 */
HAL_StatusTypeDef GNSS_Configure(GNSS *gnss)
{
    extern UART_HandleTypeDef GNSSuart;
    HAL_StatusTypeDef status;

    printf("[GNSS] Configurando L80-M39...\r\n");

    // 1. Taxa de atualização: 5Hz para voo (máximo é 10Hz)
    // Durante voo crítico, queremos dados rápidos
    status = GNSS_SendCommand(&GNSSuart, "PMTK220,200"); // 200ms = 5Hz
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // 2. Habilita TODAS as constelações
    status = GNSS_SendCommand(&GNSSuart, "PMTK353,1,1,1,0");
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // 3. CRÍTICO: Desabilita sentenças desnecessárias
    // Apenas GGA (posição) e RMC (velocidade)
    status = GNSS_SendCommand(&GNSSuart, "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // 4. NOVO: Configura modo dinâmico para alta velocidade
    // PMTK386: aviation mode (suporta até 515 m/s)
    status = GNSS_SendCommand(&GNSSuart, "PMTK386,1"); // 1 = aviation mode
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    // 5. Habilita EASY para TTFF rápido
    status = GNSS_SendCommand(&GNSSuart, "PMTK869,1,1"); // EASY ON
    if (status != HAL_OK) return status;
    HAL_Delay(100);

    printf("[GNSS] Configurado: 5Hz, Aviation Mode, EASY ON\r\n");

    __HAL_UART_ENABLE_IT(&GNSSuart, UART_IT_IDLE);
    return HAL_OK;
}


/**
 * @brief Inicia a recepção contínua via DMA com detecção de IDLE.
 * Esta função deve ser chamada uma vez para começar a escutar a UART.
 */
HAL_StatusTypeDef GNSS_StartDMA(GNSS *gnss)
{
    // Para DMA se já estiver rodando
    if (GNSSuart.hdmarx != NULL && GNSSuart.RxState != HAL_UART_STATE_READY) {
        HAL_UART_DMAStop(&GNSSuart);
        HAL_Delay(10);
    }

    // Limpa flags
    __HAL_UART_CLEAR_OREFLAG(&GNSSuart);
    __HAL_UART_CLEAR_IDLEFLAG(&GNSSuart);
    __HAL_UART_FLUSH_DRREGISTER(&GNSSuart);

    // Limpa buffers APENAS no início
    memset(gnss->rx_buffer, 0, sizeof(gnss->rx_buffer));
    memset(gnss->line_buffer, 0, sizeof(gnss->line_buffer));
    gnss->line_pos = 0;

    // Habilita interrupção IDLE
    __HAL_UART_ENABLE_IT(&GNSSuart, UART_IT_IDLE);

    // Inicia DMA em modo CIRCULAR - roda para sempre
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&GNSSuart,
                                                             gnss->rx_buffer,
                                                             gnss->rx_size);

    if (status != HAL_OK) {
        printf("[ERRO] Falha DMA GNSS: %d\r\n", status);
        return status;
    }

    printf("[GNSS] DMA iniciado (modo circular)\r\n");
    return HAL_OK;
}

/**
 * @brief Processa os dados brutos recebidos do DMA, monta linhas NMEA e as analisa.
 * @param gnss Ponteiro para a estrutura GNSS.
 * @param size Quantidade de bytes recebidos no buffer do DMA.
 */
void GNSS_ProcessData(GNSS *gnss, uint16_t size)
{
    // Proteção contra buffer overflow
    if (size == 0 || size > sizeof(gnss->rx_buffer)) {
        return;
    }

    // NÃO zera o rx_buffer aqui! Ele é gerenciado pelo DMA

    for (uint16_t i = 0; i < size; i++)
    {
        char received_char = gnss->rx_buffer[i];

        if (received_char == '$') {
            gnss->line_pos = 0;
            gnss->line_buffer[gnss->line_pos++] = received_char;
        }
        else if (gnss->line_pos > 0) {
            if (received_char == '\r') {
                continue;
            }

            if (received_char == '\n') {
                gnss->line_buffer[gnss->line_pos] = '\0';

                // Processa apenas se tiver conteúdo válido
                if (gnss->line_pos > 10 && gnss->line_buffer[0] == '$') {
                    parseNMEA(gnss->line_buffer, gnss);
                }

                gnss->line_pos = 0;
            }
            else if (gnss->line_pos < sizeof(gnss->line_buffer) - 1) {
                gnss->line_buffer[gnss->line_pos++] = received_char;
            }
            else {
                // Overflow de linha - descarta
                gnss->line_pos = 0;
            }
        }
    }
}

void GNSS_Write(uint8_t data)
{
    HAL_UART_Transmit(&GNSSuart, &data, sizeof(data), 100);
}

void GNSS_Read(GNSS *gnss)
{
    if (HAL_UART_Receive(&GNSSuart, (uint8_t*)gnss->GNSS_Data, 255, 100) == HAL_OK) {
        gnss->GNSS_Data[255] = '\0';
        parseNMEA(gnss->GNSS_Data, gnss);
    }
}

void GNSS_ReadIT(GNSS *gnss, uint16_t size)
{
    HAL_UART_Receive_IT(&GNSSuart, (uint8_t*)gnss->GNSS_Data, size);
}

/* ========================================================================
 * FUNÇÕES AUXILIARES DE IDENTIFICAÇÃO
 * ======================================================================== */

GNSS_System GNSS_GetSystemFromTalker(const char *talker)
{
    if (strncmp(talker, "GP", 2) == 0) return GNSS_GPS;
    if (strncmp(talker, "GL", 2) == 0) return GNSS_GLONASS;
    if (strncmp(talker, "GA", 2) == 0) return GNSS_GALILEO;
    if (strncmp(talker, "GB", 2) == 0 || strncmp(talker, "BD", 2) == 0) return GNSS_BEIDOU;
    if (strncmp(talker, "GN", 2) == 0) return GNSS_COMBINED;
    return GNSS_UNKNOWN;
}

NMEA_MessageType GNSS_GetMessageType(const char *sentence)
{
    if (strstr(sentence, "RMC") != NULL) return NMEA_RMC;
    if (strstr(sentence, "GGA") != NULL) return NMEA_GGA;
    if (strstr(sentence, "GSA") != NULL) return NMEA_GSA;
    if (strstr(sentence, "GSV") != NULL) return NMEA_GSV;
    if (strstr(sentence, "VTG") != NULL) return NMEA_VTG;
    if (strstr(sentence, "GLL") != NULL) return NMEA_GLL;
    return NMEA_UNKNOWN;
}

uint8_t GNSS_ValidateChecksum(const char *sentence)
{
    if (sentence[0] != '$') return 0;

    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) return 0;

    // Verifica se há pelo menos 2 caracteres após o asterisco
    if (strlen(asterisk) < 3) return 0;

    uint8_t checksum = 0;
    // Calcula o XOR de todos os caracteres entre $ e *
    for (const char *p = sentence + 1; p < asterisk; p++) {
        checksum ^= *p;
    }

    // Lê o checksum recebido usando strtol
    char *endptr;
    long received_checksum = strtol(asterisk + 1, &endptr, 16);

    // Verifica se a conversão foi bem-sucedida
    if (endptr != asterisk + 3) {
        printf("[CHECKSUM] Erro ao converter checksum\n");
        return 0;
    }

    if (checksum != (uint8_t)received_checksum) {
        printf("[CHECKSUM] Calc: 0x%02X, Recv: 0x%02X\n", checksum, (uint8_t)received_checksum);
        return 0;
    }

    return 1;
}

/* ========================================================================
 * FUNÇÕES DE CONVERSÃO
 * ======================================================================== */

double GNSS_ConvertCoordinate(double nmea_coord)
{
    int degrees = (int)(nmea_coord / 100);
    double minutes = nmea_coord - (degrees * 100);
    return degrees + (minutes / 60.0);
}

void GNSS_ConvertToDecimal(GNSS *gnss)
{
    if (gnss->lat != 0) {
        gnss->lat_decimal = GNSS_ConvertCoordinate(gnss->lat);
        if (gnss->lat_dir == 'S') gnss->lat_decimal = -gnss->lat_decimal;
    }

    if (gnss->lon != 0) {
        gnss->lon_decimal = GNSS_ConvertCoordinate(gnss->lon);
        if (gnss->lon_dir == 'W') gnss->lon_decimal = -gnss->lon_decimal;
    }

    gnss->velocidade_kmh = gnss->velocidade * 1.852;
}

/* ========================================================================
 * FUNÇÕES DE STATUS E FIX
 * ======================================================================== */

void GNSS_CheckFixStatus(GNSS *gnss)
{
	if ((gnss->status == 'A') || // From RMC
	        (gnss->quality > QUALITY_INVALID)) { // From GGA

	        gnss->is_fixed = 1;

	        if (!gnss->has_initial_pos) {
	            // Esta chamada agora é segura, pois só será acionada
	            // por GGA ou RMC, que já preencheram lat_decimal
	            GNSS_SaveInitialPosition(gnss);
	        }
	    } else {
	        gnss->is_fixed = 0;
	    }
}

void GNSS_SaveInitialPosition(GNSS *gnss)
{
	if (!gnss->is_fixed || gnss->lat_decimal == 0.0 || gnss->lon_decimal == 0.0)
	    {
	        return;
	    }

	    gnss->lat_inicial = gnss->lat_decimal;
	    gnss->lon_inicial = gnss->lon_decimal;
	    gnss->alt_inicial = gnss->altitude;
	    gnss->has_initial_pos = 1;

	    gnss->distancia_total = 0.0;
	    gnss->deslocamento_x = 0.0;
	    gnss->deslocamento_y = 0.0;
	    gnss->deslocamento_z = 0.0;
}

void GNSS_CalculateDisplacement(GNSS *gnss)
{
    if (!gnss->has_initial_pos || !gnss->is_fixed) return;

    const double R = 6371000.0;

    double lat1 = gnss->lat_inicial * M_PI / 180.0;
    double lon1 = gnss->lon_inicial * M_PI / 180.0;
    double lat2 = gnss->lat_decimal * M_PI / 180.0;
    double lon2 = gnss->lon_decimal * M_PI / 180.0;

    double delta_lat = lat2 - lat1;
    double delta_lon = lon2 - lon1;

    gnss->deslocamento_y = delta_lat * R;

    double lat_media = (lat1 + lat2) / 2.0;
    gnss->deslocamento_x = delta_lon * R * cos(lat_media);

    gnss->deslocamento_z = gnss->altitude - gnss->alt_inicial;

    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(delta_lon / 2.0) * sin(delta_lon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    gnss->distancia_total = R * c;
}

/* ========================================================================
 * FUNÇÕES DE AUXÍLIO (A-GPS)
 * ======================================================================== */

/**
 * @brief Calcula checksum NMEA
 */
static uint8_t GNSS_CalculateChecksum(const char *sentence)
{
    uint8_t checksum = 0;

    // Pula o '$' inicial
    if (sentence[0] == '$') {
        sentence++;
    }

    // XOR até encontrar '*' ou fim da string
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence;
        sentence++;
    }

    return checksum;
}

/**
 * @brief Envia comando PMTK com checksum
 */
HAL_StatusTypeDef GNSS_SendCommand(UART_HandleTypeDef *uart, const char *cmd)
{
    char buffer[128];
    uint8_t checksum;

    // Calcula checksum
    checksum = GNSS_CalculateChecksum(cmd);

    // Formata comando com checksum
    snprintf(buffer, sizeof(buffer), "$%s*%02X\r\n", cmd, checksum);

    // Envia pela UART
    return HAL_UART_Transmit(uart, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief Injeta posição, altitude e tempo conhecidos
 * Ajuda o GPS a fazer fix mais rápido (Hot Start assistido)
 */
HAL_StatusTypeDef GNSS_InjectAidData(GNSS *gnss, double lat, double lon, float alt,
                                     uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t minute, uint8_t second)
{
    extern UART_HandleTypeDef GNSSuart;
    char cmd[128];
    HAL_StatusTypeDef status;

    printf("[GNSS] Injetando dados de auxilio...\r\n");

    // 1. Injeta hora UTC (PMTK740)
    // Formato: PMTK740,year,month,day,hour,minute,second
    snprintf(cmd, sizeof(cmd), "PMTK740,%d,%d,%d,%d,%d,%d",
             year, month, day, hour, minute, second);

    status = GNSS_SendCommand(&GNSSuart, cmd);
    if (status != HAL_OK) {
        printf("[GNSS] Erro ao injetar hora\r\n");
        return status;
    }

    HAL_Delay(100);

    // 2. Injeta posição aproximada (PMTK741)
    // Formato: PMTK741,lat,lon,alt
    // Converte lat/lon para formato NMEA (ddmm.mmmm)
    int lat_deg = (int)fabs(lat);
    double lat_min = (fabs(lat) - lat_deg) * 60.0;
    char lat_dir = (lat >= 0) ? 'N' : 'S';

    int lon_deg = (int)fabs(lon);
    double lon_min = (fabs(lon) - lon_deg) * 60.0;
    char lon_dir = (lon >= 0) ? 'E' : 'W';

    snprintf(cmd, sizeof(cmd), "PMTK741,%02d%07.4f,%c,%03d%07.4f,%c,%.1f",
             lat_deg, lat_min, lat_dir,
             lon_deg, lon_min, lon_dir,
             alt);

    status = GNSS_SendCommand(&GNSSuart, cmd);
    if (status != HAL_OK) {
        printf("[GNSS] Erro ao injetar posicao\r\n");
        return status;
    }

    HAL_Delay(100);

    // 3. Força Hot Start para usar os dados injetados (PMTK101)
    status = GNSS_SetHotStart(gnss);
    if (status != HAL_OK) {
        printf("[GNSS] Erro ao iniciar Hot Start\r\n");
        return status;
    }

    printf("[GNSS] Dados injetados: %.6f,%.6f @ %.1fm\r\n", lat, lon, alt);
    printf("[GNSS] Data/Hora: %04d-%02d-%02d %02d:%02d:%02d UTC\r\n",
           year, month, day, hour, minute, second);

    return HAL_OK;
}

/**
 * @brief Hot Start - usa ephemeris, almanac, posição e tempo da memória
 * Tempo típico de fix: 1-5 segundos
 */
HAL_StatusTypeDef GNSS_SetHotStart(GNSS *gnss)
{
    extern UART_HandleTypeDef GNSSuart;

    printf("[GNSS] Configurando Hot Start...\r\n");
    return GNSS_SendCommand(&GNSSuart, "PMTK101");
}

/**
 * @brief Warm Start - usa almanac, mas não posição
 * Tempo típico de fix: 28-32 segundos
 */
HAL_StatusTypeDef GNSS_SetWarmStart(GNSS *gnss)
{
    extern UART_HandleTypeDef GNSSuart;

    printf("[GNSS] Configurando Warm Start...\r\n");
    return GNSS_SendCommand(&GNSSuart, "PMTK102");
}

/**
 * @brief Cold Start - limpa ephemeris e posição
 * Tempo típico de fix: 30-35 segundos
 */
HAL_StatusTypeDef GNSS_SetColdStart(GNSS *gnss)
{
    extern UART_HandleTypeDef GNSSuart;

    printf("[GNSS] Configurando Cold Start...\r\n");
    return GNSS_SendCommand(&GNSSuart, "PMTK103");
}

/**
 * @brief Full Cold Start - limpa TUDO (factory reset)
 * Tempo típico de fix: 32+ segundos
 */
HAL_StatusTypeDef GNSS_SetFullColdStart(GNSS *gnss)
{
    extern UART_HandleTypeDef GNSSuart;

    printf("[GNSS] Configurando Full Cold Start...\r\n");
    return GNSS_SendCommand(&GNSSuart, "PMTK104");
}

/**
 * @brief Verifica se comandos estão sendo recebidos
 * PMTK000 retorna ACK
 */
HAL_StatusTypeDef GNSS_TestCommunication(GNSS *gnss)
{
    char cmd[] = "PMTK000";

    printf("[GNSS] Testando comunicacao...\r\n");

    if (GNSS_SendCommand(&GNSSuart, cmd) != HAL_OK) {
        printf("[ERRO] Falha ao enviar comando\r\n");
        return HAL_ERROR;
    }

    // Aguarda resposta PMTK001,0,3 (ACK)
    HAL_Delay(500);

    printf("[INFO] Comando enviado - verifique resposta\r\n");
    return HAL_OK;
}

/* ========================================================================
 * PARSER PRINCIPAL
 * ======================================================================== */

void parseNMEA(char *msg, GNSS *gnss)
{
    if (msg[0] != '$') return;

    // Remove todos os printfs de debug

    const char *asterisk = strchr(msg, '*');
    if (!asterisk) return;

    if (strlen(asterisk) < 3) return;

    if (!GNSS_ValidateChecksum(msg)) {
        return;  // Remove o printf de checksum inválido também
    }

    char talker[3];
    strncpy(talker, msg + 1, 2);
    talker[2] = '\0';
    gnss->system = GNSS_GetSystemFromTalker(talker);
    strncpy(gnss->talker_id, talker, 2);
    gnss->talker_id[2] = '\0';

    gnss->msg_type = GNSS_GetMessageType(msg);

    switch (gnss->msg_type) {
        case NMEA_RMC:
            parseRMC(msg, gnss);
            break;
        case NMEA_GGA:
            parseGGA(msg, gnss);
            break;
        case NMEA_GSA:
            parseGSA(msg, gnss);
            break;
        default:
            break;
    }

    GNSS_CheckFixStatus(gnss);

    if (gnss->is_fixed) {
        if (gnss->has_initial_pos) {
            GNSS_CalculateDisplacement(gnss);
        }
        gnss->last_update = HAL_GetTick();
    }
}

/* ========================================================================
 * PARSERS DE MENSAGENS ESPECÍFICAS
 * ======================================================================== */

void parseRMC(char *msg, GNSS *gnss)
{
    char buffer[256];
    strncpy(buffer, msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token;
    int field = 0;

    // TEMPORÁRIOS - só atualiza se sentença completa válida
    char temp_status = 'V';
    double temp_lat = 0;
    double temp_lon = 0;
    char temp_lat_dir = 'N';
    char temp_lon_dir = 'E';

    token = strtok(buffer, ",");
    while (token != NULL) {
        switch (field) {
            case 0: /* tipo */
                break;
            case 1: /* hora */
                if (strlen(token) > 0) {
                    strncpy(gnss->hora, token, sizeof(gnss->hora) - 1);
                }
                break;
            case 2: /* status */
                if (strlen(token) > 0) {
                    temp_status = token[0];
                }
                break;
            case 3: /* lat */
                if (strlen(token) > 0) {
                    temp_lat = atof(token);
                }
                break;
            case 4: /* lat_dir */
                if (strlen(token) > 0) {
                    temp_lat_dir = token[0];
                }
                break;
            case 5: /* lon */
                if (strlen(token) > 0) {
                    temp_lon = atof(token);
                }
                break;
            case 6: /* lon_dir */
                if (strlen(token) > 0) {
                    temp_lon_dir = token[0];
                }
                break;

            case 7:
                if (strlen(token) > 0)
                    gnss->velocidade = atof(token);
                break;

            case 8:
                if (strlen(token) > 0)
                    gnss->angulo = atof(token);
                break;

            case 9:
                if (strlen(token) > 0) {
                    strncpy(gnss->data, token, sizeof(gnss->data) - 1);
                    gnss->data[sizeof(gnss->data) - 1] = '\0';
                }
                break;

            case 10:
                if (strlen(token) > 0)
                    gnss->variacao_magnetica = atof(token);
                break;

            case 11:
                {
                    char *asterisco = strchr(token, '*');
                    if (asterisco != NULL) *asterisco = '\0';
                    if (strlen(token) > 0)
                        gnss->direcao_variacao = token[0];
                }
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    // SÓ ATUALIZA SE STATUS FOR VÁLIDO
    if (temp_status == 'A' && temp_lat != 0 && temp_lon != 0) {
        gnss->status = temp_status;
        gnss->lat = temp_lat;
        gnss->lon = temp_lon;
        gnss->lat_dir = temp_lat_dir;
        gnss->lon_dir = temp_lon_dir;
        gnss->is_valid = 1;
        gnss->is_fixed = 1;

        GNSS_ConvertToDecimal(gnss);
    } else if (temp_status == 'V') {
        // Marca como inválido mas MANTÉM última posição conhecida
        gnss->status = 'V';
        gnss->is_valid = 0;
        gnss->is_fixed = 0;
        // NÃO zera lat/lon - mantém último valor válido
    }
}

void parseGGA(char *msg, GNSS *gnss)
{
    char buffer[256];
    strncpy(buffer, msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token;
    int field = 0;

    token = strtok(buffer, ",");
    while (token != NULL) {
        switch (field) {
            case 1:
                if (strlen(token) > 0) {
                    strncpy(gnss->hora, token, sizeof(gnss->hora) - 1);
                    gnss->hora[sizeof(gnss->hora) - 1] = '\0';
                }
                break;

            case 2:
                if (strlen(token) > 0)
                    gnss->lat = atof(token);
                break;

            case 3:
                if (strlen(token) > 0)
                    gnss->lat_dir = token[0];
                break;

            case 4:
                if (strlen(token) > 0)
                    gnss->lon = atof(token);
                break;

            case 5:
                if (strlen(token) > 0)
                    gnss->lon_dir = token[0];
                break;

            case 6:
                if (strlen(token) > 0) {
                    gnss->quality = (GNSS_Quality)atoi(token);
                    gnss->is_valid = (gnss->quality > QUALITY_INVALID) ? 1 : 0;
                    gnss->is_fixed = gnss->is_valid;
                }
                break;

            case 7:
                if (strlen(token) > 0)
                    gnss->satelites_usados = atoi(token);
                break;

            case 8:
                if (strlen(token) > 0)
                    gnss->hdop = atof(token);
                break;

            case 9:
                if (strlen(token) > 0)
                    gnss->altitude = atof(token);
                break;

            case 11:
                if (strlen(token) > 0)
                    gnss->geoidal_sep = atof(token);
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    GNSS_ConvertToDecimal(gnss);
}

void parseGSA(char *msg, GNSS *gnss)
{
    char buffer[256];
    strncpy(buffer, msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token;
    int field = 0;

    memset(gnss->satelites_ids, 0, sizeof(gnss->satelites_ids));

    token = strtok(buffer, ",");
    while (token != NULL) {
        switch (field) {
            case 1:
                if (strlen(token) > 0)
                    gnss->modo_fixo = token[0];
                break;

            case 2:
                if (strlen(token) > 0)
                    gnss->fix_mode = (GNSS_FixMode)atoi(token);
                break;

            case 3: case 4: case 5: case 6: case 7: case 8:
            case 9: case 10: case 11: case 12: case 13: case 14:
                if (strlen(token) > 0 && field >= 3 && field <= 14) {
                    gnss->satelites_ids[field - 3] = atoi(token);
                }
                break;

            case 15:
                if (strlen(token) > 0)
                    gnss->pdop = atof(token);
                break;

            case 16:
                if (strlen(token) > 0)
                    gnss->hdop = atof(token);
                break;

            case 17:
                {
                    char *asterisco = strchr(token, '*');
                    if (asterisco != NULL) *asterisco = '\0';
                    if (strlen(token) > 0)
                        gnss->vdop = atof(token);
                }
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }
}

/* ========================================================================
 * FUNÇÕES DE CÁLCULO
 * ======================================================================== */

double GNSS_CalculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0;

    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double delta_phi = (lat2 - lat1) * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}

double GNSS_CalculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double y = sin(delta_lambda) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lambda);

    double bearing = atan2(y, x) * 180.0 / M_PI;

    return fmod((bearing + 360.0), 360.0);
}

/* ========================================================================
 * FUNÇÃO DE IMPRESSÃO
 * ======================================================================== */

void GNSS_PrintInfo(GNSS *gnss)
{
    printf("\n========== INFORMACOES DO GNSS ==========\n");

    printf("\n--- STATUS ---\n");
    printf("Sistema GNSS: %s ", gnss->talker_id);
    switch(gnss->system) {
        case GNSS_GPS:      printf("(GPS)\n"); break;
        case GNSS_GLONASS:  printf("(GLONASS)\n"); break;
        case GNSS_GALILEO:  printf("(Galileo)\n"); break;
        case GNSS_BEIDOU:   printf("(BeiDou)\n"); break;
        case GNSS_COMBINED: printf("(Combinado)\n"); break;
        default:            printf("(Desconhecido)\n"); break;
    }

    printf("Status Fix: %s\n", gnss->is_fixed ? "FIXADO" : "SEM FIX");
    printf("Status RMC: %c (%s)\n", gnss->status,
           gnss->status == 'A' ? "Ativo" : "Invalido");

    printf("Qualidade: ");
    switch(gnss->quality) {
        case QUALITY_INVALID:    printf("Invalido\n"); break;
        case QUALITY_GPS_FIX:    printf("GPS Fix\n"); break;
        case QUALITY_DGPS_FIX:   printf("DGPS Fix\n"); break;
        case QUALITY_PPS_FIX:    printf("PPS Fix\n"); break;
        case QUALITY_RTK_FIX:    printf("RTK Fix\n"); break;
        case QUALITY_RTK_FLOAT:  printf("RTK Float\n"); break;
        default:                 printf("Outro\n"); break;
    }

    printf("Modo Fix: ");
    switch(gnss->fix_mode) {
        case FIX_NONE: printf("Sem Fix\n"); break;
        case FIX_2D:   printf("2D\n"); break;
        case FIX_3D:   printf("3D\n"); break;
        default:       printf("Desconhecido\n"); break;
    }

    printf("\n--- POSICAO ATUAL ---\n");
    printf("Hora: %s UTC\n", gnss->hora);
    printf("Data: %s\n", gnss->data);
    printf("Latitude: %.6f%c (%.6f deg)\n",
           gnss->lat, gnss->lat_dir, gnss->lat_decimal);
    printf("Longitude: %.6f%c (%.6f deg)\n",
           gnss->lon, gnss->lon_dir, gnss->lon_decimal);
    printf("Altitude: %.2f m\n", gnss->altitude);

    printf("\n--- MOVIMENTO ---\n");
    printf("Velocidade: %.2f nos (%.2f km/h)\n",
           gnss->velocidade, gnss->velocidade_kmh);
    printf("Angulo/Curso: %.2f graus\n", gnss->angulo);

    printf("\n--- SATELITES E PRECISAO ---\n");
    printf("Satelites usados: %d\n", gnss->satelites_usados);
    printf("HDOP: %.2f\n", gnss->hdop);
    printf("PDOP: %.2f\n", gnss->pdop);
    printf("VDOP: %.2f\n", gnss->vdop);

    if (gnss->has_initial_pos) {
        printf("\n--- POSICAO INICIAL ---\n");
        printf("Latitude inicial: %.6f deg\n", gnss->lat_inicial);
        printf("Longitude inicial: %.6f deg\n", gnss->lon_inicial);
        printf("Altitude inicial: %.2f m\n", gnss->alt_inicial);

        printf("\n--- DESLOCAMENTO ---\n");
        printf("Deslocamento X (Leste/Oeste): %.2f m %s\n",
               fabs(gnss->deslocamento_x),
               gnss->deslocamento_x >= 0 ? "(Leste)" : "(Oeste)");
        printf("Deslocamento Y (Norte/Sul): %.2f m %s\n",
               fabs(gnss->deslocamento_y),
               gnss->deslocamento_y >= 0 ? "(Norte)" : "(Sul)");
        printf("Deslocamento Z (Vertical): %.2f m %s\n",
               fabs(gnss->deslocamento_z),
               gnss->deslocamento_z >= 0 ? "(Subiu)" : "(Desceu)");
        printf("Distancia total: %.2f m\n", gnss->distancia_total);
    } else {
        printf("\n--- POSICAO INICIAL ---\n");
        printf("Aguardando primeiro fix valido...\n");
    }

    printf("\n--- MAGNETISMO ---\n");
    printf("Variacao magnetica: %.2f%c\n",
           gnss->variacao_magnetica, gnss->direcao_variacao);

    printf("\n=========================================\n\n");
}

void GNSS_PrintDiagnostics(GNSS *gnss)
{
    printf("\r\n========== DIAGNOSTICO GPS ==========\r\n");
    printf("Status: %c (%s)\r\n", gnss->status, gnss->status == 'A' ? "VALIDO" : "INVALIDO");
    printf("Quality: %d\r\n", gnss->quality);
    printf("Satelites visiveis: %d\r\n", gnss->satelites_usados);
    printf("is_fixed: %d\r\n", gnss->is_fixed);
    printf("is_valid: %d\r\n", gnss->is_valid);

    if (gnss->lat_decimal != 0 || gnss->lon_decimal != 0) {
        printf("Lat: %.6f, Lon: %.6f\r\n", gnss->lat_decimal, gnss->lon_decimal);
    } else {
        printf("Sem coordenadas validas\r\n");
    }

    printf("HDOP: %.2f\r\n", gnss->hdop);
    printf("=====================================\r\n");
}


//-------------------+-----------------------------+-----------------------


