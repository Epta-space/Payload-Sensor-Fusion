/*
 * ROCKET.c
 *
 *  Created on: Feb 9, 2025
 *      Author: de4lerr
 */

#include "ROCKET.h"
#include <string.h>

/* Intervalos padrão (ms) */
static uint32_t tx_interval = 2000;
static uint32_t sd_interval = 100;
static uint32_t bar_interval = 100;
static uint32_t imu_interval = 5;
static uint32_t ekf_interval = 20;

/* ========================================================================
 * FUNÇÕES AUXILIARES INTERNAS
 * ======================================================================== */

static int invert3x3(const float A[3][3], float A_inv[3][3]) {
    float det = A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
              - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
              + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);

    if (fabs(det) < 1e-6) return 0;

    A_inv[0][0] =  (A[1][1]*A[2][2]-A[1][2]*A[2][1]) / det;
    A_inv[0][1] = -(A[0][1]*A[2][2]-A[0][2]*A[2][1]) / det;
    A_inv[0][2] =  (A[0][1]*A[1][2]-A[0][2]*A[1][1]) / det;

    A_inv[1][0] = -(A[1][0]*A[2][2]-A[1][2]*A[2][0]) / det;
    A_inv[1][1] =  (A[0][0]*A[2][2]-A[0][2]*A[2][0]) / det;
    A_inv[1][2] = -(A[0][0]*A[1][2]-A[0][2]*A[1][0]) / det;

    A_inv[2][0] =  (A[1][0]*A[2][1]-A[1][1]*A[2][0]) / det;
    A_inv[2][1] = -(A[0][0]*A[2][1]-A[0][1]*A[2][0]) / det;
    A_inv[2][2] =  (A[0][0]*A[1][1]-A[0][1]*A[1][0]) / det;

    return 1;
}

/* ========================================================================
 * INICIALIZAÇÃO
 * ======================================================================== */

void ROCKET_Init(ROCKET_System_t *rocket, GNSS *gnss, SD *sd, LORA *lora,
                 BAR *bar, IMU *imu, IMU_Fusion *fusion, ROCKET_FSM_t *fsm)
{
    memset(rocket, 0, sizeof(ROCKET_System_t));

    rocket->gnss = gnss;
    rocket->sd = sd;
    rocket->lora = lora;
    rocket->bar = bar;
    rocket->imu = imu;
    rocket->fusion = fusion;
    rocket->fsm = fsm;

    rocket->timers.lora_tx = HAL_GetTick();
    rocket->timers.sd_write = HAL_GetTick();
    rocket->timers.bar_update = HAL_GetTick();
    rocket->timers.gnss_check = HAL_GetTick();
    rocket->timers.imu_update = HAL_GetTick();
    rocket->timers.ekf_update = HAL_GetTick();

    rocket->use_ekf = 1;
    rocket->ekf_update_rate = 50.0f; // 50Hz

    ROCKET_Logger_Init(rocket);
}

HAL_StatusTypeDef ROCKET_BeginPeripherals(ROCKET_System_t *rocket,
                                          UART_HandleTypeDef *huart_lora,
                                          UART_HandleTypeDef *huart_gnss,
                                          I2C_HandleTypeDef *hi2c_bar,
                                          SPI_HandleTypeDef *hspi_imu,
                                          GPIO_TypeDef *m0_port, uint16_t m0_pin,
                                          GPIO_TypeDef *m1_port, uint16_t m1_pin,
                                          GPIO_TypeDef *imu_cs_port, uint16_t imu_cs_pin)
{
    printf("\r\n========== INICIALIZACAO DO SISTEMA ==========\r\n");

    /* LoRa */
    LORA_Init(rocket->lora);
    if (LORA_Begin(rocket->lora, huart_lora, m0_port, m0_pin, m1_port, m1_pin) != HAL_OK) {
        printf("[ERRO] Falha ao inicializar LoRa\r\n");
        rocket->status.lora_initialized = 0;
        return HAL_ERROR;
    }
    rocket->status.lora_initialized = 1;
    printf("[OK] LoRa inicializado\r\n");

    /* SD Card */
    SD_Init(rocket->sd);
    uint8_t retry = 0;
    while (SD_Mount(rocket->sd) != HAL_OK && retry < 5) {
        printf("[INFO] Tentativa %d de montar SD...\r\n", retry + 1);
        HAL_Delay(500);
        retry++;
    }
    rocket->status.sd_initialized = (retry < 5);
    printf("[%s] SD Card %s\r\n",
           rocket->status.sd_initialized ? "OK" : "AVISO",
           rocket->status.sd_initialized ? "montado" : "nao montado");

    /* GNSS */
    GNSS_Init(*huart_gnss, rocket->gnss);
    if (GNSS_Configure(rocket->gnss) != HAL_OK) {
        printf("[ERRO] Falha ao configurar GNSS\r\n");
        rocket->status.gnss_initialized = 0;
        return HAL_ERROR;
    }
    HAL_Delay(1000);
    printf("[OK] GNSS configurado\r\n");
    if (GNSS_StartDMA(rocket->gnss) != HAL_OK) {
        printf("[ERRO] Falha ao iniciar GNSS DMA\r\n");
        rocket->status.gnss_initialized = 0;
        return HAL_ERROR;
    }
    HAL_Delay(1000);
    rocket->status.gnss_initialized = 1;
    printf("[OK] GNSS inicializado\r\n");

    /* Barômetro */
    BAR_Init(rocket->bar);
    if (BAR_Begin(rocket->bar, hi2c_bar) == HAL_OK) {
        rocket->status.bar_initialized = 1;
        printf("[OK] Barometro inicializado\r\n");
    } else {
        rocket->status.bar_initialized = 0;
        printf("[AVISO] Barometro nao encontrado\r\n");
    }

    /* IMU */
    if (rocket->imu != NULL && hspi_imu != NULL) {
        IMU_Init(rocket->imu);
        if (IMU_Begin(rocket->imu, hspi_imu, imu_cs_port, imu_cs_pin) == HAL_OK) {
            rocket->status.imu_initialized = 1;
            printf("[OK] IMU inicializado\r\n");

            printf("[INFO] Calibrando IMU (mantenha estatico)...\r\n");

            // Hard Iron Offset (b)
            float mag_offset[3] = {90.459303f, 148.119876f, 54.776028f};

            // Soft Iron Matrix (A)
            float mag_scale[3][3] = {
                {0.93316302f, 0.01360155f, -0.02370045f},
                {0.01360155f, 1.27097147f, 0.06653814f},
                {-0.02370045f, 0.06653814f, 0.84740728f}
            };

            // Aplicar calibracao
            IMU_SetMagCalibration(rocket->imu, mag_offset, mag_scale);

            if (ROCKET_CalibrateIMU(rocket, 100) == HAL_OK) {
                printf("[OK] IMU calibrado\r\n");
            } else {
                printf("[AVISO] Falha na calibracao do IMU\r\n");
            }

            if (rocket->fusion != NULL) {
                FUSION_Init(rocket->fusion, 200.0f);
                rocket->status.fusion_initialized = 1;
                printf("[OK] Fusao sensorial IMU inicializada\r\n");
            }
        } else {
            rocket->status.imu_initialized = 0;
            printf("[AVISO] IMU nao encontrado\r\n");
        }
    }

    /* Inicializa EKF */
    if (rocket->use_ekf) {
        ROCKET_EKF_Init(&rocket->ekf, 1.0f / rocket->ekf_update_rate);
        ROCKET_InitEKF_PreFlight(rocket);
        rocket->status.ekf_initialized = 1;
        printf("[OK] EKF inicializado (taxa: %.0f Hz)\r\n", rocket->ekf_update_rate);
    }

    rocket->status.system_ready = 1;
    printf("==============================================\r\n\r\n");

    return HAL_OK;
}

/* ========================================================================
 * MANAGER PRINCIPAL
 * ======================================================================== */

void ROCKET_Manager(ROCKET_System_t *rocket)
{
    uint32_t now = HAL_GetTick();

    // AJUSTAR INTERVALOS POR FASE
    static FlightState_t last_sensor_config = FLIGHT_IDLE;
    if (rocket->fsm->current_state != last_sensor_config) {

        // GPS
        GNSS_SetFlightMode(rocket->gnss, rocket->fsm->current_state);

        // Barômetro
        BAR_SetModeForPhase(rocket->bar, rocket->fsm->current_state);

//        // IMU (opcional)
//        IMU_SetODRForPhase(rocket->imu, rocket->fsm->current_state);

        // Intervalos do sistema
        ROCKET_AdjustTimingsForPhase(rocket, rocket->fsm->current_state);

        last_sensor_config = rocket->fsm->current_state;

        printf("[SENSORES] Reconfigurados para %s\r\n",
               ROCKET_FSM_GetStateName(rocket->fsm->current_state));
    }

    /* LoRa (prioritário) */
    if (rocket->status.lora_initialized) {
        LORA_Manager(rocket->lora);
    }

    /* IMU (alta frequência - 200Hz) */
    if (rocket->status.imu_initialized && (now - rocket->timers.imu_update) >= imu_interval) {
        ROCKET_UpdateIMU(rocket);
        rocket->timers.imu_update = now;
    }

    /* Barômetro (10Hz) */
    if (rocket->status.bar_initialized && (now - rocket->timers.bar_update) >= bar_interval) {
        ROCKET_UpdateBarometer(rocket);
        rocket->timers.bar_update = now;
    }

    /* EKF (50Hz) */
    if (rocket->use_ekf && rocket->status.ekf_initialized &&
        (now - rocket->timers.ekf_update) >= ekf_interval) {
        ROCKET_UpdateState(rocket);
        rocket->timers.ekf_update = now;
    }

    /* Transmissão LoRa */
    if ((now - rocket->timers.lora_tx) >= tx_interval) {
        ROCKET_SendTelemetry(rocket);
        rocket->timers.lora_tx = now;
    }

    /* Gravação SD */
    // Reabrir imediatamente se fechou acidentalmente
    if (rocket->logger.file_open && !rocket->sd->is_file_open) {
        SD_OpenFile(rocket->sd, rocket->logger.current_filename, SD_MODE_APPEND);
    }
    /* Adiciona amostra ao buffer após atualizar sensores */
    static uint32_t last_sample = 0;
    if (rocket->logger.file_open && (now - last_sample) >= sd_interval) {
        ROCKET_Logger_AddSample(rocket);
        last_sample = now;
    }

	/* Flush inteligente baseado na fase de voo */
	uint32_t flush_interval;
	switch(rocket->fsm->current_state) {
		case FLIGHT_BOOST:
		case FLIGHT_COAST:
			// VOO CRÍTICO - flush rápido para não perder dados
			flush_interval = CRITICAL_FLUSH_INTERVAL;  // 100ms
			break;

		case FLIGHT_APOGEE:
		case FLIGHT_DROGUE_DESCENT:
		case FLIGHT_MAIN_DESCENT:
			// Flush moderado
			flush_interval = 500;
			break;

		default:
			// Flush lento para economizar ciclos
			flush_interval = 2000;
			break;
	}

	if ((now - rocket->logger.last_flush) >= flush_interval) {
		ROCKET_Logger_Flush(rocket);
	}

    /* Watchdog para GNSS travado */
//    static uint32_t gnss_watchdog = 0;
//    if ((HAL_GetTick() - rocket->gnss->last_update) > 3000) {
//        printf("[AVISO] GNSS sem atualizacao ha 1,1s\r\n");
//        GNSS_StartDMA(rocket->gnss);  // Reinicia
//        gnss_watchdog = HAL_GetTick();
//    }
}

/* ========================================================================
 * FUSÃO SENSORIAL (EKF)
 * ======================================================================== */

void ROCKET_EKF_Init(ROCKET_EKF *ekf, float dt)
{
    int i;//, j;
    ekf->dt = dt;

    // Estado inicial
    memset(ekf->x, 0, sizeof(ekf->x));

    // Covariância inicial
    memset(ekf->P, 0, sizeof(ekf->P));
    for(i = 0; i < 6; i++) {
        ekf->P[i][i] = 1.0f;
    }

    // --- Ruído do Processo (Q) ---
	// Incerteza do modelo físico (vento, empuxo)
    // MUITO maiores que o ruído do sensor ICM-20948.

	// Desv. padrão da posição (ruído "jerk") - 1cm
	float q_pos_horiz = 0.0001f; // (0.01m)^2
	float q_pos_vert  = 0.0001f; // (0.01m)^2

	// Horizontal (vento): sigma = 0.5 m/s^2
	float q_vel_horiz = 0.25f;   // (0.5)^2
	// Vertical (empuxo): sigma = 1.5 m/s^2
	float q_vel_vert  = 2.25f;   // (1.5)^2

	memset(ekf->Q, 0, sizeof(ekf->Q));
	ekf->Q[0][0] = q_pos_horiz; // Posição Norte
	ekf->Q[1][1] = q_pos_horiz; // Posição Leste
	ekf->Q[2][2] = q_pos_vert;  // Posição Baixo (Down)
	ekf->Q[3][3] = q_vel_horiz; // Velocidade Norte
	ekf->Q[4][4] = q_vel_horiz; // Velocidade Leste
	ekf->Q[5][5] = q_vel_vert;  // Velocidade Baixo (Down)


	// --- Ruído de Medição (R) ---
	// Incerteza dos sensores (baseado nos datasheets)

	// Ruído GPS (Quectel L80: <2.5m CEP)
	// R_horiz = (2.12m)^2 = 4.5
	// R_vert = (3.18m)^2 = 10.1
	memset(ekf->R_gps, 0, sizeof(ekf->R_gps));
	ekf->R_gps[0][0] = 4.5f;  // Norte (horizontal)
	ekf->R_gps[1][1] = 4.5f;  // Leste (horizontal)
	ekf->R_gps[2][2] = 10.1f; // Baixo (vertical)

	// Ruído Barômetro (ICP-101xx: 0.8 Pa RMS @ LN Mode)
	// R_baro = (0.068m)^2 = 0.0046
	ekf->R_baro = 0.0046f;
}

void ROCKET_EKF_EnforceSymmetry(ROCKET_EKF *ekf)
{
    int i, j;
    float P_sym[6][6];

    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            P_sym[i][j] = 0.5f * (ekf->P[i][j] + ekf->P[j][i]);
        }
    }
    memcpy(ekf->P, P_sym, sizeof(P_sym));
}

void ROCKET_InitEKF_PreFlight(ROCKET_System_t *rocket)
{
    // Inicializa EKF com posição zero
    rocket->ekf.x[0] = 0;  // x
    rocket->ekf.x[1] = 0;  // y
    rocket->ekf.x[2] = 0;  // z (referência initial)
    rocket->ekf.x[3] = 0;  // vx
    rocket->ekf.x[4] = 0;  // vy
    rocket->ekf.x[5] = 0;  // vz

    // Covariância alta inicial
    for(int i = 0; i < 3; i++) {
        rocket->ekf.P[i][i] = 100.0f;      // Posição incerta
        rocket->ekf.P[i+3][i+3] = 10.0f;   // Velocidade incerta
    }

    rocket->status.ekf_initialized = 1;
    printf("[EKF] Pré-inicializado para voo\r\n");
}

void ROCKET_EKF_Predict(ROCKET_EKF *ekf, float ax, float ay, float az)
{
    int i, j, k;
    float dt = ekf->dt;
    float dt2 = 0.5f * dt * dt;

    // Predição do estado: x = x + v*dt + 0.5*a*dt²
    ekf->x[0] += ekf->x[3]*dt + dt2*ax;
    ekf->x[1] += ekf->x[4]*dt + dt2*ay;
    ekf->x[2] += ekf->x[5]*dt + dt2*az;
    ekf->x[3] += dt*ax;
    ekf->x[4] += dt*ay;
    ekf->x[5] += dt*az;

    // Matriz de transição F
    float F[6][6] = {
        {1, 0, 0, dt, 0,  0},
        {0, 1, 0, 0,  dt, 0},
        {0, 0, 1, 0,  0,  dt},
        {0, 0, 0, 1,  0,  0},
        {0, 0, 0, 0,  1,  0},
        {0, 0, 0, 0,  0,  1}
    };

    // P = F*P*F' + Q
    float P_temp[6][6] = {0};
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            for(k = 0; k < 6; k++) {
                P_temp[i][j] += F[i][k] * ekf->P[k][j];
            }
        }
    }

    float P_new[6][6] = {0};
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            for(k = 0; k < 6; k++) {
                P_new[i][j] += P_temp[i][k] * F[j][k];
            }
            P_new[i][j] += ekf->Q[i][j];
        }
    }

    memcpy(ekf->P, P_new, sizeof(P_new));

    ROCKET_EKF_EnforceSymmetry(ekf);
}

void ROCKET_EKF_UpdateGPS(ROCKET_EKF *ekf, float gps_x, float gps_y, float gps_z)
{
    int i, j, k;
    float z_meas[3] = {gps_x, gps_y, gps_z};
    float z_pred[3] = {ekf->x[0], ekf->x[1], ekf->x[2]};
    float y_innov[3] = {z_meas[0]-z_pred[0], z_meas[1]-z_pred[1], z_meas[2]-z_pred[2]};

    // S = H*P*H' + R
    float S[3][3] = {0};
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            S[i][j] = ekf->P[i][j] + ekf->R_gps[i][j];
        }
    }

    float S_inv[3][3];
    if (!invert3x3(S, S_inv)) return;

    // K = P*H'*S_inv
    float K[6][3] = {0};
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 3; j++) {
            for(k = 0; k < 3; k++) {
                K[i][j] += ekf->P[i][k] * S_inv[k][j];
            }
        }
    }

    // x = x + K*y
    for(i = 0; i < 6; i++) {
        float delta = 0;
        for(j = 0; j < 3; j++) {
            delta += K[i][j] * y_innov[j];
        }
        ekf->x[i] += delta;
    }

    // P = (I - K*H)*P
    float P_new[6][6];
    memcpy(P_new, ekf->P, sizeof(P_new));
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            float correction = 0;
            for(k = 0; k < 3; k++) {
                correction += K[i][k] * ekf->P[k][j];
            }
            P_new[i][j] -= correction;
        }
    }
    memcpy(ekf->P, P_new, sizeof(P_new));

    ROCKET_EKF_EnforceSymmetry(ekf);
}

void ROCKET_EKF_UpdateBaro(ROCKET_EKF *ekf, float baro_z)
{
    int i, j;
    float z_meas = baro_z;
    float z_pred = ekf->x[2];
    float y = z_meas - z_pred;

    float S = ekf->P[2][2] + ekf->R_baro;
    if (fabs(S) < 1e-6) return;

    float K[6];
    for(i = 0; i < 6; i++) {
        K[i] = ekf->P[i][2] / S;
    }

    for(i = 0; i < 6; i++) {
        ekf->x[i] += K[i] * y;
    }

    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            ekf->P[i][j] -= K[i] * ekf->P[2][j];
        }
    }

    ROCKET_EKF_EnforceSymmetry(ekf);
}

void ROCKET_GPS_ToNED(ROCKET_System_t *rocket, double lat, double lon, float alt,
                      float *ned_x, float *ned_y, float *ned_z)
{
    if (!rocket->gnss->has_initial_pos) {
        *ned_x = 0;
        *ned_y = 0;
        *ned_z = 0;
        return;
    }

    const double R = 6371000.0; // Raio da Terra (m)

    // Posição inicial (origem NED)
    double lat1 = rocket->gnss->lat_inicial * M_PI / 180.0;
    double lon1 = rocket->gnss->lon_inicial * M_PI / 180.0;
    float  alt1 = rocket->gnss->alt_inicial; // MSL inicial

    // Posição atual
    double lat2 = lat * M_PI / 180.0;
    double lon2 = lon * M_PI / 180.0;

    // Delta angular
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // ============ NORTE (positivo = norte) ============
    *ned_y = (float)(dlat * R);

    // ============ LESTE (positivo = leste) ============
    *ned_x = (float)(dlon * R * cos((lat1 + lat2) / 2.0));

    // ============ BAIXO (positivo = para baixo) ============
    *ned_z = -(alt - alt1);  // ✅ Altitude relativa com sinal NED
}


void ROCKET_UpdateState(ROCKET_System_t *rocket)
{
    static uint8_t ekf_initialized_from_gps = 0;
    ROCKET_State *state = &rocket->state;
    state->dt = rocket->ekf.dt;

    state->gps_valid = rocket->gnss->is_fixed && (rocket->gnss->status == 'A');
    state->baro_valid = rocket->bar->is_valid;
    state->imu_valid = rocket->imu->is_valid && rocket->imu->attitude_reliable;

    // ============ INICIALIZAÇÃO COM GPS ============
    if (state->gps_valid && !ekf_initialized_from_gps) {

        // ✅ Se é o primeiro fix, salva como ORIGEM
    	if (rocket->gnss->has_initial_pos && !ekf_initialized_from_gps) {
    		//GNSS_SaveInitialPosition(rocket->gnss);
            printf("[EKF] Origem NED definida: LAT=%.6f LON=%.6f ALT=%.1fm\r\n",
                   rocket->gnss->lat_decimal,
                   rocket->gnss->lon_decimal,
                   rocket->gnss->altitude);
        }

        // ✅ CORREÇÃO: Converte GPS para NED (agora retorna 0,0,0)
        float ned_x, ned_y, ned_z;
        ROCKET_GPS_ToNED(rocket,
                        rocket->gnss->lat_decimal,
                        rocket->gnss->lon_decimal,
                        rocket->gnss->altitude,
                        &ned_x, &ned_y, &ned_z);

        // ✅ Estado inicial do EKF = origem (0,0,0)
        rocket->ekf.x[0] = ned_x;  // Deve ser ~0
        rocket->ekf.x[1] = ned_y;  // Deve ser ~0
        rocket->ekf.x[2] = ned_z;  // Deve ser ~0
        rocket->ekf.x[3] = 0;      // Velocidade inicial zero
        rocket->ekf.x[4] = 0;
        rocket->ekf.x[5] = 0;

        // ✅ Covariância inicial
        for(int i = 0; i < 3; i++) {
            rocket->ekf.P[i][i] = 10.0f;      // ±3m horizontal
            rocket->ekf.P[i+3][i+3] = 1.0f;   // ±1m/s velocidade
        }
        rocket->ekf.P[2][2] = 25.0f;          // ±5m vertical (GPS menos preciso)

        ekf_initialized_from_gps = 1;
        printf("[EKF] Inicializado: NED=[%.2f, %.2f, %.2f] m (esperado ~0)\r\n",
               ned_x, ned_y, ned_z);

        // ✅ VALIDAÇÃO: Se não estiver próximo de zero, algo está errado
        if (fabs(ned_x) > 5 || fabs(ned_y) > 5 || fabs(ned_z) > 10) {
            printf("[AVISO] Conversão GPS→NED suspeita! Verifique GNSS_SaveInitialPosition\r\n");
        }
    }

    // ✅ NOVO: Zero-velocity update em repouso
    static uint32_t last_motion = 0;

    // Detecta movimento significativo
    float acc_mag = sqrtf(rocket->imu->acc_x * rocket->imu->acc_x +
                          rocket->imu->acc_y * rocket->imu->acc_y +
                          rocket->imu->acc_z * rocket->imu->acc_z);

    // Se aceleração ~1g (repouso) por 2 segundos
    if (fabs(acc_mag - 1.0f) < 0.1f) {
        if ((HAL_GetTick() - last_motion) > 2000) {
            // Força velocidade zero
            rocket->ekf.x[3] = 0;
            rocket->ekf.x[4] = 0;
            rocket->ekf.x[5] = 0;

            // Reduz covariância de velocidade
            rocket->ekf.P[3][3] = 0.01f;
            rocket->ekf.P[4][4] = 0.01f;
            rocket->ekf.P[5][5] = 0.01f;
        }
    } else {
        last_motion = HAL_GetTick();
    }

    // SÓ ATUALIZA EKF SE JÁ FOI INICIALIZADO
    if (!ekf_initialized_from_gps) {
        return;
    }

    // ============ PREDIÇÃO COM IMU ============
    if (state->imu_valid) {
        // ✅ Remove offset (gravidade em repouso)
        float ax = rocket->imu->acc_ned_x - rocket->imu->acc_ned_offset_x;
        float ay = rocket->imu->acc_ned_y - rocket->imu->acc_ned_offset_y;
        float az = rocket->imu->acc_ned_z - rocket->imu->acc_ned_offset_z;

        // ✅ Threshold para eliminar ruído em repouso
        if (fabs(ax) < 0.05f * 9.81f) ax = 0;
        if (fabs(ay) < 0.05f * 9.81f) ay = 0;
        if (fabs(az) < 0.05f * 9.81f) az = 0;

        ROCKET_EKF_Predict(&rocket->ekf, ax, ay, az);

        state->imu_acc[0] = ax;
        state->imu_acc[1] = ay;
        state->imu_acc[2] = az;
    }

    // ============ ATUALIZAÇÃO GPS ============
    if (state->gps_valid) {
        float ned_x, ned_y, ned_z;
        ROCKET_GPS_ToNED(rocket,
                        rocket->gnss->lat_decimal,
                        rocket->gnss->lon_decimal,
                        rocket->gnss->altitude,
                        &ned_x, &ned_y, &ned_z);

        // ✅ VALIDAÇÃO: Rejeita medições absurdas
        if (fabs(ned_x) < 10000 && fabs(ned_y) < 10000 && fabs(ned_z) < 5000) {
            ROCKET_EKF_UpdateGPS(&rocket->ekf, ned_x, ned_y, ned_z);
        } else {
            printf("[AVISO] GPS NED rejeitado: [%.1f, %.1f, %.1f]\r\n", ned_x, ned_y, ned_z);
        }
    }

    // ============ ATUALIZAÇÃO BARÔMETRO ============
    if (state->baro_valid && rocket->bar->has_initial_alt) {
        // ✅ Barômetro: altitude relativa → NED (negativo = subiu)
        float baro_ned_z = -(rocket->bar->altitude_m - rocket->bar->altitude_inicial);

        // ✅ VALIDAÇÃO: Rejeita leituras absurdas
        if (fabs(baro_ned_z) < 2000) {  // Não voa mais de 2km
            ROCKET_EKF_UpdateBaro(&rocket->ekf, baro_ned_z);
        }
    }

    // ============ COPIA ESTADO ESTIMADO ============
    for (int i = 0; i < 3; i++) {
        state->pos[i] = rocket->ekf.x[i];
        state->vel[i] = rocket->ekf.x[i+3];
    }

    // ============ ATITUDE ============
    if (state->imu_valid) {
        if (rocket->status.fusion_initialized && rocket->fusion != NULL) {
            state->roll = rocket->fusion->roll_mw;
            state->pitch = rocket->fusion->pitch_mw;
            state->yaw = rocket->fusion->yaw_mw;
        } else {
            state->roll = rocket->imu->roll;
            state->pitch = rocket->imu->pitch;
            state->yaw = rocket->imu->yaw;
        }
    }
}


/* ========================================================================
 * ATUALIZAÇÃO DE SENSORES
 * ======================================================================== */

void ROCKET_UpdateBarometer(ROCKET_System_t *rocket)
{
	if (rocket->status.bar_initialized) {
		BAR_Update(rocket->bar);
	}
}

void ROCKET_UpdateIMU(ROCKET_System_t *rocket)
{
	if (rocket->status.imu_initialized) {
		//IMU_Update_(rocket->imu);
		IMU_Update(rocket->imu, rocket->fusion, rocket->fsm);

		if (rocket->status.fusion_initialized && rocket->fusion != NULL) {
			//FUSION_Update(rocket->fusion, rocket->imu);
		}
	}
}

void ROCKET_CheckGNSS(ROCKET_System_t *rocket)
{
	uint32_t time_since_update = HAL_GetTick() - rocket->gnss->last_update;

	if (time_since_update > 5000) {
		printf("[AVISO] GNSS sem dados ha %lu ms\r\n", time_since_update);
	}
}

/* ========================================================================
 * TELEMETRIA
 * ======================================================================== */

void ROCKET_BuildPayload(ROCKET_System_t *rocket, ROCKET_Payload_t *payload)
{
    memset(payload, 0, sizeof(ROCKET_Payload_t));

    payload->time = HAL_GetTick() / 1000.0f;

    // ============ GPS (coordenadas absolutas) ============
    if (rocket->gnss->is_fixed &&
        rocket->gnss->status == 'A' &&
        fabs(rocket->gnss->lat_decimal) <= 90.0 &&
        fabs(rocket->gnss->lon_decimal) <= 180.0) {

        payload->lat = (float)rocket->gnss->lat_decimal;
        payload->lon = (float)rocket->gnss->lon_decimal;
        payload->alt = (float)rocket->gnss->altitude;  // MSL absoluto
        payload->gnss_fix = 1;
    } else {
        payload->gnss_fix = 0;
    }

    // ============ BARÔMETRO (altitude absoluta MSL) ============
    if (rocket->bar->is_valid) {
        payload->bar_alt = rocket->bar->altitude_m;  // MSL absoluto
    }

    // ============ EKF (posições NED relativas) ============
    if (rocket->use_ekf && rocket->status.ekf_initialized) {

        // Validação de sanidade
        if (fabs(rocket->state.pos[0]) < 100000 &&
            fabs(rocket->state.pos[1]) < 100000 &&
            fabs(rocket->state.pos[2]) < 10000 &&
            fabs(rocket->state.vel[0]) < 1000 &&
            fabs(rocket->state.vel[1]) < 1000 &&
            fabs(rocket->state.vel[2]) < 1000) {

            // ✅ NORTH = X (positivo = norte)
            payload->north = rocket->state.pos[0];

            // ✅ EAST = Y (positivo = leste)
            payload->east = rocket->state.pos[1];

            // ✅ DOWN = -Z (convenção: DOWN positivo = subiu)
            // EKF usa NED: Z negativo = subiu
            // Telemetria: DOWN positivo = altitude relativa
            payload->down = -rocket->state.pos[2];  // Inverte sinal para intuição

            // ✅ Velocidades NED (sem inversão)
            payload->veln = rocket->state.vel[0];   // Norte
            payload->vele = rocket->state.vel[1];   // Leste
            payload->veld = rocket->state.vel[2];   // Vertical NED (neg = subindo)

            // ✅ Acelerações NED (sem gravidade)
            payload->acc_x = rocket->state.imu_acc[0];
            payload->acc_y = rocket->state.imu_acc[1];
            payload->acc_z = rocket->state.imu_acc[2];
        }
    }

    // ============ ATITUDE IMU ============
    if (rocket->imu->is_valid) {
        payload->imu_valid = 1;

        if (rocket->status.fusion_initialized) {
            payload->roll = rocket->fusion->roll_mw * 180.0f / M_PI;
            payload->pitch = rocket->fusion->pitch_mw * 180.0f / M_PI;
            payload->yaw = rocket->fusion->yaw_mw * 180.0f / M_PI;
        } else {
            payload->roll = rocket->imu->roll * 180.0f / M_PI;
            payload->pitch = rocket->imu->pitch * 180.0f / M_PI;
            payload->yaw = rocket->imu->yaw * 180.0f / M_PI;
        }
    }
}


HAL_StatusTypeDef ROCKET_SendTelemetry(ROCKET_System_t *rocket)
{
	if (!rocket->status.lora_initialized) {
		return HAL_ERROR;
	}

	// Verifica se TX anterior completou
	int8_t tx_status = LORA_IsTxComplete(rocket->lora);
	if (tx_status == 0) {
		printf("[WARN] TX ocupado\r\n");
		return HAL_BUSY;
	} else if (tx_status < 0) {
		printf("[WARN] TX timeout\r\n");
	}

	// Constrói payload
	ROCKET_Payload_t payload;
	ROCKET_BuildPayload(rocket, &payload);

	// Envia de forma assíncrona
	HAL_StatusTypeDef result = LORA_SendStruct(rocket->lora, &payload, sizeof(payload));

	if (result == HAL_OK) {
		rocket->status.transmission_count++;
		memcpy(&rocket->last_payload, &payload, sizeof(payload));

		for(int i = 0; i < 10; i++) {
			LORA_Manager(rocket->lora);
			HAL_Delay(1);
		}

		printf("[TX #%lu] T:%.4fs | Lat:%.6f Lon:%.6f | Alt:%.1f/%.1f\r\n",
			   rocket->status.transmission_count,
			   payload.time,
			   payload.lat, payload.lon,
			   payload.alt, payload.bar_alt);

		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		return HAL_OK;

	} else if (result == HAL_BUSY) {
		return HAL_BUSY;
	}

	return HAL_ERROR;
}

/* ========================================================================
 * NOVO: SISTEMA DE LOGGING UNIFICADO - IMPLEMENTAÇÃO
 * ======================================================================== */

HAL_StatusTypeDef ROCKET_Logger_Init(ROCKET_System_t *rocket)
{
    memset(&rocket->logger, 0, sizeof(ROCKET_Logger_t));

    rocket->logger.buffer.write_idx = 0;
    rocket->logger.buffer.count = 0;
    rocket->logger.buffer.overflow = 0;
    rocket->logger.buffer.lost_samples = 0;

    rocket->logger.log_buffer_pos = 0;
    rocket->logger.samples_written = 0;
    rocket->logger.flush_count = 0;
    rocket->logger.file_open = 0;
    rocket->logger.header_written = 0;

    printf("[LOGGER] Sistema inicializado (buffer: %d amostras)\r\n",
           FLIGHT_BUFFER_SIZE);

    return HAL_OK;
}

HAL_StatusTypeDef ROCKET_Logger_CreateFile(ROCKET_System_t *rocket, const char *filename)
{
    if (!rocket->status.sd_initialized) {
        printf("[LOGGER] SD não inicializado\r\n");
        return HAL_ERROR;
    }

    // Gera nome automático se não fornecido
    if (filename == NULL) {
        snprintf(rocket->logger.current_filename,
                sizeof(rocket->logger.current_filename),
                "flight_%lu.csv", HAL_GetTick());
    } else {
        strncpy(rocket->logger.current_filename, filename,
                sizeof(rocket->logger.current_filename) - 1);
    }

    // Abre arquivo
    if (SD_OpenFile(rocket->sd, rocket->logger.current_filename,
                    SD_MODE_APPEND) != HAL_OK) {
        printf("[LOGGER] Erro ao criar arquivo\r\n");
        return HAL_ERROR;
    }

    rocket->logger.file_open = 1;

    // Escreve cabeçalho completo COM MAGNETÔMETRO E UTC
    const char *header =
        "timestamp_ms,gnss_time,"
        "lat,lon,alt_gps,gnss_fix,gnss_sats,"
        "alt_bar,pressure,temperature,"
        "pos_n,pos_e,pos_d,"
        "vel_n,vel_e,vel_d,"
        "acc_n,acc_e,acc_d,"
        "acc_x_raw,acc_y_raw,acc_z_raw,"
        "gyro_x,gyro_y,gyro_z,"
        "mag_x,mag_y,mag_z,"
        "roll,pitch,yaw,"
        "flight_state,"
        "imu_valid,ekf_valid,bar_valid,attitude_reliable\n";

    SD_WriteLine(rocket->sd, header);
    SD_Sync(rocket->sd);

    rocket->logger.header_written = 1;

    printf("[LOGGER] Arquivo criado: %s\r\n", rocket->logger.current_filename);

    return HAL_OK;
}

HAL_StatusTypeDef ROCKET_Logger_AddSample(ROCKET_System_t *rocket)
{
    ROCKET_DataBuffer_t *buf = &rocket->logger.buffer;
    ROCKET_LogData_t *sample = &buf->samples[buf->write_idx];

    // Preenche estrutura de dados
    sample->timestamp_ms = HAL_GetTick();

    // GPS
    if (rocket->gnss->is_fixed && rocket->gnss->status == 'A') {
        sample->lat = (float)rocket->gnss->lat_decimal;
        sample->lon = (float)rocket->gnss->lon_decimal;
        sample->alt_gps = rocket->gnss->altitude;
        sample->gnss_fix = 1;
        sample->gnss_sats = rocket->gnss->satelites_usados;

        // NOVO: Copia hora UTC
        strncpy(sample->gnss_time, rocket->gnss->hora, sizeof(sample->gnss_time) - 1);
        sample->gnss_time[sizeof(sample->gnss_time) - 1] = '\0';
    } else {
        sample->lat = 0;
        sample->lon = 0;
        sample->alt_gps = 0;
        sample->gnss_fix = 0;
        sample->gnss_sats = rocket->gnss->satelites_usados;
        strcpy(sample->gnss_time, "N/A");
    }

    // Barômetro
    if (rocket->bar->is_valid) {
        sample->alt_bar = rocket->bar->altitude_m;
        sample->pressure = rocket->bar->pressure_hpa;
        sample->temperature = rocket->bar->temperature_c;
        sample->bar_valid = 1;
    } else {
        sample->alt_bar = 0;
        sample->pressure = 0;
        sample->temperature = 0;
        sample->bar_valid = 0;
    }

    // EKF - Posição e Velocidade NED
    if (rocket->use_ekf && rocket->status.ekf_initialized) {
        sample->pos_n = rocket->state.pos[0];
        sample->pos_e = rocket->state.pos[1];
        sample->pos_d = rocket->state.pos[2];

        sample->vel_n = rocket->state.vel[0];
        sample->vel_e = rocket->state.vel[1];
        sample->vel_d = rocket->state.vel[2];

        sample->ekf_valid = 1;
    } else {
        memset(&sample->pos_n, 0, 6 * sizeof(float));
        sample->ekf_valid = 0;
    }

    // IMU
    if (rocket->imu->is_valid) {
        // Aceleração NED (sem gravidade)
        sample->acc_n = rocket->imu->acc_ned_x - rocket->imu->acc_ned_offset_x;
        sample->acc_e = rocket->imu->acc_ned_y - rocket->imu->acc_ned_offset_y;
        sample->acc_d = rocket->imu->acc_ned_z - rocket->imu->acc_ned_offset_z;

        // Aceleração Body (raw)
        sample->acc_x_raw = rocket->imu->acc_x;
        sample->acc_y_raw = rocket->imu->acc_y;
        sample->acc_z_raw = rocket->imu->acc_z;

        // Giroscópio
        sample->gyro_x = rocket->imu->gyro_x;
        sample->gyro_y = rocket->imu->gyro_y;
        sample->gyro_z = rocket->imu->gyro_z;

        // NOVO: Magnetômetro
        sample->mag_x = rocket->imu->mag_x;
        sample->mag_y = rocket->imu->mag_y;
        sample->mag_z = rocket->imu->mag_z;

        // Atitude (converter para graus)
        if (rocket->status.fusion_initialized && rocket->fusion != NULL) {
            sample->roll = rocket->fusion->roll_mw * 180.0f / M_PI;
            sample->pitch = rocket->fusion->pitch_mw * 180.0f / M_PI;
            sample->yaw = rocket->fusion->yaw_mw * 180.0f / M_PI;
        } else {
            sample->roll = rocket->imu->roll * 180.0f / M_PI;
            sample->pitch = rocket->imu->pitch * 180.0f / M_PI;
            sample->yaw = rocket->imu->yaw * 180.0f / M_PI;
        }

        sample->imu_valid = 1;
        sample->attitude_reliable = rocket->imu->attitude_reliable;  // NOVO
    } else {
        memset(&sample->acc_n, 0, 15 * sizeof(float));
        sample->imu_valid = 0;
        sample->attitude_reliable = 0;
    }

    // FSM
    sample->flight_state = rocket->fsm->current_state;

    // Atualiza índices do buffer circular
    buf->write_idx = (buf->write_idx + 1) % FLIGHT_BUFFER_SIZE;

    if (buf->count < FLIGHT_BUFFER_SIZE) {
        buf->count++;
    } else {
        buf->overflow = 1;
        buf->lost_samples++;
    }

    return HAL_OK;
}

void ROCKET_Logger_DataToCSV(ROCKET_LogData_t *data, char *buffer, size_t size)
{
    // ✅ Use conversão incremental para evitar overflow de heap
    int pos = 0;

    // Timestamp + UTC
    pos += snprintf(buffer + pos, size - pos, "%lu,%s,",
                    data->timestamp_ms, data->gnss_time);

    // GPS (3 floats por vez)
    pos += snprintf(buffer + pos, size - pos, "%.8f,%.8f,%.2f,%d,%d,",
                    data->lat, data->lon, data->alt_gps,
                    data->gnss_fix, data->gnss_sats);

    // Barômetro
    pos += snprintf(buffer + pos, size - pos, "%.2f,%.2f,%.2f,",
                    data->alt_bar, data->pressure, data->temperature);

    // EKF posição
    pos += snprintf(buffer + pos, size - pos, "%.4f,%.4f,%.4f,",
                    data->pos_n, data->pos_e, data->pos_d);

    // EKF velocidade
    pos += snprintf(buffer + pos, size - pos, "%.4f,%.4f,%.4f,",
                    data->vel_n, data->vel_e, data->vel_d);

    // IMU aceleração NED
    pos += snprintf(buffer + pos, size - pos, "%.4f,%.4f,%.4f,",
                    data->acc_n, data->acc_e, data->acc_d);

    // IMU aceleração raw
    pos += snprintf(buffer + pos, size - pos, "%.4f,%.4f,%.4f,",
                    data->acc_x_raw, data->acc_y_raw, data->acc_z_raw);

    // IMU giroscópio
    pos += snprintf(buffer + pos, size - pos, "%.4f,%.4f,%.4f,",
                    data->gyro_x, data->gyro_y, data->gyro_z);

    // IMU magnetômetro
    pos += snprintf(buffer + pos, size - pos, "%.4f,%.4f,%.4f,",
                    data->mag_x, data->mag_y, data->mag_z);

    // Atitude
    pos += snprintf(buffer + pos, size - pos, "%.2f,%.2f,%.2f,",
                    data->roll, data->pitch, data->yaw);

    // Flight state
    pos += snprintf(buffer + pos, size - pos, "%d\n", data->flight_state);

    // Proteção contra overflow
    if (pos >= size - 1) {
        printf("[WARN] CSV truncado! Aumente buffer\r\n");
    }
}

HAL_StatusTypeDef ROCKET_Logger_Flush(ROCKET_System_t *rocket)
{
    if (!rocket->logger.file_open || rocket->logger.buffer.count == 0) {
        return HAL_OK;
    }

    uint32_t start_time = HAL_GetTick();
    ROCKET_DataBuffer_t *buf = &rocket->logger.buffer;

    // ✅ Use o buffer do logger (já existe!)
    static char line_buffer[512];  // Estático = fora da stack

    // Reabre arquivo se necessário
    if (!rocket->sd->is_file_open) {
        if (SD_OpenFile(rocket->sd, rocket->logger.current_filename,
                        SD_MODE_APPEND) != HAL_OK) {
            printf("[LOGGER] Erro ao reabrir arquivo\r\n");
            buf->count = 0;
            buf->overflow = 0;
            return HAL_ERROR;
        }
    }

    uint16_t samples_to_write = buf->count;

    uint16_t start_idx = (buf->write_idx >= buf->count) ?
                         (buf->write_idx - buf->count) :
                         (FLIGHT_BUFFER_SIZE - (buf->count - buf->write_idx));

    uint16_t successful_writes = 0;

    for (uint16_t i = 0; i < buf->count; i++) {
        uint16_t idx = (start_idx + i) % FLIGHT_BUFFER_SIZE;
        ROCKET_LogData_t *sample = &buf->samples[idx];

        // ✅ Agora usa buffer estático (seguro!)
        ROCKET_Logger_DataToCSV(sample, line_buffer, sizeof(line_buffer));

        if (SD_WriteLine(rocket->sd, line_buffer) == HAL_OK) {
            rocket->logger.samples_written++;
            successful_writes++;
        } else {
            printf("[LOGGER] Erro linha %d - continuando...\r\n", i);
        }
    }

    // Sync para garantir gravação
    static uint8_t flush_counter = 0;
    flush_counter++;

    if (flush_counter >= 5 || rocket->fsm->current_state == FLIGHT_APOGEE) {
        SD_Sync(rocket->sd);  // Força gravação física
        flush_counter = 0;
    }

    // ===== MODIFICADO: SEMPRE LIMPA BUFFER =====
    buf->count = 0;
    buf->overflow = 0;
    // ============================================

    rocket->logger.flush_count++;
    rocket->logger.last_flush = HAL_GetTick();

//    uint32_t duration = HAL_GetTick() - start_time;
//
//    if (duration > 100) {
//        printf("[LOGGER] Flush lento: %lu ms (%d/%d ok)\r\n",
//               duration, successful_writes, buf->count);
//    }

    return (successful_writes > 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef ROCKET_Logger_ForceFlush(ROCKET_System_t *rocket)
{
    printf("[LOGGER] Flush forçado\r\n");
    return ROCKET_Logger_Flush(rocket);
}

HAL_StatusTypeDef ROCKET_Logger_Close(ROCKET_System_t *rocket)
{
    if (!rocket->logger.file_open) {
        return HAL_OK;
    }

    // Flush final
    ROCKET_Logger_Flush(rocket);

    // Escreve estatísticas no final do arquivo
    char stats[256];
    snprintf(stats, sizeof(stats),
        "\n# Estatísticas\n"
        "# Total de amostras: %lu\n"
        "# Flushes: %lu\n"
        "# Amostras perdidas: %lu\n",
        rocket->logger.samples_written,
        rocket->logger.flush_count,
        rocket->logger.buffer.lost_samples);

    SD_WriteLine(rocket->sd, stats);
    SD_CloseFile(rocket->sd);

    rocket->logger.file_open = 0;

    printf("[LOGGER] Arquivo fechado: %s\r\n", rocket->logger.current_filename);
    ROCKET_Logger_PrintStats(rocket);

    return HAL_OK;
}

void ROCKET_Logger_PrintStats(ROCKET_System_t *rocket)
{
    printf("\r\n========== ESTATÍSTICAS DO LOGGER ==========\r\n");
    printf("Arquivo: %s\r\n", rocket->logger.current_filename);
    printf("Amostras gravadas: %lu\r\n", rocket->logger.samples_written);
    printf("Flushes realizados: %lu\r\n", rocket->logger.flush_count);
    printf("Amostras perdidas: %lu\r\n", rocket->logger.buffer.lost_samples);
    printf("Taxa de perda: %.2f%%\r\n",
           rocket->logger.samples_written > 0 ?
           (rocket->logger.buffer.lost_samples * 100.0f) /
           (rocket->logger.samples_written + rocket->logger.buffer.lost_samples) : 0);
    printf("Buffer atual: %d/%d amostras\r\n",
           rocket->logger.buffer.count, FLIGHT_BUFFER_SIZE);
    printf("============================================\r\n\r\n");
}

/* ========================================================================
 * GRAVAÇÃO (DEPRECIADO)
 * ======================================================================== */
HAL_StatusTypeDef ROCKET_CollectMagCalibrationData(ROCKET_System_t *rocket,
													uint32_t duration_ms)
{

	if (rocket->status.imu_initialized!=1) {
		printf("[ERRO] IMU nao inicializado\r\n");
		return HAL_ERROR;
	}

	if (rocket->status.sd_initialized!=1) {
		printf("[INFO] Montando SD...\r\n");
		if (SD_Mount(rocket->sd) != HAL_OK) {
			printf("[ERRO] SD nao disponivel\r\n");
			return HAL_ERROR;
		}
		rocket->status.sd_initialized = 1;
	}

	return IMU_CollectMagDataToSD(rocket->imu, rocket->sd, duration_ms, "MagData.txt");
}

HAL_StatusTypeDef ROCKET_CreateLogFile(ROCKET_System_t *rocket, const char *filename)
{
//	if (!rocket->status.sd_initialized) {
//		return HAL_ERROR;
//	}
//
//	if (SD_FileExists(rocket->sd, filename) == HAL_OK) {
//		return HAL_OK;
//	}
//
//	if (SD_OpenFile(rocket->sd, filename, SD_MODE_APPEND) != HAL_OK) {
//		return HAL_ERROR;
//	}
//
//	SD_WriteCSVHeader(rocket->sd,
//		"time,lat,lon,alt_gps,alt_bar,veld,veln,vele,roll,pitch,yaw,"
//		"acc_x,acc_y,acc_z,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,"
//		"gnss_fix,imu_valid,tx_count");
//	SD_CloseFile(rocket->sd);
//
//	return HAL_OK;
	return ROCKET_Logger_CreateFile(rocket, filename);
}

HAL_StatusTypeDef ROCKET_LogToSD(ROCKET_System_t *rocket)
{
//	if (!rocket->status.sd_initialized) {
//		if (SD_Mount(rocket->sd) != HAL_OK) {
//			return HAL_ERROR;
//		}
//		rocket->status.sd_initialized = 1;
//	}
//
//	ROCKET_Payload_t *p = &rocket->last_payload;
//	ROCKET_State *s = &rocket->state;
//
//	if (SD_QuickWriteFormatted(rocket->sd, "telemetry.csv",
//		"%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
//		"%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
//		"%d,%d,%lu\n",
//		p->time, p->lat, p->lon, p->alt, p->bar_alt,
//		p->veld, p->veln, p->vele,
//		p->roll, p->pitch, p->yaw,
//		p->acc_x, p->acc_y, p->acc_z,
//		s->pos[0], s->pos[1], s->pos[2],
//		s->vel[0], s->vel[1], s->vel[2],
//		p->gnss_fix, p->imu_valid,
//		rocket->status.transmission_count) == HAL_OK) {
//
//		rocket->status.sd_write_count++;
//		return HAL_OK;
//	}
//
//	return HAL_ERROR;
	return ROCKET_Logger_AddSample(rocket);
}

HAL_StatusTypeDef ROCKET_CreateEKFLogFile(ROCKET_System_t *rocket, const char *filename)
{
	if (!rocket->status.sd_initialized) {
		return HAL_ERROR;
	}

	if (SD_FileExists(rocket->sd, filename) == HAL_OK) {
		return HAL_OK;
	}

	if (SD_OpenFile(rocket->sd, filename, SD_MODE_APPEND) != HAL_OK) {
		return HAL_ERROR;
	}

	// Cabeçalho CSV expandido para análise EKF
	SD_WriteCSVHeader(rocket->sd,
		"time,"
		"gps_lat,gps_lon,gps_alt,gps_valid,"
		"baro_alt,baro_valid,"
		"ekf_x,ekf_y,ekf_z,"
		"ekf_vx,ekf_vy,ekf_vz,"
		"ekf_P_xx,ekf_P_yy,ekf_P_zz,ekf_P_vxvx,ekf_P_vyvy,ekf_P_vzvz,"
		"imu_acc_x,imu_acc_y,imu_acc_z,"
		"imu_gyro_x,imu_gyro_y,imu_gyro_z,"
		"imu_roll,imu_pitch,imu_yaw,"
		"imu_valid");

	SD_CloseFile(rocket->sd);

	return HAL_OK;
}

HAL_StatusTypeDef ROCKET_LogEKFToSD(ROCKET_System_t *rocket)
{
//	if (!rocket->status.sd_initialized) {
//		if (SD_Mount(rocket->sd) != HAL_OK) {
//			return HAL_ERROR;
//		}
//		rocket->status.sd_initialized = 1;
//	}
//
//	ROCKET_State *s = &rocket->state;
//
//	if (SD_QuickWriteFormatted(rocket->sd, "ekf_log.csv",
//		"%.3f,"                                  // time
//		"%.8f,%.8f,%.2f,%d,"                    // GPS
//		"%.2f,%d,"                              // Barômetro
//		"%.4f,%.4f,%.4f,"                       // EKF posição
//		"%.4f,%.4f,%.4f,"                       // EKF velocidade
//		"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"       // EKF covariância
//		"%.4f,%.4f,%.4f,"                       // IMU aceleração
//		"%.4f,%.4f,%.4f,"                       // IMU giroscópio
//		"%.4f,%.4f,%.4f,"                       // IMU atitude
//		"%d\n",                                  // IMU válido
//
//		HAL_GetTick() / 1000.0f,
//		rocket->gnss->lat_decimal, rocket->gnss->lon_decimal, rocket->gnss->altitude, s->gps_valid,
//		rocket->bar->altitude_m, s->baro_valid,
//		s->pos[0], s->pos[1], s->pos[2],
//		s->vel[0], s->vel[1], s->vel[2],
//		rocket->ekf.P[0][0], rocket->ekf.P[1][1], rocket->ekf.P[2][2],
//		rocket->ekf.P[3][3], rocket->ekf.P[4][4], rocket->ekf.P[5][5],
//		rocket->imu->acc_ned_x, rocket->imu->acc_ned_y, rocket->imu->acc_ned_z,
//		rocket->imu->gyro_x, rocket->imu->gyro_y, rocket->imu->gyro_z,
//		s->roll * 180/M_PI, s->pitch * 180/M_PI, s->yaw * 180/M_PI,
//		s->imu_valid) == HAL_OK) {
//
//		return HAL_OK;
//	}
//
//	return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef ROCKET_LogToSD_Smart(ROCKET_System_t *rocket, ROCKET_FSM_t *fsm)
{
    if (!rocket->status.sd_initialized) return HAL_ERROR;

    // Dados essenciais (sempre)
    static char log_buffer[512];
    int len = snprintf(log_buffer, sizeof(log_buffer),
        "%.3f,%s,%.6f,%.6f,%.2f,%.2f,",
        HAL_GetTick() / 1000.0f,
        ROCKET_FSM_GetStateName(fsm->current_state),
        rocket->gnss->lat_decimal,
        rocket->gnss->lon_decimal,
        rocket->gnss->altitude,
        rocket->bar->altitude_m);

    // Dados extras em voo crítico
    if (ROCKET_FSM_IsFlying(fsm)) {
        len += snprintf(log_buffer + len, sizeof(log_buffer) - len,
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,",
            rocket->state.vel[0], rocket->state.vel[1], rocket->state.vel[2],
            rocket->imu->acc_ned_x, rocket->imu->acc_ned_y, rocket->imu->acc_ned_z,
            rocket->imu->roll * 180/M_PI,
            rocket->imu->pitch * 180/M_PI,
            rocket->imu->yaw * 180/M_PI);
    }

    // Estatísticas em tempo real
    len += snprintf(log_buffer + len, sizeof(log_buffer) - len,
        "%.1f,%.2f,%d\n",
        fsm->stats.max_altitude,
        fsm->stats.max_gforce,
        fsm->current_state);

    return SD_QuickWriteLine(rocket->sd, "flight_data.csv", log_buffer);
}

/* ========================================================================
 * CALIBRAÇÃO
 * ======================================================================== */

HAL_StatusTypeDef ROCKET_CalibrateIMU(ROCKET_System_t *rocket, uint16_t samples)
{
	if (!rocket->status.imu_initialized) {
		return HAL_ERROR;
	}

	return IMU_Calibrate(rocket->imu, samples);
}

void ROCKET_SetEKFNoiseParameters(ROCKET_System_t *rocket,
								  float q_pos, float q_vel,
								  float r_gps, float r_baro)
{
	for (int i = 0; i < 3; i++) {
		rocket->ekf.Q[i][i] = q_pos;
		rocket->ekf.Q[i+3][i+3] = q_vel;
		rocket->ekf.R_gps[i][i] = r_gps;
	}
	rocket->ekf.R_baro = r_baro;
}

void ROCKET_AutoCalibrateOnPad(ROCKET_System_t *rocket)
{
    printf("[CALIBRAÇÃO] Iniciando calibração na rampa...\r\n");
    printf("[INFO] Mantenha o foguete IMÓVEL por 10 segundos\r\n");

    // 10 segundos de calibração
    for (int i = 10; i > 0; i--) {
        printf("[%d] ", i);
        HAL_Delay(1000);
    }

    // Calibra IMU
    ROCKET_CalibrateIMU(rocket, 200);

    // Calibra barômetro (referência de altitude)
    float baro_sum = 0;
    for (int i = 0; i < 50; i++) {
        BAR_Update(rocket->bar);
        baro_sum += rocket->bar->altitude_m;
        HAL_Delay(20);
    }
    rocket->bar->altitude_inicial = baro_sum / 50.0f;

    // Salva referência GPS
    if (rocket->gnss->is_fixed) {
        GNSS_SaveInitialPosition(rocket->gnss);
        printf("[OK] Posição GPS salva: %.6f, %.6f\r\n",
               rocket->gnss->lat_decimal, rocket->gnss->lon_decimal);
    }

    printf("[OK] Calibração concluída!\r\n");
    printf("[INFO] Sistema pronto para voo\r\n");
}

/* ========================================================================
 * FUNÇÕES AUXILIARES
 * ======================================================================== */
uint8_t ROCKET_DetectApogee_Redundant(ROCKET_System_t *rocket, ROCKET_FSM_t *fsm)
{
    uint8_t votes = 0;
    uint8_t sensors_available = 0;

    // Método 1: EKF (velocidade vertical)
    if (rocket->use_ekf && rocket->status.ekf_initialized) {
        sensors_available++;
        if (rocket->state.vel[2] > -2.0f) {  // NED: negativo = subindo
            votes++;
        }
    }

    // Método 2: Barômetro (queda de altitude)
    if (rocket->bar->is_valid) {
        sensors_available++;
        static float max_baro_alt = 0;
        if (rocket->bar->altitude_m > max_baro_alt) {
            max_baro_alt = rocket->bar->altitude_m;
        }
        if ((max_baro_alt - rocket->bar->altitude_m) > 3.0f) {
            votes++;
        }
    }

    // Método 3: IMU (aceleração para baixo sustentada)
    if (rocket->imu->is_valid) {
        sensors_available++;
        if (rocket->imu->acc_ned_z > 5.0f) {  // Gravidade + desaceleração
            votes++;
        }
    }

    // Método 4: GPS (velocidade baixa) - menos confiável
    if (rocket->gnss->is_fixed && rocket->gnss->velocidade_kmh < 50.0f) {
        sensors_available++;
        votes++;
    }

    // Requer maioria (≥50%) + mínimo de 2 sensores
    if (sensors_available >= 2 && votes >= (sensors_available / 2 + 1)) {
        printf("[APOGEE] Detectado por voting: %d/%d sensores\r\n",
               votes, sensors_available);
        return 1;
    }

    // Timeout de segurança
    float flight_time = (HAL_GetTick() - fsm->stats.time_liftoff) / 1000.0f;
    if (flight_time > fsm->params.timeout_apogee) {
        printf("[APOGEE] FORÇADO por timeout (%.2fs)\r\n", flight_time);
        return 1;
    }

    return 0;
}

void ROCKET_AdjustTimingsForPhase(ROCKET_System_t *rocket, FlightState_t phase)
{
	switch(phase) {
		case FLIGHT_IDLE:
		case FLIGHT_ARMED:
			// Economia de energia
			imu_interval = 50;      // 20Hz
			bar_interval = 500;     // 2Hz
			ekf_interval = 100;     // 10Hz
			tx_interval = 2000;     // 0.5Hz
			break;

		case FLIGHT_BOOST:
		case FLIGHT_COAST:
			// MÁXIMA PERFORMANCE
			imu_interval = 5;       // 200Hz
			bar_interval = 20;      // 50Hz
			ekf_interval = 10;      // 100Hz
			tx_interval = 100;      // 10Hz (telemetria crítica!)
			break;

		case FLIGHT_APOGEE:
		case FLIGHT_DROGUE_DESCENT:
			// Alta taxa para controle
			imu_interval = 10;      // 100Hz
			bar_interval = 50;      // 20Hz
			ekf_interval = 20;      // 50Hz
			tx_interval = 200;      // 5Hz
			break;

		case FLIGHT_MAIN_DESCENT:
		case FLIGHT_LANDED:
			// Taxa reduzida
			imu_interval = 20;      // 50Hz
			bar_interval = 100;     // 10Hz
			ekf_interval = 50;      // 20Hz
			tx_interval = 1000;     // 1Hz
			break;
		case FLIGHT_ERROR:
	}
}

void ROCKET_CheckGNSS_Smart(ROCKET_System_t *rocket, ROCKET_FSM_t *fsm)
{
	uint32_t time_since_update = HAL_GetTick() - rocket->gnss->last_update;
	uint32_t timeout;

	// Timeout adaptativo por fase
	switch(fsm->current_state) {
		case FLIGHT_BOOST:
		case FLIGHT_COAST:
			timeout = 500;  // 0.5s - GPS pode perder durante alta aceleração
			break;

		case FLIGHT_APOGEE:
		case FLIGHT_DROGUE_DESCENT:
			timeout = 1000; // 1s
			break;

		default:
			timeout = 3000; // 3s - padrão
			break;
	}

	if (time_since_update > timeout) {
		printf("[AVISO] GNSS timeout (%lums) - Estado: %s\r\n",
			   time_since_update, ROCKET_FSM_GetStateName(fsm->current_state));

		// NÃO reinicia durante voo crítico - confia no EKF
		if (fsm->current_state == FLIGHT_IDLE ||
			fsm->current_state == FLIGHT_ARMED ||
			fsm->current_state == FLIGHT_LANDED) {
			GNSS_StartDMA(rocket->gnss);
		}
	}
}

void ROCKET_PrintEKFForMatlab(ROCKET_System_t *rocket)
{
	ROCKET_State *s = &rocket->state;

	// Formato: EKF_DATA,timestamp,dados...
	// Facilita parsing no MATLAB
	printf("EKF_DATA,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%d,%d\n",
		   HAL_GetTick() / 1000.0f,
		   s->pos[0], s->pos[1], s->pos[2],
		   s->vel[0], s->vel[1], s->vel[2],
		   rocket->ekf.P[0][0], rocket->ekf.P[1][1], rocket->ekf.P[2][2],
		   s->gps_valid, s->baro_valid);
}

void ROCKET_PrintInfo(ROCKET_System_t *rocket)
{
	printf("\r\n========== STATUS DO SISTEMA ==========\r\n");
	printf("Sistema Pronto: %s\r\n", rocket->status.system_ready ? "SIM" : "NAO");

	printf("\r\n--- Perifericos ---\r\n");
	printf("LoRa: %s\r\n", rocket->status.lora_initialized ? "OK" : "FALHA");
	printf("GNSS: %s (Fix: %s)\r\n",
		   rocket->status.gnss_initialized ? "OK" : "FALHA",
		   rocket->gnss->is_fixed ? "SIM" : "NAO");
	printf("SD Card: %s\r\n", rocket->status.sd_initialized ? "OK" : "FALHA");
	printf("Barometro: %s\r\n", rocket->status.bar_initialized ? "OK" : "FALHA");
	printf("IMU: %s (Cal: %s)\r\n",
		   rocket->status.imu_initialized ? "OK" : "FALHA",
		   (rocket->status.imu_initialized && rocket->imu->is_calibrated) ? "SIM" : "NAO");
	printf("Fusao IMU: %s\r\n", rocket->status.fusion_initialized ? "OK" : "NAO");
	printf("EKF: %s\r\n", rocket->status.ekf_initialized ? "OK" : "NAO");

	printf("\r\n--- Estatisticas ---\r\n");
	printf("Transmissoes: %lu\r\n", rocket->status.transmission_count);
	printf("Gravacoes SD: %lu\r\n", rocket->status.sd_write_count);

	printf("\r\n--- Ultimo Payload ---\r\n");
	printf("Tempo: %.2f s\r\n", rocket->last_payload.time);
	printf("Posicao: %.6f, %.6f\r\n", rocket->last_payload.lat, rocket->last_payload.lon);
	printf("Altitude: %.1f m (GPS) / %.1f m (BAR)\r\n",
		   rocket->last_payload.alt, rocket->last_payload.bar_alt);
	if (rocket->last_payload.imu_valid) {
		printf("Atitude: R:%.1f P:%.1f Y:%.1f deg\r\n",
			   rocket->last_payload.roll, rocket->last_payload.pitch, rocket->last_payload.yaw);
		printf("Aceleracao: X:%.2f Y:%.2f Z:%.2f m/s2\r\n",
			   rocket->last_payload.acc_x, rocket->last_payload.acc_y, rocket->last_payload.acc_z);
	}
	printf("========================================\r\n\r\n");
}

void ROCKET_PrintState(ROCKET_System_t *rocket)
{
	ROCKET_State *s = &rocket->state;

	printf("\r\n========== ESTADO DO FOGUETE (EKF) ==========\r\n");
	printf("Posicao NED: [%.2f, %.2f, %.2f] m\r\n", s->pos[0], s->pos[1], s->pos[2]);
	printf("Velocidade NED: [%.2f, %.2f, %.2f] m/s\r\n", s->vel[0], s->vel[1], s->vel[2]);
	printf("Aceleracao NED: [%.2f, %.2f, %.2f] m/s2\r\n", s->acc[0], s->acc[1], s->acc[2]);
	printf("Atitude: R:%.1f P:%.1f Y:%.1f deg\r\n",
		   s->roll*180/M_PI, s->pitch*180/M_PI, s->yaw*180/M_PI);
	printf("Sensores validos: GPS:%d BAR:%d IMU:%d\r\n",
		   s->gps_valid, s->baro_valid, s->imu_valid);
	printf("=============================================\r\n\r\n");
}

uint8_t ROCKET_CheckPeripherals(ROCKET_System_t *rocket)
{
	uint8_t all_ok = 1;

	if (!rocket->status.lora_initialized) all_ok = 0;
	if (!rocket->status.gnss_initialized) all_ok = 0;

	return all_ok;
}

void ROCKET_SetTransmissionInterval(ROCKET_System_t *rocket, uint32_t interval_ms)
{
	tx_interval = interval_ms;
}

void ROCKET_SetSDWriteInterval(ROCKET_System_t *rocket, uint32_t interval_ms)
{
	sd_interval = interval_ms;
}

void ROCKET_EnableEKF(ROCKET_System_t *rocket, uint8_t enable)
{
	rocket->use_ekf = enable;

	if (enable && !rocket->status.ekf_initialized) {
		ROCKET_EKF_Init(&rocket->ekf, 1.0f / rocket->ekf_update_rate);
		rocket->status.ekf_initialized = 1;
		printf("[INFO] EKF habilitado\r\n");
	} else if (!enable) {
		printf("[INFO] EKF desabilitado\r\n");
	}
}

HAL_StatusTypeDef ROCKET_InjectGPSAidData(ROCKET_System_t *rocket,
										  double lat, double lon, float alt,
										  uint16_t year, uint8_t month, uint8_t day,
										  uint8_t hour, uint8_t minute, uint8_t second)
{
	if (!rocket->status.gnss_initialized) {
		printf("[ROCKET] GPS nao inicializado\r\n");
		return HAL_ERROR;
	}

	return GNSS_InjectAidData(rocket->gnss, lat, lon, alt,
							 year, month, day, hour, minute, second);
}

/* ========================================================================
 * AUXILIARES
 * ======================================================================== */

void GNSS_SetFlightMode(GNSS *gnss, FlightState_t phase)
{
    extern UART_HandleTypeDef GNSSuart;

    switch(phase) {
        case FLIGHT_IDLE:
        case FLIGHT_ARMED:
            // 1Hz para economizar energia
            GNSS_SendCommand(&GNSSuart, "PMTK220,1000");
            break;

        case FLIGHT_BOOST:
            // 5Hz suficiente
            GNSS_SendCommand(&GNSSuart, "PMTK220,200");
            break;
        case FLIGHT_COAST:
            break;
        case FLIGHT_APOGEE:
        	break;
        case FLIGHT_MAIN_DESCENT:
        	break;
        case FLIGHT_DROGUE_DESCENT:
            break;
        case FLIGHT_LANDED:
            // 1Hz
            GNSS_SendCommand(&GNSSuart, "PMTK220,1000");
            break;
        case FLIGHT_ERROR:
        	break;
    }
}

void BAR_SetModeForPhase(BAR *bar, FlightState_t phase)
{
    switch(phase) {
        case FLIGHT_IDLE:
        case FLIGHT_ARMED:
            bar->measure_mode = BAR_MODE_LOW_POWER;
            bar->conversion_delay_ms = 2;
            break;

        case FLIGHT_BOOST:
            // Barômetro não é confiável, mas usa modo rápido
            bar->measure_mode = BAR_MODE_NORMAL;
            bar->conversion_delay_ms = 7;
            break;

        case FLIGHT_COAST:
        case FLIGHT_DROGUE_DESCENT:
        case FLIGHT_MAIN_DESCENT:
            // Tracking preciso
            bar->measure_mode = BAR_MODE_LOW_NOISE;
            bar->conversion_delay_ms = 25;
            break;

        case FLIGHT_APOGEE:
            // MÁXIMA PRECISÃO para detecção
            bar->measure_mode = BAR_MODE_ULTRA_LOW_NOISE;
            bar->conversion_delay_ms = 95;
            break;

        case FLIGHT_LANDED:
            bar->measure_mode = BAR_MODE_LOW_POWER;
            bar->conversion_delay_ms = 2;
            break;
        case FLIGHT_ERROR:

    }
}

float BAR_GetFilteredAltitude(BAR *bar, BAR_Filter_t *filter, ROCKET_FSM_t *fsm)
{
    float current = bar->altitude_m;

    // Durante boost, barômetro não é confiável
    if (fsm->current_state == FLIGHT_BOOST) {
        return filter->last_valid;
    }

    // Adiciona ao buffer
    filter->readings[filter->index] = current;
    filter->index = (filter->index + 1) % 10;
    if (filter->count < 10) filter->count++;

    // Mediana (mais robusto que média)
    float sorted[10];
    memcpy(sorted, filter->readings, filter->count * sizeof(float));

    // Bubble sort simples
    for (int i = 0; i < filter->count - 1; i++) {
        for (int j = 0; j < filter->count - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    float median = sorted[filter->count / 2];
    filter->last_valid = median;
    return median;
}

