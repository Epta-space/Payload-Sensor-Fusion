/*
 * ROCKET_FSM.h
 *
 *  Created on: Oct 12, 2025
 *      Author: de4lerr
 */

#include "main.h"
#include <stdio.h>

//#include "ROCKET.h"


#ifndef INC_ROCKET_FSM_H_
#define INC_ROCKET_FSM_H_

typedef struct ROCKET_System_s ROCKET_System_t;
typedef struct ROCKET_Payload_s ROCKET_Payload_t;
typedef struct SD_s SD;

/* ========================================================================
 * ESTADOS DO FOGUETE
 * ======================================================================== */

typedef enum {
    FLIGHT_IDLE = 0,           // Sistema ligado, aguardando
    FLIGHT_ARMED,              // RBF removido, pronto para lançamento
    FLIGHT_BOOST,              // Motor queimando (0-1.54s)
    FLIGHT_COAST,              // Subida inercial pós-burnout
    FLIGHT_APOGEE,             // Apogeu detectado
    FLIGHT_DROGUE_DESCENT,     // Descida com paraquedas drogue
    FLIGHT_MAIN_DESCENT,       // Descida com paraquedas principal
    FLIGHT_LANDED,             // Foguete pousado
    FLIGHT_ERROR               // Estado de erro
} FlightState_t;

/* ========================================================================
 * PARÂMETROS DE DETECÇÃO
 * ======================================================================== */

typedef struct {
    /* Detecção de Liftoff */
    float liftoff_acc_threshold;      // [g] - Aceleração mínima (default: 2.0g)
    uint16_t liftoff_acc_samples;     // Amostras consecutivas (default: 5)

    /* Detecção de Burnout */
    float burnout_acc_threshold;      // [g] - Queda de aceleração (default: 0.5g)
    float burnout_time_max;           // [s] - Tempo máximo de queima (default: 2.0s)

    /* Detecção de Apogeu */
    float apogee_vel_threshold;       // [m/s] - Velocidade vertical (default: 2.0 m/s)
    uint16_t apogee_vel_samples;      // Amostras (default: 10)
    float apogee_alt_drop;            // [m] - Queda de altitude (default: 3.0m)

    /* Detecção de Pouso */
    float landing_vel_threshold;      // [m/s] - Velocidade (default: 1.0 m/s)
    float landing_acc_threshold;      // [g] - Aceleração (default: 0.2g)
    uint16_t landing_time;            // [ms] - Tempo estável (default: 3000ms)

    /* Segurança */
    float max_altitude;               // [m] - Altitude máxima esperada
    float timeout_apogee;             // [s] - Timeout para forçar apogeu

} FlightDetectionParams_t;

/* ========================================================================
 * ESTATÍSTICAS DE VOO
 * ======================================================================== */

typedef struct {
    /* Tempos de Eventos */
    uint32_t time_armed;
    uint32_t time_liftoff;
    uint32_t time_burnout;
    uint32_t time_apogee;
    uint32_t time_drogue;
    uint32_t time_main;
    uint32_t time_landed;

    /* Máximos Registrados */
    float max_altitude;
    float max_velocity;
    float max_acceleration;
    float max_gforce;

    /* Dados no Apogeu */
    float apogee_altitude;
    float apogee_time;

    /* Performance */
    float avg_descent_rate;
    float landing_velocity;

} FlightStatistics_t;

/* ========================================================================
 * ESTRUTURA PRINCIPAL FSM
 * ======================================================================== */

typedef struct ROCKET_FSM_s {
    /* Estado Atual */
    FlightState_t current_state;
    FlightState_t previous_state;
    uint32_t state_entry_time;

    /* Parâmetros */
    FlightDetectionParams_t params;

    /* Estatísticas */
    FlightStatistics_t stats;

    /* Contadores para Detecção */
    uint16_t liftoff_counter;
    uint16_t apogee_counter;
    uint16_t landing_counter;

    /* Flags */
    uint8_t motor_burnout_detected;
    uint8_t apogee_detected;
    uint8_t parachute_deployed;

    /* Histórico para Análise */
    float altitude_history[20];
    float velocity_history[20];
    uint8_t history_index;

} ROCKET_FSM_t;

/* ========================================================================
 * FUNÇÕES PRINCIPAIS
 * ======================================================================== */

/**
 * @brief Inicializa a máquina de estados
 */
void ROCKET_FSM_Init(ROCKET_FSM_t *fsm);

/**
 * @brief Atualiza a máquina de estados (chamar a cada ciclo)
 */
void ROCKET_FSM_Update(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket);

/**
 * @brief Força transição de estado (para testes/emergência)
 */
void ROCKET_FSM_SetState(ROCKET_FSM_t *fsm, FlightState_t new_state);

/**
 * @brief Retorna string do estado atual
 */
const char* ROCKET_FSM_GetStateName(FlightState_t state);

/**
 * @brief Verifica se está em voo ativo
 */
uint8_t ROCKET_FSM_IsFlying(ROCKET_FSM_t *fsm);

/**
 * @brief Retorna tempo no estado atual
 */
float ROCKET_FSM_GetStateTime(ROCKET_FSM_t *fsm);

/* ========================================================================
 * FUNÇÕES DE DETECÇÃO
 * ======================================================================== */

/**
 * @brief Detecta decolagem baseado em aceleração
 */
uint8_t ROCKET_FSM_DetectLiftoff(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket);

/**
 * @brief Detecta burnout do motor
 */
uint8_t ROCKET_FSM_DetectBurnout(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket);

/**
 * @brief Detecta apogeu (múltiplos métodos)
 */
uint8_t ROCKET_FSM_DetectApogee(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket);

/**
 * @brief Detecta pouso
 */
uint8_t ROCKET_FSM_DetectLanding(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket);

/* ========================================================================
 * FUNÇÕES DE CONFIGURAÇÃO
 * ======================================================================== */

/**
 * @brief Configura parâmetros para seu perfil de voo
 */
void ROCKET_FSM_ConfigureForProfile(ROCKET_FSM_t *fsm,
                                    float thrust_time,
                                    float expected_apogee,
                                    float max_gforce);

/**
 * @brief Imprime estatísticas de voo
 */
void ROCKET_FSM_PrintStatistics(ROCKET_FSM_t *fsm);

/**
 * @brief Salva resumo do voo no SD
 */
HAL_StatusTypeDef ROCKET_FSM_SaveFlightSummary(ROCKET_FSM_t *fsm, SD *sd);


#endif /* INC_ROCKET_FSM_H_ */
