/*
 * ROCKET_FSM.c
 *
 *  Created on: Oct 12, 2025
 *      Author: de4lerr
 */

#include "ROCKET_FSM.h"
#include "ROCKET.h"

/* ==========================================================================================================
 * FSM
 * ========================================================================================================== */

/* ========================================================================
 * INICIALIZAÇÃO
 * ======================================================================== */

void ROCKET_FSM_Init(ROCKET_FSM_t *fsm)
{
	memset(fsm, 0, sizeof(ROCKET_FSM_t));

	fsm->current_state = FLIGHT_IDLE;
	fsm->previous_state = FLIGHT_IDLE;
	fsm->state_entry_time = HAL_GetTick();

	/* Parâmetros padrão otimizados para seu foguete */
	fsm->params.liftoff_acc_threshold = 2.0f;      // 2g de aceleração
	fsm->params.liftoff_acc_samples = 5;            // 25ms @ 200Hz

	fsm->params.burnout_acc_threshold = 0.5f;      // Queda para <0.5g
	fsm->params.burnout_time_max = 2.0f;           // Margem sobre 1.54s

	fsm->params.apogee_vel_threshold = 2.0f;       // Velocidade vertical
	fsm->params.apogee_vel_samples = 10;           // 50ms @ 200Hz
	fsm->params.apogee_alt_drop = 3.0f;            // 3m de queda

	fsm->params.landing_vel_threshold = 1.0f;
	fsm->params.landing_acc_threshold = 0.4f;
	fsm->params.landing_time = 3000;               // 3s estável

	fsm->params.max_altitude = 600.0f;             // Margem sobre 500m
	fsm->params.timeout_apogee = 5.0f;             // Safety timeout

	printf("[FSM] Inicializado em IDLE\r\n");
}

void ROCKET_FSM_ConfigureForProfile(ROCKET_FSM_t *fsm,
									float thrust_time,
									float expected_apogee,
									float max_gforce)
{
	fsm->params.burnout_time_max = thrust_time * 1.3f;  // 30% margem
	fsm->params.max_altitude = expected_apogee * 1.2f;   // 20% margem
	fsm->params.liftoff_acc_threshold = max_gforce * 0.3f; // 30% do pico

	printf("[FSM] Perfil: Queima=%.2fs, Apogeu=%.0fm, MaxG=%.1fg\r\n",
		   thrust_time, expected_apogee, max_gforce);
}

/* ========================================================================
 * TRANSIÇÕES DE ESTADO
 * ======================================================================== */

static void FSM_TransitionTo(ROCKET_FSM_t *fsm, FlightState_t new_state)
{
	if (fsm->current_state == new_state) return;

	fsm->previous_state = fsm->current_state;
	fsm->current_state = new_state;
	fsm->state_entry_time = HAL_GetTick();

	printf("[FSM] %s -> %s\r\n",
		   ROCKET_FSM_GetStateName(fsm->previous_state),
		   ROCKET_FSM_GetStateName(new_state));

	/* Atualiza timestamps */
	switch(new_state) {
		case FLIGHT_ARMED:
			fsm->stats.time_armed = HAL_GetTick();
			break;
		case FLIGHT_BOOST:
			fsm->stats.time_liftoff = HAL_GetTick();
			break;
		case FLIGHT_COAST:
			fsm->stats.time_burnout = HAL_GetTick();
			fsm->motor_burnout_detected = 1;
			break;
		case FLIGHT_APOGEE:
			fsm->stats.time_apogee = HAL_GetTick();
			fsm->apogee_detected = 1;
			break;
		case FLIGHT_DROGUE_DESCENT:
			fsm->stats.time_drogue = HAL_GetTick();
			fsm->parachute_deployed = 1;
			break;
		case FLIGHT_MAIN_DESCENT:
			fsm->stats.time_main = HAL_GetTick();
			break;
		case FLIGHT_LANDED:
			fsm->stats.time_landed = HAL_GetTick();
			break;
		default:
			break;
	}
}

/* ========================================================================
 * DETECÇÃO DE EVENTOS
 * ======================================================================== */

uint8_t ROCKET_FSM_DetectLiftoff(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket)
{
	if (!rocket->imu->is_valid) return 0;

	/* Aceleração total (magnitude) */
	float acc_mag = sqrtf(rocket->imu->acc_x * rocket->imu->acc_x +
						  rocket->imu->acc_y * rocket->imu->acc_y +
						  rocket->imu->acc_z * rocket->imu->acc_z);

	/* Detecta aceleração sustentada acima do threshold */
	if (acc_mag > fsm->params.liftoff_acc_threshold) {
		fsm->liftoff_counter++;

		if (fsm->liftoff_counter >= fsm->params.liftoff_acc_samples) {
			printf("[LIFTOFF] Detectado! Acc=%.2fg\r\n", acc_mag);
			fsm->liftoff_counter = 0;
			return 1;
		}
	} else {
		fsm->liftoff_counter = 0;
	}

	return 0;
}

uint8_t ROCKET_FSM_DetectBurnout(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket)
{
	if (!rocket->imu->is_valid) return 0;

	float flight_time = (HAL_GetTick() - fsm->stats.time_liftoff) / 1000.0f;

	/* Método 1: Tempo máximo de queima excedido */
	if (flight_time > fsm->params.burnout_time_max) {
		printf("[BURNOUT] Timeout em %.2fs\r\n", flight_time);
		return 1;
	}

	/* Método 2: Aceleração caiu drasticamente */
	float acc_mag = sqrtf(rocket->imu->acc_x * rocket->imu->acc_x +
						  rocket->imu->acc_y * rocket->imu->acc_y +
						  rocket->imu->acc_z * rocket->imu->acc_z);

	if (acc_mag < fsm->params.burnout_acc_threshold && flight_time > 0.5f) {
		printf("[BURNOUT] Acc caiu para %.2fg em %.2fs\r\n", acc_mag, flight_time);
		return 1;
	}

	return 0;
}

uint8_t ROCKET_FSM_DetectApogee(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket)
{
	float flight_time = (HAL_GetTick() - fsm->stats.time_liftoff) / 1000.0f;

	/* Timeout de segurança */
	if (flight_time > fsm->params.timeout_apogee) {
		printf("[APOGEE] TIMEOUT em %.2fs - FORÇANDO!\r\n", flight_time);
		return 1;
	}

	/* Atualiza histórico de altitude e velocidade */
	fsm->altitude_history[fsm->history_index] = rocket->state.pos[2];
	fsm->velocity_history[fsm->history_index] = rocket->state.vel[2];
	fsm->history_index = (fsm->history_index + 1) % 20;

	/* Método 1: Velocidade vertical próxima de zero (EKF) */
	if (rocket->use_ekf && rocket->status.ekf_initialized) {
		float vz = rocket->state.vel[2]; // NED: negativo = subindo

		if (vz > -fsm->params.apogee_vel_threshold) {
			fsm->apogee_counter++;

			if (fsm->apogee_counter >= fsm->params.apogee_vel_samples) {
				float altitude = -rocket->state.pos[2]; // Converte NED para altitude
				printf("[APOGEE] Vel=%.2f m/s, Alt=%.1fm\r\n", vz, altitude);
				fsm->stats.apogee_altitude = altitude;
				fsm->stats.apogee_time = flight_time;
				return 1;
			}
		} else {
			fsm->apogee_counter = 0;
		}
	}

	/* Método 2: Barômetro detecta queda de altitude */
	if (rocket->bar->is_valid && rocket->bar->has_initial_alt) {
		static float last_max_alt = 0;
		float current_alt = rocket->bar->altitude_m;

		if (current_alt > last_max_alt) {
			last_max_alt = current_alt;
		}

		float alt_drop = last_max_alt - current_alt;
		if (alt_drop > fsm->params.apogee_alt_drop && flight_time > 1.0f) {
			printf("[APOGEE] Barômetro: Queda de %.1fm\r\n", alt_drop);
			fsm->stats.apogee_altitude = last_max_alt;
			fsm->stats.apogee_time = flight_time;
			return 1;
		}
	}

	/* Método 3: Aceleração aponta para cima (foguete desacelerando) */
	if (rocket->imu->is_valid) {
		float acc_z_ned = rocket->imu->acc_ned_z;

		/* Se aceleração NED-Z é positiva (para baixo), está desacelerando */
		static uint8_t decel_count = 0;
		if (acc_z_ned > 2.0f && flight_time > 1.5f) {
			decel_count++;
			if (decel_count > 20) { // 100ms @ 200Hz
				printf("[APOGEE] Desaceleração detectada\r\n");
				return 1;
			}
		} else {
			decel_count = 0;
		}
	}

	return 0;
}

uint8_t ROCKET_FSM_DetectLanding(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket)
{
	if (!rocket->imu->is_valid) return 0;

	/* Velocidade baixa E aceleração ~1g */
	float vel_mag = sqrtf(rocket->state.vel[0] * rocket->state.vel[0] +
						  rocket->state.vel[1] * rocket->state.vel[1] +
						  rocket->state.vel[2] * rocket->state.vel[2]);

	float acc_mag = sqrtf(rocket->imu->acc_x * rocket->imu->acc_x +
						  rocket->imu->acc_y * rocket->imu->acc_y +
						  rocket->imu->acc_z * rocket->imu->acc_z);

	if (vel_mag < fsm->params.landing_vel_threshold &&
		fabsf(acc_mag - 1.0f) < fsm->params.landing_acc_threshold) {

		fsm->landing_counter++;

		if (fsm->landing_counter * 5 >= fsm->params.landing_time) { // @ 200Hz
			printf("[LANDED] Vel=%.2f m/s, Acc=%.2fg\r\n", vel_mag, acc_mag);
			fsm->stats.landing_velocity = vel_mag;
			return 1;
		}
	} else {
		fsm->landing_counter = 0;
	}

	return 0;
}

/* ========================================================================
 * UPDATE PRINCIPAL
 * ======================================================================== */

void ROCKET_FSM_Update(ROCKET_FSM_t *fsm, ROCKET_System_t *rocket)
{
	/* Atualiza estatísticas máximas */
	if (rocket->imu->is_valid) {
		float acc_mag = sqrtf(rocket->imu->acc_x * rocket->imu->acc_x +
							  rocket->imu->acc_y * rocket->imu->acc_y +
							  rocket->imu->acc_z * rocket->imu->acc_z);
		if (acc_mag > fsm->stats.max_gforce) {
			fsm->stats.max_gforce = acc_mag;
		}
	}

	if (rocket->bar->is_valid) {
		if (rocket->bar->altitude_m > fsm->stats.max_altitude) {
			fsm->stats.max_altitude = rocket->bar->altitude_m;
		}
	}

	/* Máquina de Estados */
	switch(fsm->current_state) {

		case FLIGHT_IDLE:
			/* Aguarda armamento manual ou automático */
			// Você pode implementar lógica de RBF aqui
			break;

		case FLIGHT_ARMED:
			/* Detecta liftoff */
			if (ROCKET_FSM_DetectLiftoff(fsm, rocket)) {
				FSM_TransitionTo(fsm, FLIGHT_BOOST);
			}
			break;

		case FLIGHT_BOOST:
			/* Detecta burnout */
			if (ROCKET_FSM_DetectBurnout(fsm, rocket)) {
				FSM_TransitionTo(fsm, FLIGHT_COAST);
			}
			break;

		case FLIGHT_COAST:
			/* Detecta apogeu */
			if (ROCKET_FSM_DetectApogee(fsm, rocket)) {
				FSM_TransitionTo(fsm, FLIGHT_APOGEE);
			}
			break;

		case FLIGHT_APOGEE:
			/* Aciona paraquedas e vai para descida */
			printf("[DEPLOY] Paraquedas acionado!\r\n");
			FSM_TransitionTo(fsm, FLIGHT_DROGUE_DESCENT);
			break;

		case FLIGHT_DROGUE_DESCENT:
			/* Detecta altura para paraquedas principal */
			if (rocket->bar->is_valid && rocket->bar->altitude_m < 150.0f) {
				printf("[DEPLOY] Paraquedas principal @ %.1fm\r\n",
					   rocket->bar->altitude_m);
				FSM_TransitionTo(fsm, FLIGHT_MAIN_DESCENT);
			}

			/* Ou detecta pouso direto */
			if (ROCKET_FSM_DetectLanding(fsm, rocket)) {
				FSM_TransitionTo(fsm, FLIGHT_LANDED);
			}
			break;

		case FLIGHT_MAIN_DESCENT:
			/* Detecta pouso */
			if (ROCKET_FSM_DetectLanding(fsm, rocket)) {
				FSM_TransitionTo(fsm, FLIGHT_LANDED);
			}
			break;

		case FLIGHT_LANDED:
			/* Estado final - continua telemetria */
			break;

		case FLIGHT_ERROR:
			/* Recovery ou safe mode */
			break;
	}
}

/* ========================================================================
 * FUNÇÕES AUXILIARES
 * ======================================================================== */

void ROCKET_FSM_SetState(ROCKET_FSM_t *fsm, FlightState_t new_state)
{
	printf("[FSM] FORÇADO: %s\r\n", ROCKET_FSM_GetStateName(new_state));
	FSM_TransitionTo(fsm, new_state);
}

const char* ROCKET_FSM_GetStateName(FlightState_t state)
{
	switch(state) {
		case FLIGHT_IDLE:           return "IDLE";
		case FLIGHT_ARMED:          return "ARMED";
		case FLIGHT_BOOST:          return "BOOST";
		case FLIGHT_COAST:          return "COAST";
		case FLIGHT_APOGEE:         return "APOGEE";
		case FLIGHT_DROGUE_DESCENT: return "DROGUE";
		case FLIGHT_MAIN_DESCENT:   return "MAIN";
		case FLIGHT_LANDED:         return "LANDED";
		case FLIGHT_ERROR:          return "ERROR";
		default:                    return "UNKNOWN";
	}
}

uint8_t ROCKET_FSM_IsFlying(ROCKET_FSM_t *fsm)
{
	return (fsm->current_state >= FLIGHT_BOOST &&
			fsm->current_state < FLIGHT_LANDED);
}

float ROCKET_FSM_GetStateTime(ROCKET_FSM_t *fsm)
{
	return (HAL_GetTick() - fsm->state_entry_time) / 1000.0f;
}

void ROCKET_FSM_PrintStatistics(ROCKET_FSM_t *fsm)
{
	printf("\r\n========== RESUMO DO VOO ==========\r\n");
	printf("Estado Atual: %s\r\n", ROCKET_FSM_GetStateName(fsm->current_state));

	if (fsm->stats.time_liftoff > 0) {
		printf("\n--- Eventos ---\r\n");
		printf("Liftoff:  T+%.3fs\r\n",
			   (fsm->stats.time_liftoff - fsm->stats.time_armed) / 1000.0f);

		if (fsm->motor_burnout_detected) {
			printf("Burnout:  T+%.3fs\r\n",
				   (fsm->stats.time_burnout - fsm->stats.time_liftoff) / 1000.0f);
		}

		if (fsm->apogee_detected) {
			printf("Apogeu:   T+%.3fs @ %.1fm\r\n",
				   fsm->stats.apogee_time, fsm->stats.apogee_altitude);
		}

		if (fsm->stats.time_landed > 0) {
			printf("Pouso:    T+%.3fs\r\n",
				   (fsm->stats.time_landed - fsm->stats.time_liftoff) / 1000.0f);
		}
	}

	printf("\n--- Máximos ---\r\n");
	printf("Altitude:     %.1f m\r\n", fsm->stats.max_altitude);
	printf("Aceleração:   %.1f g\r\n", fsm->stats.max_gforce);

	printf("===================================\r\n\r\n");
}

HAL_StatusTypeDef ROCKET_FSM_SaveFlightSummary(ROCKET_FSM_t *fsm, SD *sd)
{
	if (SD_OpenFile(sd, "flight_summary.txt", SD_MODE_APPEND) != HAL_OK) {
		return HAL_ERROR;
	}

	SD_WriteFormatted(sd, "\n========== VOO %lu ==========\n",
					 fsm->stats.time_armed);
	SD_WriteFormatted(sd, "Apogeu: %.1fm @ T+%.2fs\n",
					 fsm->stats.apogee_altitude, fsm->stats.apogee_time);
	SD_WriteFormatted(sd, "Max G-Force: %.2fg\n", fsm->stats.max_gforce);
	SD_WriteFormatted(sd, "Tempo Total: %.1fs\n",
					 (fsm->stats.time_landed - fsm->stats.time_liftoff) / 1000.0f);

	SD_CloseFile(sd);
	return HAL_OK;
}
