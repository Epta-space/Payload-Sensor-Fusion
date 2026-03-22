/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32G4 LoRa Transmitter - Corrigido DMA RX
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU.h"
#include "SD.h"
#include "GNSS.h"
#include "BAR.h"
#include "IMU.h"
#include "e22900t22d.h"
#include "LORA.h"

#include "ROCKET.h"
#include "ROCKET_FSM.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// Buffers para recepção com IDLE detection


// Contador de transmissões
int transmissionCounter = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

GNSS gnss;
SD sd;
LORA lora;
BAR bar;
IMU imu;
IMU_Fusion fusion;
ROCKET_System_t rocket;
ROCKET_FSM_t fsm;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int _write(int fd, char * ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

/* ========================================================================
 * SEQUÊNCIA PRÉ-VOO
 * ======================================================================== */

void TEST_SDWriteSpeed_Detailed(void) {
    printf("\r\n=== TESTE DETALHADO DE VELOCIDADE ===\r\n");

    if (SD_OpenFile(&sd, "speed.txt", SD_MODE_OVERWRITE) != HAL_OK) {
        printf("[ERRO] Falha ao abrir arquivo\r\n");
        return;
    }

    // Teste 1: Write sem sync
    uint32_t start = HAL_GetTick();
    for (int i = 0; i < 100; i++) {
        char line[256];
        snprintf(line, sizeof(line), "%d,test,data,line,with,many,fields\r\n", i);
        f_puts(line, &sd.file);  // Sem sync
    }
    uint32_t write_time = HAL_GetTick() - start;

    // Teste 2: Sync
    start = HAL_GetTick();
    f_sync(&sd.file);
    uint32_t sync_time = HAL_GetTick() - start;

    SD_CloseFile(&sd);

    printf("Resultados:\r\n");
    printf("- 100 writes: %lu ms (%.1f ms/linha)\r\n", write_time, write_time/100.0f);
    printf("- 1 sync: %lu ms\r\n", sync_time);
    printf("- Total: %lu ms\r\n", write_time + sync_time);

    if (write_time / 100.0f > 5.0f) {
        printf("\r\n[CRÍTICO] SD MUITO LENTO!\r\n");
        printf("Soluções:\r\n");
        printf("1. Reformate o SD (FAT32, cluster 4KB)\r\n");
        printf("2. Use SD Classe 10 ou superior\r\n");
        printf("3. Aumente FLIGHT_BUFFER_SIZE para 100+\r\n");
    }
}

void ROCKET_EmergencyMode()
{
    printf("\r\n[EMERGÊNCIA] Sistema em modo seguro\r\n");

    while(1) {
        // Pisca LED de erro
        HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

        // Tenta transmitir status de erro
        printf("ERROR: System failure");

        HAL_Delay(500);
    }
}


void ROCKET_PreFlightSequence(void)
{
    printf("\r\n");
    printf("╔════════════════════════════════════════════╗\r\n");
    printf("║    SISTEMA DE TELEMETRIA SRAD v2.0         ║\r\n");
    printf("║    Foguete: 500m Apogeu                    ║\r\n");
    printf("╚════════════════════════════════════════════╝\r\n");
    printf("\r\n");

    // 1. Verifica periféricos críticos
    printf("[1/6] Verificando periféricos...\r\n");
    if (!rocket.status.lora_initialized) {
        printf("[ERRO] LoRa não inicializado!\r\n");
        ROCKET_EmergencyMode();
    }
    if (!rocket.status.imu_initialized) {
        printf("[ERRO] IMU não inicializado!\r\n");
        ROCKET_EmergencyMode();
    }
    printf("[OK] Periféricos OK\r\n");

    // 2. Aguarda GPS fix
    printf("[2/6] Aguardando GPS fix...\r\n");
    uint32_t gps_timeout = HAL_GetTick() + 10000; // 1 minuto
    while (!rocket.gnss->is_fixed && HAL_GetTick() < gps_timeout) {
        ROCKET_Manager(&rocket);
        HAL_Delay(100);

        if ((HAL_GetTick() % 5000) == 0) {
            printf("    Satélites: %d, Status: %c\r\n",
                   rocket.gnss->satelites_usados, rocket.gnss->status);
        }
    }

    if (!rocket.gnss->is_fixed) {
        printf("[AVISO] GPS sem fix - continuando sem GPS\r\n");
    } else {
        printf("[OK] GPS Fix obtido!\r\n");
    }

    // 3. Calibração automática
    printf("[3/6] Calibrando sensores na rampa...\r\n");
    printf("    MANTENHA O FOGUETE IMÓVEL!\r\n");

    for (int i = 5; i > 0; i--) {
        printf("    %d...\r\n", i);
        HAL_Delay(1000);
    }

    ROCKET_CalibrateIMU(&rocket, 200);

    // Calibra barômetro
    float baro_sum = 0;
    for (int i = 0; i < 50; i++) {
        BAR_Update(&bar);
        baro_sum += bar.altitude_m;
        HAL_Delay(20);
    }
    bar.altitude_inicial = baro_sum / 50.0f;
    bar.has_initial_alt = 1;

    printf("[OK] Calibração concluída\r\n");
    printf("    Altitude Base: %.1f m\r\n", bar.altitude_inicial);

    // 4. Inicializa EKF com estado zero
    printf("[4/6] Inicializando EKF...\r\n");
    rocket.ekf.x[0] = 0;
    rocket.ekf.x[1] = 0;
    rocket.ekf.x[2] = 0;
    for(int i = 0; i < 3; i++) {
        rocket.ekf.P[i][i] = 100.0f;
        rocket.ekf.P[i+3][i+3] = 10.0f;
    }
    rocket.status.ekf_initialized = 1;
    printf("[OK] EKF pronto\r\n");

    // 5. Configura FSM para perfil de voo
    printf("[5/6] Configurando perfil de voo...\r\n");
    ROCKET_FSM_ConfigureForProfile(&fsm, 1.54f, 500.0f, 10.0f);
    printf("[OK] Perfil carregado\r\n");

    // 6. Cria arquivo de log
    printf("[6/6] Criando arquivo de log...\r\n");
    if (ROCKET_Logger_CreateFile(&rocket, "flight_log.csv") == HAL_OK) {
        printf("[OK] Arquivo de log criado\r\n");
    } else {
        printf("[AVISO] Falha ao criar arquivo - continuando sem gravação\r\n");
    }

    printf("\r\n");
    printf("╔════════════════════════════════════════════╗\r\n");
    printf("║           SISTEMA PRONTO PARA VOO          ║\r\n");
    printf("║                                            ║\r\n");
    printf("║   			BON VOYANGE :D    		   	 ║\r\n");
    printf("╚════════════════════════════════════════════╝\r\n");
    printf("\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Antes de MX_USART2_UART_Init()
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SPI2_CS_BAR_GPIO_Port, SPI2_CS_BAR_Pin, SET);

  // Inicializa subsistemas
  ROCKET_Init(&rocket, &gnss, &sd, &lora, &bar, &imu, &fusion, &fsm);
  ROCKET_FSM_Init(&fsm);

  if (ROCKET_BeginPeripherals(&rocket,
                              &huart2, &huart3, &hi2c2, &hspi2,
                              LORA_M0_GPIO_Port, LORA_M0_Pin,
                              LORA_M1_GPIO_Port, LORA_M1_Pin,
                              SPI2_CS_IMU_GPIO_Port, SPI2_CS_IMU_Pin) != HAL_OK) {
      Error_Handler();
  }

  // Configura LoRa
  LORA_SetAddress(&lora, LORA_ADDRESS_STM32G4, LORA_ADDRESS_ESP32);
  LORA_SetChannel(&lora, LORA_CHANNEL);

  // Habilita interrupção IDLE do GNSS
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

//  // Cria arquivos de log
//  ROCKET_CreateLogFile(&rocket, "telemetry.csv");
//  ROCKET_CreateLogFile(&rocket, "flight_buffer.csv");

//  // Coordenadas do local atual
//  double last_lat = -18.9186;
//  double last_lon = -48.2772;
//  float last_alt = 850.0;
//
//  // Data/hora atual (ajuste conforme RTC ou manualmente)
//  uint16_t last_year = 2025;
//  uint8_t last_month = 10;
//  uint8_t last_day = 22;
//  uint8_t last_hour = 11;  // UTC (Brasília é UTC-3)
//  uint8_t last_min = 25;
//  uint8_t last_sec = 0;
//
//  printf("\r\n[INFO] Injetando dados de auxilio no GPS...\r\n");
//  if (ROCKET_InjectGPSAidData(&rocket, last_lat, last_lon, last_alt,
//                              last_year, last_month, last_day,
//                              last_hour, last_min, last_sec) == HAL_OK) {
//      printf("[OK] GPS configurado para Hot Start\r\n");
//      printf("[INFO] Aguardando fix GPS (esperado: 1-5s)...\r\n");
//  } else {
//      printf("[AVISO] Falha ao injetar dados - GPS usara Cold Start\r\n");
//  }

  // SEQUÊNCIA PRÉ-VOO
  ROCKET_PreFlightSequence();

  ROCKET_FSM_SetState(&fsm, FLIGHT_ARMED);
  printf("[ARMED] Sistema armado - aguardando liftoff\r\n");

/*
   // Calibração do magnetômetro
   printf("\r\n=== CALIBRACAO DO MAGNETOMETRO ===\r\n");
   for(int c = 20; c !=0; c--){
	   ROCKET_Manager(&rocket);
   }

   if (ROCKET_CollectMagCalibrationData(&rocket, 120000) == HAL_OK) { // 2 minutos
       printf("[OK] Dados coletados!\r\n");
       printf("Remova o SD e execute MagCalibration.m no MATLAB\r\n");
       while(1); // Para para remover SD
   }

   //Após calibrado, altere os valores em ROCKET.c -> ROCKET_BeginPeripherals()
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print = 0;
  uint32_t last_stats = 0;

  //TEST_SDWriteSpeed_Detailed();


  while (1)
  {
	  ROCKET_SetTransmissionInterval(&rocket, 500);

      // MANAGER PRINCIPAL
      ROCKET_Manager(&rocket);

      // ATUALIZA MÁQUINA DE ESTADOS
      ROCKET_FSM_Update(&fsm, &rocket);

      // AÇÕES ESPECÍFICAS POR ESTADO
      switch(fsm.current_state) {

          case FLIGHT_ARMED:
              // Apenas aguarda detecção de liftoff
              break;

          case FLIGHT_BOOST:
          case FLIGHT_COAST:
               // VOO CRÍTICO - dados sendo gravados automaticamente no Manager
               // Flush forçado a cada 100ms (definido no Manager)

          case FLIGHT_APOGEE:
              // ACIONA PARAQUEDAS AQUI!
              printf("[DEPLOY] *** EJETANDO PARAQUEDAS ***\r\n");

              // Flush imediato antes de acionar paraquedas
              ROCKET_Logger_ForceFlush(&rocket);
              // TODO: Adicionar código do skib
              break;

          case FLIGHT_DROGUE_DESCENT:
        	  break;

          case FLIGHT_MAIN_DESCENT:
        	  // Monitora descida - dados continuam sendo gravados
              break;

          case FLIGHT_LANDED:
              // Flush final e fecha arquivo
              static uint8_t landing_processed = 0;
              if (!landing_processed) {
                  printf("[LANDED] Processando dados finais...\r\n");

                  ROCKET_Logger_ForceFlush(&rocket);
                  ROCKET_FSM_SaveFlightSummary(&fsm, &sd);

                  // Mantém arquivo aberto para continuar gravando telemetria com taxa reduzida

                  landing_processed = 1;
              }
              break;

          default:
              break;
      }

      // STATUS PERIÓDICO
      if ((HAL_GetTick() - last_print) >= 2000) {
          printf("\r\n[%s] T+%.2fs | Alt: %.1f/%.1f m | Vel: %.1f m/s | G: %.2fg\r\n",
                 ROCKET_FSM_GetStateName(fsm.current_state),
                 ROCKET_FSM_GetStateTime(&fsm),
                 rocket.bar->altitude_m,
                 fsm.stats.max_altitude,
                 -rocket.state.vel[2],
                 fsm.stats.max_gforce);

          	  	 //*************************printf(gnss.rx_buffer);

          // Estatísticas em voo
          if (ROCKET_FSM_IsFlying(&fsm)) {
              printf("    GPS: %s (%d sats) | IMU: %s | EKF: %s\r\n",
                     rocket.gnss->is_fixed ? "OK" : "NO",
                     rocket.gnss->satelites_usados,
                     rocket.imu->is_valid ? "OK" : "NO",
                     rocket.status.ekf_initialized ? "OK" : "NO");

              // Buffer status
              printf("    Buffer: %d/%d | Gravadas: %lu | Perdidas: %lu\r\n",
                     rocket.logger.buffer.count,
                     FLIGHT_BUFFER_SIZE,
                     rocket.logger.samples_written,
                     rocket.logger.buffer.lost_samples);
          }

          // Status da transmissão
          if(rocket.lora->tx_in_progress) {
              printf("    [TX] Transmissão em andamento...\r\n");
          }

          ROCKET_PrintState(&rocket);

          last_print = HAL_GetTick();
      }

      // Estatísticas detalhadas a cada 10 segundos
      if ((HAL_GetTick() - last_stats) >= 10000) {
          if (ROCKET_FSM_IsFlying(&fsm)) {
              ROCKET_Logger_PrintStats(&rocket);
          }
          last_stats = HAL_GetTick();
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0090194B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI2_CS_BAR_Pin|LORA_M1_Pin|IMU_INT1_Pin|IMU_FSYNC_Pin
                          |SPI3_CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_IMU_Pin|LORA_M0_Pin|LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRST_Pin */
  GPIO_InitStruct.Pin = NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_BAR_Pin LORA_M1_Pin IMU_INT1_Pin IMU_FSYNC_Pin
                           SPI3_CS_SD_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_BAR_Pin|LORA_M1_Pin|IMU_INT1_Pin|IMU_FSYNC_Pin
                          |SPI3_CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_IMU_Pin LORA_M0_Pin LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_IMU_Pin|LORA_M0_Pin|LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_Pin */
  GPIO_InitStruct.Pin = BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) {
    	//printf("[DEBUG] TX Complete callback!\r\n");

        // ✅ Notifica a biblioteca E22 que está pronta
        e22_lora_make_ready();

        // ✅ Notifica o LORA que TX completou
        if(rocket.lora) {
            LORA_MakeReady(rocket.lora);
        }

        //HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
