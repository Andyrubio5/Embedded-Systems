/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (USART3 version)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_AMBIENTE      25.0f
#define T_HEATER        80.0f
#define ALPHA           0.1f
#define BETA            0.02f
#define HISTERESIS      0.5f
#define UART_BUFFER_LEN 32

#define RUNTIME_STATS_TIMER_FREQ 10000  // 10 kHz para mejor resolución

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/* demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
// #define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
float temperatura_actual = T_AMBIENTE;
float temperatura_consigna = 30.0f;
bool calefactor_on = false;
bool auto_status_enabled = false;

osThreadId_t SimuladorTaskHandle;
osThreadId_t ControlTaskHandle;
osThreadId_t UARTTaskHandle;
osThreadId_t StatusTaskHandle;
osMutexId_t TempMutexHandle;
//osMessageQueueId_t UARTQueueHandle;

static char uart_rx_buffer[UART_BUFFER_LEN];
static uint8_t uart_rx_index = 0;
static char uart_cmd_buffer[UART_BUFFER_LEN];  // Buffer compartido
static volatile bool uart_cmd_ready = false;   // Flag
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void StartSimuladorTask(void *argument);
void StartControlTask(void *argument);
void StartUARTTask(void *argument);
void StartStatusTask(void *argument);
void StartProfilingTask(void *argument);
void procesar_comando(char *cmd);
void enviar_profiling_report(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Tarea simulador de temperatura
 */
void StartSimuladorTask(void *argument)
{
    uint32_t tick_count = osKernelGetTickCount();

    for(;;)
    {
        osMutexAcquire(TempMutexHandle, osWaitForever);

        float delta = 0.0f;

        if (calefactor_on)
            delta += ALPHA * (T_HEATER - temperatura_actual);

        delta -= BETA * (temperatura_actual - T_AMBIENTE);

        temperatura_actual += delta;

        osMutexRelease(TempMutexHandle);

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

        tick_count += 500;
        osDelayUntil(tick_count);
    }
}

/**
 * @brief  Tarea de control ON/OFF
 */
void StartControlTask(void *argument)
{
    uint32_t tick_count = osKernelGetTickCount();

    for(;;)
    {
        osMutexAcquire(TempMutexHandle, osWaitForever);

        if (temperatura_actual < (temperatura_consigna - HISTERESIS))
        {
            calefactor_on = true;
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
#ifdef HAL_TIM_MODULE_ENABLED
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 800);
#endif
        }
        else if (temperatura_actual > (temperatura_consigna + HISTERESIS))
        {
            calefactor_on = false;
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
#ifdef HAL_TIM_MODULE_ENABLED
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
#endif
        }

        osMutexRelease(TempMutexHandle);

        tick_count += 1000;
        osDelayUntil(tick_count);
    }
}

/**
 * @brief  Tarea UART
 */
void StartUARTTask(void *argument)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)"[UART Task OK]\r\n", 16, 100);

    for(;;)
    {
        // Esperar notificación
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (uart_cmd_ready)
        {
            uart_cmd_ready = false;
            procesar_comando(uart_cmd_buffer);
        }
    }
}
/**
 * @brief  Tarea Status
 */
void StartStatusTask(void *argument)
{
    char msg[100];

    for(;;)
    {
        osDelay(5000);

        if (!auto_status_enabled)
        {
            continue;
        }

        osMutexAcquire(TempMutexHandle, osWaitForever);

        int temp_ent = (int)temperatura_actual;
        int temp_dec = (int)((temperatura_actual - temp_ent) * 100);
        int cons_ent = (int)temperatura_consigna;
        int cons_dec = (int)((temperatura_consigna - cons_ent) * 100);

        snprintf(msg, sizeof(msg), "Temp: %d.%02d C | Heater: %s | Setpoint: %d.%02d C\r\n",
                 temp_ent, temp_dec,
                 calefactor_on ? "ON" : "OFF",
                 cons_ent, cons_dec);

        osMutexRelease(TempMutexHandle);

        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

/**
 * @brief  Tarea de Profiling
 * @note   Muestra estadísticas de uso de CPU y memoria cada 10 segundos
 */
//void StartProfilingTask(void *argument)
//{
//    for(;;)
//    {
//        osDelay(10000);
//        enviar_profiling_report();
//    }
//}

/**
 * @brief  Genera y envía reporte de profiling
 */
void enviar_profiling_report(void)
{
    char msg[100];
    char buffer[512];  // ✅ Para runtime stats

    snprintf(msg, sizeof(msg), "\r\n========== PROFILING REPORT ==========\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    size_t free_heap = xPortGetFreeHeapSize();
    size_t min_free_heap = xPortGetMinimumEverFreeHeapSize();
    size_t total_heap = configTOTAL_HEAP_SIZE;
    size_t used_heap = total_heap - free_heap;

    snprintf(msg, sizeof(msg), "HEAP: Used=%lu/%lu bytes (%.1f%%), Min Free=%lu bytes\r\n",
                    (unsigned long)used_heap, (unsigned long)total_heap,
                    (float)used_heap * 100.0f / total_heap,
                    (unsigned long)min_free_heap);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    snprintf(msg, sizeof(msg), "\r\nSTACK HIGH WATERMARK (bytes free):\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    if (SimuladorTaskHandle != NULL) {
        UBaseType_t watermark = uxTaskGetStackHighWaterMark(SimuladorTaskHandle);
        snprintf(msg, sizeof(msg), "  Simulador:  %lu bytes\r\n", watermark * 4);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    if (ControlTaskHandle != NULL) {
        UBaseType_t watermark = uxTaskGetStackHighWaterMark(ControlTaskHandle);
        snprintf(msg, sizeof(msg), "  Control:    %lu bytes\r\n", watermark * 4);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    if (UARTTaskHandle != NULL) {
        UBaseType_t watermark = uxTaskGetStackHighWaterMark(UARTTaskHandle);
        snprintf(msg, sizeof(msg), "  UART:       %lu bytes\r\n", watermark * 4);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    if (StatusTaskHandle != NULL) {
        UBaseType_t watermark = uxTaskGetStackHighWaterMark(StatusTaskHandle);
        snprintf(msg, sizeof(msg), "  Status:     %lu bytes\r\n", watermark * 4);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    snprintf(msg, sizeof(msg), "======================================\r\n\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief  Procesador de comandos
 */
/**
 * @brief  Procesador de comandos (VERSIÓN CORREGIDA)
 */
/**
 * @brief  Procesador de comandos (VERSIÓN CORREGIDA)
 */
void procesar_comando(char *cmd)
{
    char msg[100];

    // Debug: mostrar comando recibido SIN LIMPIAR AÚN
    snprintf(msg, sizeof(msg), "CMD RAW: [%s]\r\n", cmd);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Limpieza: remover espacios, \r, \n al final
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\r' || cmd[len-1] == '\n'))
    {
        cmd[len-1] = '\0';
        len--;
    }

    // Debug: mostrar comando limpio
    snprintf(msg, sizeof(msg), "CMD CLEAN: [%s]\r\n", cmd);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // ========== STATUS ==========
    if ((strcmp(cmd, "STATUS") == 0) || (strcmp(cmd, "status") == 0))
    {
        osMutexAcquire(TempMutexHandle, osWaitForever);

        int temp_ent = (int)temperatura_actual;
        int temp_dec = (int)((temperatura_actual - temp_ent) * 100);
        int cons_ent = (int)temperatura_consigna;
        int cons_dec = (int)((temperatura_consigna - cons_ent) * 100);

        snprintf(msg, sizeof(msg), "Temp: %d.%02d C | Heater: %s | Setpoint: %d.%02d C\r\n",
                 temp_ent, temp_dec,
                 calefactor_on ? "ON" : "OFF",
                 cons_ent, cons_dec);

        osMutexRelease(TempMutexHandle);

        // ✅ ENVIAR EL MENSAJE
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // ========== PROF ==========
    else if ((strcmp(cmd, "PROF") == 0) || (strcmp(cmd, "prof") == 0))
    {
        enviar_profiling_report();
    }
    // ========== AUTO ON ==========
    else if ((strncmp(cmd, "AUTO ON", 7) == 0) || (strncmp(cmd, "auto on", 7) == 0))
    {
        auto_status_enabled = true;
        snprintf(msg, sizeof(msg), "Auto status enabled (every 5s)\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // ========== AUTO OFF ==========
    else if ((strncmp(cmd, "AUTO OFF", 8) == 0) || (strncmp(cmd, "auto off", 8) == 0))
    {
        auto_status_enabled = false;
        snprintf(msg, sizeof(msg), "Auto status disabled\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // ========== SET ==========
    else if ((strncmp(cmd, "SET ", 4) == 0) || (strncmp(cmd, "set ", 4) == 0))
    {
        float nueva = atof(cmd + 4);
        temperatura_consigna = nueva;

        int cons_ent = (int)temperatura_consigna;
        int cons_dec = (int)((temperatura_consigna - cons_ent) * 100);

        snprintf(msg, sizeof(msg), "Nueva consigna: %d.%02d C\r\n", cons_ent, cons_dec);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // ========== ON ==========
    else if ((strcmp(cmd, "ON") == 0) || (strcmp(cmd, "on") == 0))
    {
        calefactor_on = true;
        snprintf(msg, sizeof(msg), "Heater turned on.\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // ========== OFF ==========
    else if ((strcmp(cmd, "OFF") == 0) || (strcmp(cmd, "off") == 0))
    {
        calefactor_on = false;
        snprintf(msg, sizeof(msg), "Heater turned off.\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    // ========== COMANDO DESCONOCIDO ==========
    else
    {
        snprintf(msg, sizeof(msg), "Comandos: STATUS, SET xx, ON, OFF, PROF, AUTO ON, AUTO OFF\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNdame el paos a paso de este codigo que debe de hacer C_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* ============================================ */
  /* 1. PRIMERO: Inicializar el kernel */
  /* ============================================ */
  osKernelInitialize();
  MX_FREERTOS_Init();

  MX_USART3_UART_Init();
  HAL_UART_Transmit(&huart3, (uint8_t*)"*** INICIO ***\r\n", 16, 1000);

  /* Crear mutex */
  const osMutexAttr_t TempMutex_attributes = { .name = "TempMutex" };
  TempMutexHandle = osMutexNew(&TempMutex_attributes);

  /* NO CREAR COLA - ELIMINAR ESTA SECCIÓN COMPLETAMENTE */

  /* Crear tareas */
  const osThreadAttr_t SimuladorTask_attributes = {
      .name = "Simulador",
      .stack_size = 256 * 4,
      .priority = (osPriority_t) osPriorityNormal,
  };
  SimuladorTaskHandle = osThreadNew(StartSimuladorTask, NULL, &SimuladorTask_attributes);

  const osThreadAttr_t ControlTask_attributes = {
      .name = "Control",
      .stack_size = 256 * 4,
      .priority = (osPriority_t) osPriorityNormal,
  };
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  const osThreadAttr_t UARTTask_attributes = {
      .name = "UART",
      .stack_size = 512 * 4,
      .priority = (osPriority_t) osPriorityAboveNormal,
  };
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  const osThreadAttr_t StatusTask_attributes = {
      .name = "Status",
      .stack_size = 512 * 4,
      .priority = (osPriority_t) osPriorityBelowNormal,
  };
  StatusTaskHandle = osThreadNew(StartStatusTask, NULL, &StatusTask_attributes);

  /* Habilitar UART */
  HAL_UART_Transmit(&huart3, (uint8_t*)"Listo\r\n> ", 9, 1000);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);

  /* LEDs y botones */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  osKernelStart();

  /* Infinite loop */
  while (1)
  {
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Callback de recepción UART completa
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // El caracter recibido está en la posición que se acaba de llenar
        char c = uart_rx_buffer[uart_rx_index];

        // 1. Manejo de Backspace/Delete
        if (c == 8 || c == 127) // ASCII 8 (Backspace) o 127 (Delete)
        {
            if (uart_rx_index > 0)
            {
                uart_rx_index--; // Retrocede el índice

                // Envía el código de borrado a la terminal (Backspace, Espacio, Backspace)
                HAL_UART_Transmit(&huart3, (uint8_t*)"\b \b", 3, 10);
            }
            // Reinicia la recepción en el nuevo índice
            HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
            return; // Termina la función para no procesar como comando
        }

        // 2. Echo (Mostrar el caracter)
        HAL_UART_Transmit(&huart3, (uint8_t*)&c, 1, 10);

        // 3. Manejo de Fin de Línea (Enter)
        if (c == '\r' || c == '\n')
        {
            if (uart_rx_index > 0)
            {
                // Sobrescribe el \r o \n con el terminador nulo para el procesamiento
                uart_rx_buffer[uart_rx_index] = '\0';

                // Copiar y notificar
                strcpy(uart_cmd_buffer, uart_rx_buffer);
                uart_cmd_ready = true;

                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(UARTTaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

                HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n> ", 4, 10);
            }

            // En cualquier caso (con o sin comando), reinicia para el siguiente
            uart_rx_index = 0;
        }
        // 4. Manejo de Caracteres Normales
        else if (uart_rx_index < UART_BUFFER_LEN - 1)
        {
            uart_rx_index++; // Avanza el índice para la próxima recepción
        }
        // 5. Desbordamiento de Buffer
        else
        {
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n[Buffer Full]\r\n> ", 19, 10);
            uart_rx_index = 0;
        }

        // 6. Volver a pedir la recepción del siguiente byte en la nueva posición
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
