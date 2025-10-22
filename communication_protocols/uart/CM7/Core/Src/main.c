/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Comunicación UART - Envía una palabra y recibe respuesta
  *
  * HARDWARE:
  * - USART3 configurado en pines PB10 (TX) y PB11 (RX)
  *
  * CONFIGURACIÓN:
  * - Velocidad de comunicación: 9600 baudios
  * - 8 bits de datos, sin paridad, 1 bit de parada
  *
  * FUNCIONAMIENTO:
  * - Envía la palabra "coca" cada segundo
  * - Espera a recibir un byte de respuesta
  * - Se queda esperando la respuesta indefinidamente
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Variables privadas --------------------------------------------------------*/
UART_HandleTypeDef huart3;  // Manejador para USART3

/* Prototipos de funciones privadas ------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART3_UART_Init(void);
void Error_Handler(void);

/**
  * @brief  Punto de entrada principal de la aplicación
  * @retval int
  */
int main(void)
{
  /* Inicialización del microcontrolador ------------------------------------*/
  HAL_Init();                    // Inicializa la librería HAL de STM32
  SystemClock_Config();          // Configura el reloj del sistema a 400 MHz
  MX_USART3_UART_Init();         // Inicializa USART3 a 9600 baudios

  /* Variables locales */
  uint8_t byteRecibido;          // Variable para almacenar el byte recibido
  HAL_StatusTypeDef estadoRX;    // Variable para verificar estado de recepción

  HAL_Delay(500);  // Espera 500 ms para que el sistema se estabilice

  /* Mensaje a enviar */
  char mensaje[] = "coca";       // Palabra que se enviará continuamente

  /* Bucle principal infinito -----------------------------------------------*/
  while (1)
  {
    /* Envía la palabra "coca" por UART */
    HAL_UART_Transmit(&huart3, (uint8_t*)mensaje, strlen(mensaje), HAL_MAX_DELAY);

    /* Espera 1 segundo antes de enviar nuevamente */
    HAL_Delay(1000);

    /* Intenta recibir 1 byte desde el UART */
    estadoRX = HAL_UART_Receive(&huart3, &byteRecibido, 1, 100);

    /* Si se recibió un byte correctamente */
    if (estadoRX == HAL_OK)
    {
      /* El byte está en la variable "byteRecibido" */
      /* Aquí podrías procesar el byte recibido */
    }
  }
}

/* ===========================================================================
 * FUNCIONES DE CONFIGURACIÓN DEL HARDWARE
 * =========================================================================== */

/**
  * @brief  Configura el reloj del sistema
  * @details Configura HSI a 64 MHz con PLL para obtener 400 MHz en SYSCLK
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef configOscilador = {0};
  RCC_ClkInitTypeDef configReloj = {0};

  /* Configura el suministro de voltaje regulado interno (LDO) */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Configura el oscilador HSI (generador de reloj interno) y el PLL */
  configOscilador.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  configOscilador.HSIState = RCC_HSI_DIV1;
  configOscilador.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  configOscilador.PLL.PLLState = RCC_PLL_ON;              // Activa el multiplicador PLL
  configOscilador.PLL.PLLSource = RCC_PLLSOURCE_HSI;      // Fuente: reloj interno
  configOscilador.PLL.PLLM = 4;   // Divisor de entrada (64 MHz / 4 = 16 MHz)
  configOscilador.PLL.PLLN = 50;  // Multiplicador (16 MHz * 50 = 800 MHz)
  configOscilador.PLL.PLLP = 2;   // Divisor para SYSCLK (800 MHz / 2 = 400 MHz)
  configOscilador.PLL.PLLQ = 2;   // Divisor para otros periféricos
  configOscilador.PLL.PLLR = 2;   // Divisor alternativo
  configOscilador.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  configOscilador.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  configOscilador.PLL.PLLFRACN = 0;

  if (HAL_RCC_OscConfig(&configOscilador) != HAL_OK)
  {
    Error_Handler();  // Si falla la configuración del oscilador
  }

  /* Configura los divisores de los buses del sistema */
  configReloj.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                        | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  configReloj.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;     // Usa el PLL como fuente
  configReloj.SYSCLKDivider = RCC_SYSCLK_DIV1;            // Sin división adicional
  configReloj.AHBCLKDivider = RCC_HCLK_DIV2;              // Divide el reloj AHB entre 2
  configReloj.APB3CLKDivider = RCC_APB3_DIV2;             // Divide APB3 entre 2
  configReloj.APB1CLKDivider = RCC_APB1_DIV2;             // Divide APB1 entre 2
  configReloj.APB2CLKDivider = RCC_APB2_DIV2;             // Divide APB2 entre 2
  configReloj.APB4CLKDivider = RCC_APB4_DIV2;             // Divide APB4 entre 2

  if (HAL_RCC_ClockConfig(&configReloj, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();  // Si falla la configuración del reloj
  }
}

/**
  * @brief  Inicializa USART3 para comunicación serial
  * @details Configura USART3 en pines PB10 (TX) y PB11 (RX) a 9600 baudios
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;                         // Velocidad: 9600 baudios (bits/segundo)
  huart3.Init.WordLength = UART_WORDLENGTH_8B;          // Cada mensaje tiene 8 bits de datos
  huart3.Init.StopBits = UART_STOPBITS_1;               // Un bit de parada entre mensajes
  huart3.Init.Parity = UART_PARITY_NONE;                // Sin bit de paridad (sin verificación)
  huart3.Init.Mode = UART_MODE_TX_RX;                   // Modo transmisión Y recepción
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;          // Sin control de flujo por hardware
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;      // Muestrea 16 veces por bit (más preciso)
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // Usa muestreo de 3 bits estándar
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;     // Sin preescalador (divisor = 1)
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // Sin características avanzadas

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();  // Si falla la inicialización, llama al manejador de errores
  }
}

/**
  * @brief  Manejador de errores
  * @details Deshabilita todas las interrupciones y entra en bucle infinito
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();  // Deshabilita todas las interrupciones del procesador
  while (1)         // Bucle infinito para detener la ejecución (indica error crítico)
  {
    /* El programa se queda aquí indefinidamente si ocurre un error */
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reporta el nombre del archivo y línea donde ocurrió un error de assert
  * @param  file: puntero al nombre del archivo
  * @param  line: número de línea donde falló el assert
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* El usuario puede añadir su implementación para reportar el error */
}
#endif /* USE_FULL_ASSERT */
