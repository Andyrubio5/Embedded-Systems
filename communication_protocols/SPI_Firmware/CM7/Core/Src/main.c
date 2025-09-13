// SPI Firmware

// Header que da acceso a los registros
#include "stm32h7xx.h"

// Definicion del puerto CS
/* Es el encargado de la seleccion a donde voy a mandar el dato, aqui definimos el puerto y el pin */

#define CS_PORT     GPIOD  // Se utiliza el puerto D
#define CS_PIN     (1u << 14)  // Se da de alta el pin 14 (segun el data sheet el pin PD14, es el utilizado para SPI_A_CS)

// Helpers, usaremos helpers para CS, para tener una forma mas sencilla de la seleccion (como un boton de atajo)

static inline void CS_L(void){CS_PORT->BSRR = (uint32_t)CS_PIN << 16; }
/* Habilita el "boton" en ON, el CS se "apaga" y ya no "escucha" nada de lo enviado */
static inline void CS_H(void){CS_PORT->BSRR = CS_PIN; }
/* Deshabilita el "boton" en OFF, el CS se "enciende" y empieza a "escuchar" lo enviado */

