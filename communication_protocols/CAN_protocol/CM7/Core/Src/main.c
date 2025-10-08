
#include <stdint.h>                              // <- Tipos enteros de tamaño fijo (uint8_t, uint16_t, etc.).

// ----------------------------------------------------------------------------
// 1. DEFINICIONES DE REGISTROS (STM32H745xx)
// ----------------------------------------------------------------------------
// Direcciones de Registros (Verificadas para H7)
#define RCC_BASE          (0x58024400UL)         // <- Dirección base del bloque RCC (clock).
#define GPIOA_BASE        (0x58020000UL)         // <- Dirección base del bloque GPIOA.

// Punteros a Registros
#define RCC_AHB4ENR       (*(volatile uint32_t *)(RCC_BASE + 0xE0))   // <- Registro que habilita clocks AHB4 (incluye GPIOA).
#define GPIOA_MODER       (*(volatile uint32_t *)(GPIOA_BASE + 0x00)) // <- Modo de pines (entrada/salida/AF/analog).
#define GPIOA_OSPEEDR     (*(volatile uint32_t *)(GPIOA_BASE + 0x08)) // <- Velocidad de salida de pines.
#define GPIOA_BSRR        (*(volatile uint32_t *)(GPIOA_BASE + 0x18)) // <- Registro de set/reset atómico de pines.

// Máscaras y Bits
#define GPIOAEN   (1 << 0)                       // <- Bit para habilitar el clock de GPIOA en RCC_AHB4ENR.
#define CAN_TX_PIN_N    8                        // <- Usaremos el pin PA8.
#define PA8_SET  (1 << CAN_TX_PIN_N)             // <- Máscara para poner PA8 a 1 (set) vía BSRR.
#define PA8_RESET (1 << (CAN_TX_PIN_N + 16))     // <- Máscara para poner PA8 a 0 (reset) vía BSRR (bits altos del BSRR).

// CAN Timing: 125 kbit/s (1 bit = 8 µs)
// **NOTA:** Ajusta BIT_TIME_CYCLES si la frecuencia de tu H7 no es la esperada.
#define BIT_TIME_CYCLES  3000                    // <- Cuántos ciclos de CPU dura 1 bit (retardo por “delay_cycles”).
#define FRAME_DELAY_LOOPS  10000000              // <- Espera larga entre tramas para que las veas separadas en el scope.

// Estados del bus CAN
#define DOMINANT   0                             // <- Nivel “dominante” (en bus real: 0 lógico).
#define RECESSIVE  1                             // <- Nivel “recesivo” (en bus real: 1 lógico).

// Estructura de la Trama CAN
typedef struct {
   uint16_t ID;                                  // <- Identificador de 11 bits (usamos uint16 para guardarlo).
   uint8_t DLC;                                  // <- Data Length Code: número de bytes de datos (0..8).
   uint8_t Data[8];                              // <- Hasta 8 bytes de datos.
} CAN_Frame_t;                                   // <- Nombre del tipo.

// ----------------------------------------------------------------------------
// 2. FUNCIONES BARE-METAL (Cortex-M7)
// ----------------------------------------------------------------------------

/**
 * @brief  Delay preciso en ciclos de CPU (ARM Assembly)
 */
__attribute__((always_inline)) static inline void delay_cycles(uint32_t cycles) {
   __asm volatile (                     // <- Bloque ensamblador en línea.
       "1: subs %0, #1\n"               // <- Resta 1 a 'cycles'; actualiza banderas.
       "   bne 1b\n"                    // <- Si no es cero, salta a la etiqueta 1 (loop).
       : "=r" (cycles)                  // <- Salida: cycles es clobbered (se consume).
       : "0" (cycles)                   // <- Entrada: cycles inicial.
   );
}                                       // <- Fin del retardo de N ciclos (aproximado).

/**
 * @brief  Envía un bit individual al bus CAN
 */
void CAN_SendBit(uint8_t state) {
   if (state == RECESSIVE) {            // <- Si el bit deseado es recesivo (1 lógico)...
       GPIOA_BSRR = PA8_SET;            // <- Escribe en BSRR parte baja: pone PA8 en 1 (alto).
   } else {                             // <- Si el bit deseado es dominante (0 lógico)...
       GPIOA_BSRR = PA8_RESET;          // <- Escribe en BSRR parte alta: pone PA8 en 0 (bajo).
   }
   delay_cycles(BIT_TIME_CYCLES);       // <- Mantén ese nivel por el “tiempo de 1 bit”.
}

void CAN_SendBit_Stuffed(uint8_t state, int *stuff_count) {
    // 1. Verificar Bit Stuffing antes de enviar el bit
    if (state == DOMINANT) {
        *stuff_count = 0; // Se rompe la secuencia de '1's recesivos.
    } else { // state == RECESSIVE
        // Si el bit es RECESSIVE (1), incrementamos el contador
        (*stuff_count)++;
    }

    // 2. Enviar el bit real
    if (state == RECESSIVE) {
        GPIOA_BSRR = PA8_SET;
    } else { // DOMINANT
        GPIOA_BSRR = PA8_RESET;
    }
    delay_cycles(BIT_TIME_CYCLES);

    // 3. Insertar el bit de relleno si es necesario
    if (*stuff_count == 5) {
        // Enviar el bit de relleno (de polaridad opuesta)
        CAN_SendBit(DOMINANT);      // Insertar un '0' dominante (stuff bit)
        *stuff_count = 0;           // Reiniciar el contador de secuencia
    }
}


/**
 * @brief  Cálculo de CRC simplificado (para la visualización)
 */
uint16_t CAN_CalculateCRC(CAN_Frame_t *frame) {
   uint16_t crc = 0;                    // <- Acumulador del “CRC” (aquí no es el real de CAN).
   // Suma simple de los datos para generar un valor determinístico
   for(int i = 0; i < frame->DLC; i++) {// <- Recorre sólo los bytes válidos (DLC).
       crc += frame->Data[i];           // <- Suma cada byte al acumulador.
   }
   // Tomar 15 bits
   return crc & 0x7FFF;                 // <- Devuelve sólo 15 bits (formato parecido al CRC de CAN).
}

/**
 * @brief  Envía una trama CAN completa (simplificada sin Bit Stuffing)
 */
void CAN_SendFrame(CAN_Frame_t *frame) {
    int i, j;
    uint16_t crc;
    // Contador para el Bit Stuffing: lleva la cuenta de bits iguales consecutivos.
    int stuff_count = 0;

    // ** Bit Stuffing se aplica desde SOF hasta el final del CRC Field. **

    // 1. INTERFRAME SPACE (3 bits recesivos)
    // NOTA: El Bit Stuffing NO se aplica aquí.
    for(i = 0; i < 3; i++) CAN_SendBit(RECESSIVE);

    // 2. START OF FRAME (SOF)
    CAN_SendBit(DOMINANT);               // SOF: un 0 (dominante).

    // 3. ARBITRATION FIELD (ID - 11 bits)
    for(i = 10; i >= 0; i--)
        CAN_SendBit_Stuffed(((frame->ID >> i) & 0x01), &stuff_count);

    // 4. CONTROL FIELD: RTR (1) | IDE (0) | r0 (0) | DLC (4 bits)
    // El Bit Stuffing también se aplica aquí.
    CAN_SendBit_Stuffed(RECESSIVE, &stuff_count);   // RTR=1
    CAN_SendBit_Stuffed(DOMINANT, &stuff_count);    // IDE=0
    CAN_SendBit_Stuffed(DOMINANT, &stuff_count);    // r0=0
    for(i = 3; i >= 0; i--)
        CAN_SendBit_Stuffed(((frame->DLC >> i) & 0x01), &stuff_count);

    // 5. DATA FIELD
    for(i = 0; i < frame->DLC; i++) {
        for(j = 7; j >= 0; j--)
            CAN_SendBit_Stuffed(((frame->Data[i] >> j) & 0x01), &stuff_count);
    }

    // 6. CRC FIELD (15 bits)
    crc = CAN_CalculateCRC(frame);
    for(i = 14; i >= 0; i--)
        CAN_SendBit_Stuffed(((crc >> i) & 0x01), &stuff_count);

    // FIN del Bit Stuffing.

    // El resto de la trama NO lleva Bit Stuffing:

    // 7. CRC Delimiter
    CAN_SendBit(RECESSIVE);              // CRC Delimiter = 1.

    // 8. ACK FIELD
    CAN_SendBit(RECESSIVE);              // ACK Slot (se mantiene en 1 al no haber receptor).
    CAN_SendBit(RECESSIVE);              // ACK Delimiter = 1.

    // 9. END OF FRAME (EOF) - 7 bits recesivos
    for(i = 0; i < 7; i++)
        CAN_SendBit(RECESSIVE);
}
/**
 * @brief  Inicialización de Clock y GPIO para PA8 (H7 Bare-Metal).
 */
static void init_bare_metal(void) {
    // 1. Habilitar el Clock para GPIOA
    RCC_AHB4ENR |= GPIOAEN;             // <- Enciende el reloj del puerto A (sin esto, GPIOA no responde).

    // 2. Configurar PA8 como Salida Push-Pull de alta velocidad
    // Limpiar MODER (bits 16 y 17) para PA8
    GPIOA_MODER &= ~(0x3 << (CAN_TX_PIN_N * 2)); // <- Borra los 2 bits de modo del pin 8.
    // Escribir 0b01 (Output Mode)
    GPIOA_MODER |= (0x1 << (CAN_TX_PIN_N * 2));  // <- Pone modo “salida” en PA8.

    // Configurar OSPEEDR (bits 16 y 17) a "Very High Speed" (0b11)
    GPIOA_OSPEEDR &= ~(0x3 << (CAN_TX_PIN_N * 2)); // <- Limpia velocidad previa de PA8.
    GPIOA_OSPEEDR |= (0x3 << (CAN_TX_PIN_N * 2));  // <- Fija “muy alta velocidad” para bordes limpios.

    // 3. Poner el pin inicial en RECESSIVE (HIGH)
    GPIOA_BSRR = PA8_SET;               // <- Arranca con la línea en 1 (bus en reposo = recesivo).
}

// ----------------------------------------------------------------------------
// 3. BUCLE PRINCIPAL
// ----------------------------------------------------------------------------

int main(void) {
    init_bare_metal();                  // <- Configura reloj de GPIOA y PA8 como salida rápida en alto.

    // Configurar trama CAN de ejemplo
    CAN_Frame_t tx_frame = {            // <- Crea una trama de ejemplo:
        .ID = 0x123, .DLC = 8,          // <- ID estándar 0x123, 8 bytes de datos.
        .Data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88} // <- El payload.
    };

    while (1) {                         // <- Bucle infinito.
        // Enviar la trama
        CAN_SendFrame(&tx_frame);       // <- Envía la trama completa bit a bit por PA8.

        // Esperar antes de enviar la siguiente
        delay_cycles(FRAME_DELAY_LOOPS);// <- Pausa larga para que se distingan en el osciloscopio.
    }
}                                       // <- Fin del programa.

