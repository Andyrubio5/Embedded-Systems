#include <stdint.h>  // Para usar tipos de dato de tamaño fijo (uint8_t, uint16_t, etc.)

// ============================================================================
// CONFIGURACIÓN DE DIRECCIONES DE MEMORIA DEL MICROCONTROLADOR STM32H745
// ============================================================================

// --- Direcciones base de los periféricos ---
#define RCC_BASE          (0x58024400UL)  // RCC: controla los relojes del micro
#define GPIOA_BASE        (0x58020000UL)  // GPIOA: controla los pines del puerto A

// --- Registros específicos que vamos a usar ---
#define RCC_AHB4ENR       (*(volatile uint32_t *)(RCC_BASE + 0xE0))   // Enciende/apaga el reloj de periféricos
#define GPIOA_MODER       (*(volatile uint32_t *)(GPIOA_BASE + 0x00)) // Define si un pin es entrada, salida, etc.
#define GPIOA_OSPEEDR     (*(volatile uint32_t *)(GPIOA_BASE + 0x08)) // Controla qué tan rápido cambia un pin
#define GPIOA_BSRR        (*(volatile uint32_t *)(GPIOA_BASE + 0x18)) // Para poner un pin en alto (1) o bajo (0)

// --- Configuración del pin que usaremos ---
#define GPIOAEN           (1 << 0)              // Bit que enciende el reloj del puerto A
#define CAN_TX_PIN_N      8                     // Vamos a usar el pin PA8
#define PA8_SET           (1 << CAN_TX_PIN_N)   // Valor para poner PA8 en alto (1)
#define PA8_RESET         (1 << (CAN_TX_PIN_N + 16))  // Valor para poner PA8 en bajo (0)

// --- Configuración de tiempos para el bus CAN ---
#define BIT_TIME_CYCLES   3000        // Cuántos ciclos de reloj dura cada bit (para 125 kbit/s)
#define FRAME_DELAY_LOOPS 10000000    // Tiempo de espera entre tramas completas

// --- Estados del bus CAN ---
#define DOMINANT   0  // Bit dominante (gana en arbitraje, representa 0 lógico)
#define RECESSIVE  1  // Bit recesivo (pierde en arbitraje, representa 1 lógico)

// --- Estructura para guardar una trama CAN ---
typedef struct {
   uint16_t ID;       // Identificador de la trama (11 bits)
   uint8_t DLC;       // Cantidad de bytes de datos (0 a 8)
   uint8_t Data[8];   // Los datos que se envían (máximo 8 bytes)
} CAN_Frame_t;

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

/**
 * Hace una pausa precisa contando ciclos del procesador
 * @param cycles: cuántos ciclos de CPU esperar
 */
__attribute__((always_inline)) static inline void delay_cycles(uint32_t cycles) {
   __asm volatile (
       "1: subs %0, #1\n"  // Resta 1 al contador
       "   bne 1b\n"       // Si no llegó a cero, repite
       : "=r" (cycles)
       : "0" (cycles)
   );
}

/**
 * Envía un solo bit por el pin PA8
 * @param state: DOMINANT (0) o RECESSIVE (1)
 */
void CAN_SendBit(uint8_t state) {
   if (state == RECESSIVE) {
       GPIOA_BSRR = PA8_SET;    // Pone el pin en alto (1)
   } else {
       GPIOA_BSRR = PA8_RESET;  // Pone el pin en bajo (0)
   }
   delay_cycles(BIT_TIME_CYCLES);  // Mantiene el nivel durante el tiempo de un bit
}

/**
 * Envía un bit aplicando la regla de "bit stuffing"
 * Bit stuffing: si hay 5 bits iguales seguidos, se inserta uno contrario
 * @param state: el bit a enviar (DOMINANT o RECESSIVE)
 * @param stuff_count: contador de bits iguales consecutivos
 */
void CAN_SendBit_Stuffed(uint8_t state, int *stuff_count) {
    // 1. Actualizar el contador de bits consecutivos
    if (state == DOMINANT) {
        *stuff_count = 0;  // Si enviamos un 0, reiniciamos el contador
    } else {
        (*stuff_count)++;  // Si enviamos un 1, aumentamos el contador
    }

    // 2. Enviar el bit solicitado
    if (state == RECESSIVE) {
        GPIOA_BSRR = PA8_SET;
    } else {
        GPIOA_BSRR = PA8_RESET;
    }
    delay_cycles(BIT_TIME_CYCLES);

    // 3. Si llevamos 5 bits iguales, insertar un bit de relleno
    if (*stuff_count == 5) {
        CAN_SendBit(DOMINANT);  // Insertamos un 0 para romper la secuencia
        *stuff_count = 0;       // Reiniciamos el contador
    }
}

/**
 * Calcula un CRC simplificado (no es el CRC real de CAN, solo para demostración)
 * @param frame: puntero a la trama CAN
 * @return: un valor de 15 bits que simula el CRC
 */
uint16_t CAN_CalculateCRC(CAN_Frame_t *frame) {
   uint16_t crc = 0;
   // Suma todos los bytes de datos
   for(int i = 0; i < frame->DLC; i++) {
       crc += frame->Data[i];
   }
   return crc & 0x7FFF;  // Solo usa 15 bits (como el CRC real de CAN)
}

/**
 * Envía una trama CAN completa siguiendo el formato estándar
 * @param frame: puntero a la trama que se va a enviar
 */
void CAN_SendFrame(CAN_Frame_t *frame) {
    int i, j;
    uint16_t crc;
    int stuff_count = 0;  // Contador para el bit stuffing

    // -----------------------------------------------
    // 1. INTERFRAME SPACE (espacio entre tramas)
    // -----------------------------------------------
    // 3 bits en alto (recesivos) para separar tramas
    for(i = 0; i < 3; i++) CAN_SendBit(RECESSIVE);

    // -----------------------------------------------
    // 2. START OF FRAME (inicio de trama)
    // -----------------------------------------------
    // Un bit dominante (0) marca el inicio
    CAN_SendBit(DOMINANT);

    // -----------------------------------------------
    // 3. ARBITRATION FIELD (identificador)
    // -----------------------------------------------
    // Envía los 11 bits del ID, del más significativo al menos significativo
    for(i = 10; i >= 0; i--)
        CAN_SendBit_Stuffed(((frame->ID >> i) & 0x01), &stuff_count);

    // -----------------------------------------------
    // 4. CONTROL FIELD (información de control)
    // -----------------------------------------------
    CAN_SendBit_Stuffed(RECESSIVE, &stuff_count);  // RTR=1 (trama de datos)
    CAN_SendBit_Stuffed(DOMINANT, &stuff_count);   // IDE=0 (ID estándar de 11 bits)
    CAN_SendBit_Stuffed(DOMINANT, &stuff_count);   // r0=0 (bit reservado)

    // Envía el DLC (cantidad de bytes de datos)
    for(i = 3; i >= 0; i--)
        CAN_SendBit_Stuffed(((frame->DLC >> i) & 0x01), &stuff_count);

    // -----------------------------------------------
    // 5. DATA FIELD (los datos)
    // -----------------------------------------------
    // Envía cada byte de datos, bit por bit
    for(i = 0; i < frame->DLC; i++) {
        for(j = 7; j >= 0; j--)
            CAN_SendBit_Stuffed(((frame->Data[i] >> j) & 0x01), &stuff_count);
    }

    // -----------------------------------------------
    // 6. CRC FIELD (verificación de errores)
    // -----------------------------------------------
    // Calcula y envía el CRC de 15 bits
    crc = CAN_CalculateCRC(frame);
    for(i = 14; i >= 0; i--)
        CAN_SendBit_Stuffed(((crc >> i) & 0x01), &stuff_count);

    // NOTA: A partir de aquí ya NO se aplica bit stuffing

    // -----------------------------------------------
    // 7. CRC DELIMITER (separador del CRC)
    // -----------------------------------------------
    CAN_SendBit(RECESSIVE);  // Siempre es 1

    // -----------------------------------------------
    // 8. ACK FIELD (confirmación)
    // -----------------------------------------------
    CAN_SendBit(RECESSIVE);  // ACK Slot (el receptor lo pondría en 0)
    CAN_SendBit(RECESSIVE);  // ACK Delimiter (siempre 1)

    // -----------------------------------------------
    // 9. END OF FRAME (fin de trama)
    // -----------------------------------------------
    // 7 bits recesivos (en alto) marcan el final
    for(i = 0; i < 7; i++)
        CAN_SendBit(RECESSIVE);
}

/**
 * Inicializa el microcontrolador para usar el pin PA8 como salida
 */
static void init_bare_metal(void) {
    // 1. Encender el reloj del puerto A (sin esto el puerto no funciona)
    RCC_AHB4ENR |= GPIOAEN;

    // 2. Configurar PA8 como salida digital
    GPIOA_MODER &= ~(0x3 << (CAN_TX_PIN_N * 2));  // Limpia la configuración anterior
    GPIOA_MODER |= (0x1 << (CAN_TX_PIN_N * 2));   // Lo pone en modo salida

    // 3. Configurar PA8 para velocidad muy alta (bordes limpios y rápidos)
    GPIOA_OSPEEDR &= ~(0x3 << (CAN_TX_PIN_N * 2));
    GPIOA_OSPEEDR |= (0x3 << (CAN_TX_PIN_N * 2));

    // 4. Poner el pin en alto (estado de reposo del bus CAN)
    GPIOA_BSRR = PA8_SET;
}

// ============================================================================
// PROGRAMA PRINCIPAL
// ============================================================================

int main(void) {
    // Configurar el microcontrolador
    init_bare_metal();

    // Crear una trama de ejemplo para enviar
    CAN_Frame_t tx_frame = {
        .ID = 0x123,    // Identificador 0x123 (en decimal: 291)
        .DLC = 8,       // Vamos a enviar 8 bytes
        .Data = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}  // Los datos
    };

    // Bucle infinito: enviar tramas continuamente
    while (1) {
        CAN_SendFrame(&tx_frame);         // Envía la trama completa
        delay_cycles(FRAME_DELAY_LOOPS);  // Espera antes de enviar la siguiente
    }
}
