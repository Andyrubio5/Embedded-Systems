#include "stm32h7xx.h"   // Cabecera CMSIS/HAL específica de la familia STM32H7 (define registros, SystemCoreClock, etc.)

/* ===== Lista de pines usados =====
   SPI_PORT: GPIOA
   SPI_SCK : PA5  -> Clock
   SPI_MOSI: PA8  -> Master Out Slave In
   SPI_MISO: PA6  -> Master In Slave Out
   SPI_CS  : PA4  -> Chip Select
*/

/* ===== Configuración de pines SPI ===== */
#define SPI_PORT       GPIOA                // Estructura de registros del puerto GPIOA
#define SPI_PORT_ENR   RCC_AHB4ENR_GPIOAEN  // Bit para habilitar reloj de GPIOA en el bus AHB4
#define SPI_SCK        5u                   // Número de pin para SCK: PA5
#define SPI_MOSI       8u                   // Número de pin para MOSI: PA8
#define SPI_MISO       6u                   // Número de pin para MISO: PA6
#define SPI_CS         4u                   // Número de pin para CS:  PA4

/* ===== Helpers GPIO: escritura rápida usando BSRR =====
   BSRR permite SET/RESET atómico de pines:
   - Escribir 1 en bit n      -> pone pin n en nivel alto (SET)
   - Escribir 1 en bit n+16   -> pone pin n en nivel bajo  (RESET)
   Ventaja: no hace read-modify-write y evita condiciones de carrera. */
static inline void sck_high(void) { SPI_PORT->BSRR = (1u << SPI_SCK); }          // SCK = 1
static inline void sck_low(void)  { SPI_PORT->BSRR = (1u << (SPI_SCK + 16u)); }  // SCK = 0
static inline void mosi_high(void){ SPI_PORT->BSRR = (1u << SPI_MOSI); }         // MOSI = 1
static inline void mosi_low(void) { SPI_PORT->BSRR = (1u << (SPI_MOSI + 16u)); } // MOSI = 0
static inline void cs_high(void)  { SPI_PORT->BSRR = (1u << SPI_CS); }           // CS = 1 (inactivo)
static inline void cs_low(void)   { SPI_PORT->BSRR = (1u << (SPI_CS + 16u)); }   // CS = 0 (activo)

/* ===== Inicialización DWT para delays precisos =====
   El DWT (Data Watchpoint and Trace) del Cortex-M7 incluye un contador de ciclos (CYCCNT).
   Lo habilitamos para poder medir tiempo y construir delays por “busy-wait” con precisión de ciclos. */
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Habilita el bloque de trazado (necesario para usar DWT)
    DWT->LAR = 0xC5ACCE55;                           // Desbloquea el acceso a registros protegidos del DWT (requerido en H7)
    DWT->CYCCNT = 0;                                 // Reinicia el contador de ciclos
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Activa el conteo de ciclos (CYCCNT empieza a incrementar)
}

/* ===== Delay en milisegundos usando ciclos de CPU =====
   Calcula cuántos ciclos corresponden al tiempo pedido en ms según SystemCoreClock.
   Luego espera activamente (busy-wait) hasta que el delta de ciclos se cumpla. */
static void delay_ms(uint32_t ms) {
    uint32_t start = DWT->CYCCNT;                                    // Muestra de inicio
    uint64_t cycles = (uint64_t)SystemCoreClock * ms / 1000u;        // Ciclos necesarios = fCPU * (ms/1000)
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { /* espera */ } // Bucle vacío hasta cumplir tiempo
}

/* ===== Inicialización de GPIO para SPI (bit-bang) =====
   Configura SCK, MOSI y CS como salidas push-pull, muy alta velocidad, sin pull-ups/downs.
   Deja MISO como entrada (por defecto). */
static void GPIO_Init_SPI(void) {
    // 1) Habilita el reloj del puerto GPIOA en RCC (sin reloj no se pueden escribir registros del puerto)
    RCC->AHB4ENR |= SPI_PORT_ENR;

    // 2) Configura los pines de salida (SCK, MOSI, CS)
    uint32_t pins[] = {SPI_SCK, SPI_MOSI, SPI_CS};
    for (int i = 0; i < 3; i++) {
        // MODER: 00=input, 01=output, 10=AF, 11=analog. Aquí ponemos 01 (salida)
        SPI_PORT->MODER   &= ~(3u << (pins[i]*2));  // Limpia los dos bits del pin
        SPI_PORT->MODER   |=  (1u << (pins[i]*2));  // 01 -> salida

        // OTYPER: 0=push-pull, 1=open-drain. Usamos push-pull
        SPI_PORT->OTYPER  &= ~(1u << pins[i]);

        // OSPEEDR: 00=bajo, 01=medio, 10=alto, 11=muy alto. Usamos muy alto para bordes más rápidos
        SPI_PORT->OSPEEDR |=  (3u << (pins[i]*2));

        // PUPDR: 00=sin pull, 01=pull-up, 10=pull-down, 11=reservado. Dejamos sin pull
        SPI_PORT->PUPDR   &= ~(3u << (pins[i]*2));
    }

    // NOTA: MISO (PA6) se deja en entrada (MODER=00 por defecto). Si tu esclavo lo requiere,
    // podrías configurar PUPDR con pull-up/down apropiado para evitar flotantes.

    // 3) Estados iniciales de las líneas
    sck_low();   // CPOL=0 -> reloj en reposo en 0
    mosi_high(); // Estas dos líneas fuerzan breve 1->0 en MOSI; no es necesario, pero no afecta
    mosi_low();  // MOSI=0 en reposo
    cs_high();   // CS inactivo (alto) -> ningún esclavo seleccionado
}

/* ===== SPI por bit-banging: master, 8 bits, MSB primero =====
   Protocolo equivalente a SPI Mode 0 (CPOL=0, CPHA=0):
   - Reloj en bajo en reposo.
   - El maestro cambia MOSI y luego hace flanco de subida en SCK; el esclavo muestrea en ese flanco.
   - Durante el flanco de subida, el master también lee MISO. */
static uint8_t SPI_Transfer(uint8_t data) {
    uint8_t received = 0; // Acumula los bits recibidos desde MISO

    cs_low();             // Activa el esclavo (CS = 0). Inicio de la transacción

    // Recorre 8 bits desde el más significativo (bit 7) hacia el menos (bit 0)
    for (int i = 7; i >= 0; i--) {
        // 1) Prepara el bit de salida en MOSI
        if (data & (1u << i)) {
            mosi_high();  // Bit = 1
        } else {
            mosi_low();   // Bit = 0
        }

        // 2) Flanco de subida del reloj: el esclavo lee MOSI y presenta su bit en MISO
        sck_high();
        delay_ms(50);     // Espera para dar tiempo al esclavo (MUY lento a propósito / demo)

        // 3) Lee MISO en el flanco de subida (típico en Mode 0)
        received <<= 1;                              // Desplaza lo recibido (hace lugar para el nuevo bit)
        if (SPI_PORT->IDR & (1u << SPI_MISO)) {     // Lee el registro de entrada del puerto y testea el bit MISO
            received |= 1u;                         // Si MISO=1, añade un '1' al LSB
        }

        // 4) Flanco de bajada del reloj: fin de la ventana de muestreo
        sck_low();
        delay_ms(50);     // Espera antes del siguiente bit (simetría)
    }

    cs_high();            // Desactiva el esclavo (CS = 1). Fin de la transacción
    return received;      // Devuelve el byte que llegó por MISO durante la transferencia
}

/* ===== Función principal =====
   - Actualiza SystemCoreClock (frecuencia real del CPU).
   - Inicializa el DWT para delays precisos.
   - Inicializa GPIOs para SPI por software.
   - En un bucle infinito, envía el carácter 'H' y espera 1 segundo. */
int main(void) {
    SystemCoreClockUpdate(); // Actualiza variable global SystemCoreClock (p.ej. 400 MHz en H7 si así está configurado)
    DWT_Init();              // Habilita contador de ciclos para temporización
    GPIO_Init_SPI();         // Configura pines SCK/MOSI/CS como salidas y deja MISO en entrada

    while (1) {
        (void)SPI_Transfer('H'); // Envía 'H' (0x48 = 0100 1000) y, de paso, lee un byte desde MISO (se ignora con (void))
        delay_ms(1000);          // Espera 1 segundo entre envíos (solo para demo)
    }
}
