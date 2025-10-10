//#include "stm32h7xx.h"
//
//// --- DEFINICIONES DE MACROS FALTANTES ---
//// Estos valores se derivan del Reference Manual RM0399 para el FDCAN.
//
//// Direcciones y tamaños
//#define FDCAN_MESSAGE_RAM_BASE           (0x4000AC00U) // Dirección de la RAM de mensajes FDCAN
//
//// Posiciones de Bits del Elemento de Filtro Estándar (Word 0 del filtro)
//// ID1 (5-15), SFT (27-28), SFEC (29-31)
//#define FDCAN_ELEMENT_STANDARD_SFID1_Pos (5U)
//#define FDCAN_ELEMENT_STANDARD_SFT_Pos   (27U) // Asumo que el código anterior buscaba el SFT (Tipo de Filtro)
//#define FDCAN_ELEMENT_STANDARD_SFEC_Pos  (29U)
//
//// Posición de Bits del Elemento de Transmisión (Word 1: T1)
//#define FDCAN_TX_ELEMENT_DLC_Pos         (20U) // Bits [23:20]
//
//// Máscaras de Petición/Pendiente de Transmisión para el Buffer Dedicado 0
//#define FDCAN_TXBRP_TRP0                 (0x1U)  // Bit 0: Transmit Request Pending for Buffer 0
//#define FDCAN_TXBAR_AR0                  (0x1U)  // Bit 0: Add Request for Buffer 0
//
//// --- FIN DE DEFINICIONES DE MACROS FALTANTES ---
//
//
///**
// * @brief Configura los pines GPIO para la comunicación CAN (TX y RX).
// *
// * Se asume FDCAN1 en PB9 (TX) y PB8 (RX), que corresponden a la AF9.
// */
//void FDCAN1_GPIO_Init(void) {
//    // 1. Habilitar el reloj para el puerto GPIOB
//    // RCC_AHB4ENR (Registro de habilitación de reloj del bus AHB4)
//    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
//
//    // 2. Configurar PB8 (RX) y PB9 (TX) en Modo de Función Alternativa (AF)
//    // MODER (Registro de modo): 10 (Función Alternativa)
//    GPIOB->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk);
//    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1); // 0b10
//
//    // 3. Configurar Tipo de Salida (OTYPER): 0 (Push-pull)
//    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
//
//    // 4. Configurar Velocidad (OSPEEDR): 11 (Muy alta velocidad)
//    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED9_Msk);
//
//    // 5. Configurar Función Alternativa (AFR): AF9 para FDCAN1
//    // AFR[1] (AFRH) controla los pines del 8 al 15.
//    // Limpiar y luego establecer el valor 9 (0b1001)
//    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk);
//    GPIOB->AFR[1] |= (9U << GPIO_AFRH_AFSEL8_Pos) | (9U << GPIO_AFRH_AFSEL9_Pos);
//}
//
///**
// * @brief Inicializa el periférico FDCAN1 para operación CAN Clásico.
// */
//void FDCAN1_Init(void) {
//    // 1. Habilitar el reloj para FDCAN1
//    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
//
//    // 2. Entrar en modo de Inicialización (INIT)
//    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
//
//    // Esperar hasta que el modo INIT sea reconocido
//    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}
//
//    // 3. Configuración Principal del Modo
//    // - Deshabilitar CAN-FD (FDOE = 0)
//    // - Deshabilitar Bit Rate Switching (BRSE = 0)
//    // - Habilitar modo CAN Estándar (CCE = 1)
//    FDCAN1->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);
//    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
//
//    // 4. Configuración de Bit Timing (Baud Rate)
//    // Usamos el registro NBTP (Nominal Bit Timing Register)
//    // (Ejemplo: 500 kbit/s con reloj de 50 MHz)
//    FDCAN1->NBTP = ( (4U << FDCAN_NBTP_NBRP_Pos)   |
//                     (0U << FDCAN_NBTP_NSJW_Pos)  |
//                     (12U << FDCAN_NBTP_NTSEG1_Pos) |
//                     (6U << FDCAN_NBTP_NTSEG2_Pos) );
//
//    // 5. Configuración Global de Filtro (GFC)
//    // ANFS=1: Rechazar IDs estándar no filtrados (Default: Ruta a FIFO0)
//    FDCAN1->GFC |= FDCAN_GFC_ANFS_0;
//
//    // 6. Configuración de Filtros Estándar (SIDFC)
//    // LSS = 1 (1 Filtro Estándar)
//    FDCAN1->SIDFC &= ~FDCAN_SIDFC_LSS_Msk;
//    FDCAN1->SIDFC |= (1U << FDCAN_SIDFC_LSS_Pos);
//
//    // Configurar el Filtro 0 (en la RAM de mensajes)
//    volatile uint32_t *pStdFilter = (volatile uint32_t *)FDCAN_MESSAGE_RAM_BASE;
//
//    // Formato de palabra de filtro: SFEC | SFT | SFID2 | SFID1
//    // SFEC=0x1 (Store in FIFO0), SFT=0x1 (Filter ID), SFID1=0x123
//    *pStdFilter = ( (0x1U << FDCAN_ELEMENT_STANDARD_SFT_Pos) | // TIPO: 0x1 (Range Filter) - CORREGIDO
//                    (0x1U << FDCAN_ELEMENT_STANDARD_SFEC_Pos) | // CONFIGURACIÓN: Store in FIFO0 - CORREGIDO
//                    (0x123U << FDCAN_ELEMENT_STANDARD_SFID1_Pos) ); // ID a filtrar - CORREGIDO
//
//    // 7. Configuración de Recepción FIFO 0 (RXF0C)
//    // F0S: Tamaño de FIFO0 = 4 mensajes, F0OM: Overwrite Mode
//    FDCAN1->RXF0C &= ~FDCAN_RXF0C_F0S_Msk;
//    FDCAN1->RXF0C |= (4U << FDCAN_RXF0C_F0S_Pos);
//    FDCAN1->RXF0C |= FDCAN_RXF0C_F0OM;
//
//    // 8. Configuración de Transmisión (TXBC)
//    // NDTB: Número de Dedicated Transmit Buffers = 1 (usamos el buffer dedicado 0)
//    FDCAN1->TXBC &= ~FDCAN_TXBC_NDTB_Msk;
//    FDCAN1->TXBC |= (1U << FDCAN_TXBC_NDTB_Pos);
//
//    // 9. Salir del modo de Inicialización (INIT)
//    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
//    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
//
//    // Esperar hasta que el modo operativo (CCA) esté listo
//    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {}
//}
//
///**
// * @brief Función para enviar un mensaje CAN simple (solo registros).
// */
//void FDCAN1_Send_Message(uint32_t id, uint8_t *data, uint8_t len) {
//    // 1. Verificar si el Transmit Buffer Dedicado 0 está libre (TRP0 = 0)
//    if (FDCAN1->TXBRP & FDCAN_TXBRP_TRP0) { // CORREGIDO: Usando la macro definida
//        // El buffer 0 está pendiente de transmisión, abortar.
//        return;
//    }
//
//    // 2. Calcular la dirección base del Elemento de Transmisión (Buffer Dedicado 0)
//    uint32_t tx_offset = FDCAN1->TXBC & FDCAN_TXBC_TBSA_Msk;
//    volatile uint32_t *tx_element = (volatile uint32_t *)(FDCAN_MESSAGE_RAM_BASE + tx_offset);
//
//    // 3. Configurar el Elemento de Transmisión (TX Buffer Dedicado 0)
//
//    // Word 0 (T0): ID y Control (XTD=0, RTR=0)
//    uint32_t id_word = (id << 18); // ID estándar de 11 bits (bits 28:18)
//    *tx_element = id_word;
//
//    // Word 1 (T1): DLC y Control (BRS=0, FDCAN=0)
//    uint32_t dlc_word = (len << FDCAN_TX_ELEMENT_DLC_Pos); // CORREGIDO: Usando la macro definida
//    *(tx_element + 1) = dlc_word;
//
//    // Word 2 - 3: Datos
//    *(tx_element + 2) = 0; // Limpiar Word 2
//    if (len > 0) *(tx_element + 2) |= (uint32_t)data[0];
//    if (len > 1) *(tx_element + 2) |= (uint32_t)data[1] << 8;
//    if (len > 2) *(tx_element + 2) |= (uint32_t)data[2] << 16;
//    if (len > 3) *(tx_element + 2) |= (uint32_t)data[3] << 24;
//
//    *(tx_element + 3) = 0; // Limpiar Word 3
//    if (len > 4) *(tx_element + 3) |= (uint32_t)data[4];
//    if (len > 5) *(tx_element + 3) |= (uint32_t)data[5] << 8;
//    if (len > 6) *(tx_element + 3) |= (uint32_t)data[6] << 16;
//    if (len > 7) *(tx_element + 3) |= (uint32_t)data[7] << 24;
//
//
//    // 4. Iniciar la transmisión (TXBAR) para el buffer dedicado 0
//    FDCAN1->TXBAR = FDCAN_TXBAR_AR0; // CORREGIDO: Usando la macro definida
//}
