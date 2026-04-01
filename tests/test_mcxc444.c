/*
 * =============================================================================
 * PetPal v2 - MCXC444 Test
 * =============================================================================
 * Tests: HC-SR04 ultrasonic (interrupt), Servo (PWM), Onboard LED, UART2
 *
 * Behavior:
 *   - LED starts ON (active-low: write 0 to turn on)
 *   - Ultrasonic measures distance every 200ms via interrupt
 *   - Distance is sent to ESP32 as a UART packet
 *   - ESP32 determines if pet is near, sends back 0x01 (pet near) or 0x00
 *   - If pet near: LED turns OFF + servo sweeps
 *   - If no pet: LED stays ON, servo centered
 *
 * Pin assignments:
 *   HC-SR04 Trig  : PTA1  (GPIO output)
 *   HC-SR04 Echo  : PTA2  (GPIO input, interrupt both edges)
 *   Servo         : PTC2  (TPM0_CH1, PWM 50Hz)
 *   Onboard LED   : PTB18 (GPIO output, active-low green LED)
 *   UART2 TX      : PTE22 (UART2_TX, Alt4) -> ESP32-S2 RX pin (GPIO44)
 *   UART2 RX      : PTE23 (UART2_RX, Alt4) -> ESP32-S2 TX pin (GPIO43)
 *
 * Protocol:
 *   MCX -> ESP:  [0xAA] [0x01] [dist_hi] [dist_lo]
 *   ESP -> MCX:  [0xBB] [0x01] [status]  (0x01=pet near, 0x00=no pet)
 *
 * NOTE: Uses fsl_uart.h (UART2), NOT fsl_lpuart.h.
 *       Make sure fsl_uart driver is included in your SDK components.
 * =============================================================================
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_tpm.h"
#include "fsl_uart.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"

/* ========================= PIN DEFINITIONS =============================== */

/* Ultrasonic */
#define TRIG_PORT       PORTA
#define TRIG_GPIO       GPIOA
#define TRIG_PIN        1U

#define ECHO_PORT       PORTA
#define ECHO_GPIO       GPIOA
#define ECHO_PIN        2U

/* Servo */
#define SERVO_TPM       TPM0
#define SERVO_CHANNEL   1U
#define SERVO_PORT      PORTC
#define SERVO_PIN       2U

/* Onboard Green LED (FRDM-MCXC444: PTB18, active-low) */
#define LED_PORT        PORTB
#define LED_GPIO        GPIOB
#define LED_PIN         18U
#define LED_ON()        GPIO_PinWrite(LED_GPIO, LED_PIN, 0U)
#define LED_OFF()       GPIO_PinWrite(LED_GPIO, LED_PIN, 1U)

/* UART2 to ESP32 on PTE22/PTE23 */
#define ESP_UART        UART2
#define ESP_UART_BAUD   115200U
#define ESP_TX_PORT     PORTE
#define ESP_TX_PIN      22U     /* UART2_TX, Alt4 */
#define ESP_RX_PORT     PORTE
#define ESP_RX_PIN      23U     /* UART2_RX, Alt4 */

/* Servo positions */
#define SERVO_CENTER    1500
#define SERVO_LEFT      800
#define SERVO_RIGHT     2200

/* Timer clock: 8MHz / 128 = 62500Hz */
#define TIMER_CLK       62500
#define SERVO_MOD       ((TIMER_CLK / 50) - 1)  /* 1249 for 50Hz */

/* ========================= ULTRASONIC STATE ============================== */

static volatile uint32_t echoStartUs = 0;
static volatile uint16_t lastDistance = 999;
static volatile uint8_t  echoReady   = 0;

/* Microsecond timing using SysTick */
static volatile uint32_t msCount = 0;

void SysTick_Handler(void)
{
    msCount++;
}

static uint32_t get_micros(void)
{
    uint32_t ms = msCount;
    uint32_t val = SysTick->VAL;
    uint32_t load = SysTick->LOAD + 1;
    uint32_t cps = SystemCoreClock / 1000000;
    if (msCount != ms) { ms = msCount; val = SysTick->VAL; }
    return (ms * 1000) + ((load - val) / cps);
}

/* ========================= UART RX BUFFER ================================ */

#define RX_BUF_SIZE     32
static volatile uint8_t  rxBuf[RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;
static volatile uint8_t  petDetected = 0;

/* ========================= ISR: Ultrasonic Echo (PORTA) ================== */

void PORTA_IRQHandler(void)
{
    uint32_t flags = GPIO_PortGetInterruptFlags(ECHO_GPIO);

    if (flags & (1U << ECHO_PIN)) {
        if (GPIO_PinRead(ECHO_GPIO, ECHO_PIN)) {
            echoStartUs = get_micros();
        } else {
            uint32_t pw = get_micros() - echoStartUs;
            if (pw > 100 && pw < 30000) {
                lastDistance = (uint16_t)(pw / 58);
            }
            echoReady = 1;
        }
        GPIO_PortClearInterruptFlags(ECHO_GPIO, 1U << ECHO_PIN);
    }
}

/* ========================= ISR: UART2 RX ================================= */
/*
 * On MCXC444, UART2 shares its IRQ vector with FLEXIO.
 * The handler name must be UART2_FLEXIO_IRQHandler.
 */
void UART2_FLEXIO_IRQHandler(void)
{
    /* Check RX data register full */
    if (UART_GetStatusFlags(ESP_UART) & kUART_RxDataRegFullFlag) {
        uint8_t data = UART_ReadByte(ESP_UART);
        uint16_t next = (rxHead + 1) % RX_BUF_SIZE;
        if (next != rxTail) {
            rxBuf[rxHead] = data;
            rxHead = next;
        }
    }

    /* Clear overrun flag if set (read S1 then D to clear) */
    if (UART_GetStatusFlags(ESP_UART) & kUART_RxOverrunFlag) {
        (void)ESP_UART->S1;
        (void)ESP_UART->D;
    }
}

/* ========================= HELPERS ======================================= */

static void delay_us(uint32_t us)
{
    volatile uint32_t c = us * 12;
    while (c--) { __NOP(); }
}

static void delay_ms(uint32_t ms)
{
    volatile uint32_t c = ms * 6000;
    while (c--) { __NOP(); }
}

static void ultrasonic_trigger(void)
{
    GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 1U);
    delay_us(10);
    GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 0U);
}

static void servo_set(uint16_t pulse_us)
{
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    SERVO_TPM->CONTROLS[SERVO_CHANNEL].CnV = pulse_us / 16;
}

static void uart_send_byte(uint8_t b)
{
    while (!(UART_GetStatusFlags(ESP_UART) & kUART_TxDataRegEmptyFlag)) {}
    UART_WriteByte(ESP_UART, b);
}

/* Send distance packet to ESP32: [0xAA][0x01][dist_hi][dist_lo] */
static void send_distance(uint16_t dist_cm)
{
    uart_send_byte(0xAA);
    uart_send_byte(0x01);
    uart_send_byte((dist_cm >> 8) & 0xFF);
    uart_send_byte(dist_cm & 0xFF);
}

/* Parse incoming bytes for ESP32 response: [0xBB][0x01][status] */
static void parse_rx(void)
{
    static uint8_t state = 0;
    static uint8_t rxType = 0;

    while (rxHead != rxTail) {
        uint8_t byte = rxBuf[rxTail];
        rxTail = (rxTail + 1) % RX_BUF_SIZE;

        switch (state) {
        case 0:
            if (byte == 0xBB) state = 1;
            break;
        case 1:
            rxType = byte;
            state = 2;
            break;
        case 2:
            if (rxType == 0x01) {
                petDetected = byte;
                PRINTF("[RX] Pet status from ESP32: %s\r\n",
                       petDetected ? "PET NEAR" : "NO PET");
            }
            state = 0;
            break;
        default:
            state = 0;
            break;
        }
    }
}

/* ========================= MAIN ========================================== */

int main(void)
{
    gpio_pin_config_t outCfg = { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0 };
    gpio_pin_config_t inCfg  = { .pinDirection = kGPIO_DigitalInput,  .outputLogic = 0 };
    tpm_config_t tpmCfg;
    uart_config_t uartCfg;

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    SysTick_Config(SystemCoreClock / 1000);

    PRINTF("\r\n========================================\r\n");
    PRINTF("  PetPal v2 - MCXC444 Test\r\n");
    PRINTF("  Ultrasonic + Servo + LED + UART2\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable port and peripheral clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortE);
    CLOCK_EnableClock(kCLOCK_Uart2);

    /* --- Ultrasonic pins --- */
    PORT_SetPinMux(TRIG_PORT, TRIG_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(TRIG_GPIO, TRIG_PIN, &outCfg);
    PORT_SetPinMux(ECHO_PORT, ECHO_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(ECHO_GPIO, ECHO_PIN, &inCfg);
    PORT_SetPinInterruptConfig(ECHO_PORT, ECHO_PIN, kPORT_InterruptEitherEdge);
    EnableIRQ(PORTA_IRQn);
    PRINTF("[INIT] Ultrasonic: Trig=PTA%d, Echo=PTA%d (INT)\r\n", TRIG_PIN, ECHO_PIN);

    /* --- Onboard LED (active-low, start ON) --- */
    PORT_SetPinMux(LED_PORT, LED_PIN, kPORT_MuxAsGpio);
    gpio_pin_config_t ledCfg = { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0 };
    GPIO_PinInit(LED_GPIO, LED_PIN, &ledCfg);
    LED_ON();
    PRINTF("[INIT] Onboard LED: PTB%d (active-low, starting ON)\r\n", LED_PIN);

    /* --- Servo PWM --- */
    CLOCK_SetTpmClock(1U);
    PORT_SetPinMux(SERVO_PORT, SERVO_PIN, kPORT_MuxAlt4);
    TPM_GetDefaultConfig(&tpmCfg);
    tpmCfg.prescale = kTPM_Prescale_Divide_128;
    TPM_Init(SERVO_TPM, &tpmCfg);
    SERVO_TPM->CONTROLS[SERVO_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    SERVO_TPM->CONTROLS[SERVO_CHANNEL].CnV = SERVO_CENTER / 16;
    SERVO_TPM->MOD = SERVO_MOD;
    TPM_StartTimer(SERVO_TPM, kTPM_SystemClock);
    PRINTF("[INIT] Servo: PTC%d (TPM0_CH%d, 50Hz)\r\n", SERVO_PIN, SERVO_CHANNEL);

    /* --- UART2 to ESP32 on PTE22 (TX) / PTE23 (RX) --- */
    PORT_SetPinMux(ESP_TX_PORT, ESP_TX_PIN, kPORT_MuxAlt4);
    PORT_SetPinMux(ESP_RX_PORT, ESP_RX_PIN, kPORT_MuxAlt4);

    UART_GetDefaultConfig(&uartCfg);
    uartCfg.baudRate_Bps = ESP_UART_BAUD;
    uartCfg.enableTx = true;
    uartCfg.enableRx = true;
    UART_Init(ESP_UART, &uartCfg, CLOCK_GetBusClkFreq());

    /* Enable UART2 RX interrupt */
    UART_EnableInterrupts(ESP_UART, kUART_RxDataRegFullInterruptEnable |
                                     kUART_RxOverrunInterruptEnable);
    EnableIRQ(UART2_FLEXIO_IRQn);

    PRINTF("[INIT] UART2: TX=PTE%d, RX=PTE%d (%d baud)\r\n\r\n",
           ESP_TX_PIN, ESP_RX_PIN, ESP_UART_BAUD);

    PRINTF("[RUN] LED is ON. Measuring distance every 200ms.\r\n");
    PRINTF("[RUN] When ESP32 says pet is near, LED turns OFF + servo sweeps.\r\n\r\n");

    /* Servo sweep state */
    uint16_t sweepPos = SERVO_CENTER;
    int16_t  sweepDir = 20;
    uint8_t  sweeping = 0;
    uint32_t cycle = 0;

    while (1) {
        cycle++;

        /* Trigger ultrasonic */
        ultrasonic_trigger();
        delay_ms(60);

        if (echoReady) {
            echoReady = 0;
            uint16_t dist = lastDistance;

            /* Send distance to ESP32 */
            send_distance(dist);

            PRINTF("#%04lu  Dist: %3d cm  Pet: %s  LED: %s\r\n",
                   cycle, dist,
                   petDetected ? "YES" : "no ",
                   petDetected ? "OFF" : "ON ");
        }

        /* Parse any incoming responses from ESP32 */
        parse_rx();

        /* Control LED based on pet status */
        if (petDetected) {
            LED_OFF();
            sweeping = 1;
        } else {
            LED_ON();
            sweeping = 0;
            servo_set(SERVO_CENTER);
        }

        /* Sweep servo when pet is detected */
        if (sweeping) {
            sweepPos += sweepDir;
            if (sweepPos >= SERVO_RIGHT) { sweepPos = SERVO_RIGHT; sweepDir = -sweepDir; }
            if (sweepPos <= SERVO_LEFT)  { sweepPos = SERVO_LEFT;  sweepDir = -sweepDir; }
            servo_set(sweepPos);
        }

        delay_ms(140);
    }
}
