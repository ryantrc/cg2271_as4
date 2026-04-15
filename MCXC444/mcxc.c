/*
 * =============================================================================
 * PetPal v2 - MCXC444 (Final Integration)
 * =============================================================================
 * Ultrasonic + Servo + LED + UART2 to ESP32
 *
 * Sends distance to ESP32, receives:
 *   - Pet detection status (LED control)
 *   - Feed command from dashboard (servo opens gate)
 *   - Play command from dashboard (servo sweeps + laser on ESP32)
 *   - Stop command from dashboard (servo centers)
 *
 * Pins:
 *   HC-SR04 Trig  : PTD2  (GPIO output)
 *   HC-SR04 Echo  : PTD4  (GPIO input, interrupt both edges)
 *   Servo         : PTC2  (TPM0_CH1, PWM 50Hz)
 *   Onboard LED   : PTD5  (active-low green LED)
 *   UART2 TX      : PTE22 (Alt4) -> ESP32 GPIO 18 (RX)
 *   UART2 RX      : PTE23 (Alt4) -> ESP32 GPIO 17 (TX)
 *
 * Protocol:
 *   MCX -> ESP:  [0xAA][type][data...]
 *     0x01 = distance:  [0xAA][0x01][dist_hi][dist_lo]
 *
 *   ESP -> MCX:  [0xBB][type][data...]
 *     0x01 = pet status: [0xBB][0x01][0x00 or 0x01]
 *     0x10 = feed now:   [0xBB][0x10]
 *     0x11 = play start: [0xBB][0x11]
 *     0x12 = stop:       [0xBB][0x12]
 * =============================================================================
 */

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_tpm.h"
#include "fsl_uart.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"

/* ========================= PIN DEFINITIONS =============================== */

#define TRIG_PORT       PORTD
#define TRIG_GPIO       GPIOD
#define TRIG_PIN        2U

#define ECHO_PORT       PORTD
#define ECHO_GPIO       GPIOD
#define ECHO_PIN        4U

#define SERVO_TPM       TPM0
#define SERVO_PORT      PORTC

/* Food dispenser servo on PTC1 -> TPM0_CH0 */
#define FOOD_SERVO_PIN      1U
#define FOOD_SERVO_CHANNEL  0U

/* Laser sweep servo on PTC2 -> TPM0_CH1 */
#define LASER_SERVO_PIN     2U
#define LASER_SERVO_CHANNEL 1U

#define LED_PORT        PORTD
#define LED_GPIO        GPIOD
#define LED_PIN         5U
#define LED_ON()        GPIO_PinWrite(LED_GPIO, LED_PIN, 0U)
#define LED_OFF()       GPIO_PinWrite(LED_GPIO, LED_PIN, 1U)

#define ESP_UART        UART2
#define ESP_UART_BAUD   115200U
#define ESP_TX_PORT     PORTE
#define ESP_TX_PIN      22U
#define ESP_RX_PORT     PORTE
#define ESP_RX_PIN      23U

/* Servo positions */
#define SERVO_CENTER    1500
#define SERVO_LEFT      800
#define SERVO_RIGHT     2200
#define SERVO_FEED_OPEN 2000
#define SERVO_FEED_CLOSED 1000

#define TIMER_CLK       375000
#define SERVO_MOD       ((TIMER_CLK / 50) - 1)

/* Feed gate timing */
#define FEED_DURATION_MS 3000   /* Keep gate open for 3 seconds */

/* ========================= COMMAND TYPES FROM ESP32 ====================== */
#define CMD_PET_STATUS  0x01
#define CMD_FEED        0x10
#define CMD_PLAY        0x11
#define CMD_STOP        0x12

/* ========================= SYSTEM STATE ================================== */

typedef enum {
    MODE_IDLE,
    MODE_FEEDING,
    MODE_PLAYING
} SystemMode_t;

static volatile SystemMode_t currentMode = MODE_IDLE;
static volatile uint8_t petDetected = 0;
static volatile uint32_t feedStartMs = 0;

/* Ultrasonic */
static volatile uint32_t echoStartTick = 0;
static volatile uint16_t lastDistance = 999;
static volatile uint8_t  echoReady = 0;

/* UART RX */
#define RX_BUF_SIZE     32
static volatile uint8_t  rxBuf[RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;
static volatile uint16_t rxTail = 0;

/* Servo sweep */
static uint16_t sweepPos = SERVO_CENTER;
static int16_t  sweepDir = 20;

/* Millisecond counter for feed timing */
static volatile uint32_t sysMs = 0;

/* ========================= FREE-RUNNING SYSTICK ========================== */

static void start_systick_freerun(void)
{
    SysTick->LOAD = 0xFFFFFFUL;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

/* ========================= SIMPLE DELAYS ================================= */

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

/* Rough ms counter using delay loop (not precise but good enough for feed timing) */
static uint32_t approx_ms(void)
{
    return sysMs;
}

/* ========================= ISR: ULTRASONIC ECHO ========================== */

void PORTC_PORTD_IRQHandler(void)
{
    uint32_t flags = GPIO_PortGetInterruptFlags(ECHO_GPIO);

    if (flags & (1U << ECHO_PIN)) {
        if (GPIO_PinRead(ECHO_GPIO, ECHO_PIN)) {
            echoStartTick = SysTick->VAL;
        } else {
            uint32_t endTick = SysTick->VAL;
            uint32_t elapsed;

            if (echoStartTick >= endTick) {
                elapsed = echoStartTick - endTick;
            } else {
                elapsed = echoStartTick + (0xFFFFFFUL + 1 - endTick);
            }

            uint32_t pulse_us = elapsed / (SystemCoreClock / 1000000);

            if (pulse_us > 50 && pulse_us < 25000) {
                lastDistance = (uint16_t)(pulse_us / 58);
            }
            echoReady = 1;
        }
        GPIO_PortClearInterruptFlags(ECHO_GPIO, 1U << ECHO_PIN);
    }
}

/* ========================= ISR: UART2 RX ================================= */

void UART2_FLEXIO_IRQHandler(void)
{
    if (UART_GetStatusFlags(ESP_UART) & kUART_RxDataRegFullFlag) {
        uint8_t data = UART_ReadByte(ESP_UART);
        uint16_t next = (rxHead + 1) % RX_BUF_SIZE;
        if (next != rxTail) {
            rxBuf[rxHead] = data;
            rxHead = next;
        }
    }
    if (UART_GetStatusFlags(ESP_UART) & kUART_RxOverrunFlag) {
        (void)ESP_UART->S1;
        (void)ESP_UART->D;
    }
}

/* ========================= UART TX ======================================= */

static void uart_send_byte(uint8_t b)
{
    while (!(UART_GetStatusFlags(ESP_UART) & kUART_TxDataRegEmptyFlag)) {}
    UART_WriteByte(ESP_UART, b);
}

static void send_distance(uint16_t dist_cm)
{
    uart_send_byte(0xAA);
    uart_send_byte(0x01);
    uart_send_byte((dist_cm >> 8) & 0xFF);
    uart_send_byte(dist_cm & 0xFF);
}

/* ========================= UART RX PARSER ================================ */
/*
 * Protocol from ESP32: [0xBB][type][optional data byte]
 *
 * type 0x01: pet status, data = 0x00 (no pet) or 0x01 (pet near)
 * type 0x10: feed now (no data byte)
 * type 0x11: play start (no data byte)
 * type 0x12: stop all (no data byte)
 */
static void parse_rx(void)
{
    static uint8_t state = 0;
    static uint8_t rxType = 0;

    while (rxHead != rxTail) {
        uint8_t b = rxBuf[rxTail];
        rxTail = (rxTail + 1) % RX_BUF_SIZE;

        switch (state) {
        case 0:
            if (b == 0xBB) state = 1;
            break;

        case 1:
            rxType = b;

            if (rxType == CMD_PET_STATUS) {
                /* Next byte is the status data */
                state = 2;
            }
            else if (rxType == CMD_FEED) {
                PRINTF("[CMD] Feed from dashboard\r\n");
                currentMode = MODE_FEEDING;
                feedStartMs = sysMs;
                state = 0;
            }
            else if (rxType == CMD_PLAY) {
                PRINTF("[CMD] Play from dashboard\r\n");
                currentMode = MODE_PLAYING;
                sweepPos = SERVO_CENTER;
                state = 0;
            }
            else if (rxType == CMD_STOP) {
                PRINTF("[CMD] Stop from dashboard\r\n");
                currentMode = MODE_IDLE;
                state = 0;
            }
            else {
                state = 0;  /* Unknown type */
            }
            break;

        case 2:
            /* Pet status data byte */
            if (rxType == CMD_PET_STATUS) {
                petDetected = b;
            }
            state = 0;
            break;

        default:
            state = 0;
            break;
        }
    }
}

/* ========================= SERVO ========================================= */

static void food_servo_set(uint16_t pulse_us) {
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    SERVO_TPM->CONTROLS[FOOD_SERVO_CHANNEL].CnV = (pulse_us * 375) / 1000;
}

static void laser_servo_set(uint16_t pulse_us) {
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    SERVO_TPM->CONTROLS[LASER_SERVO_CHANNEL].CnV = (pulse_us * 375) / 1000;
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
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    start_systick_freerun();

    PRINTF("\r\n========================================\r\n");
    PRINTF("  PetPal v2 - MCXC444 Final\r\n");
    PRINTF("  Ultrasonic + Servo + LED + UART2\r\n");
    PRINTF("========================================\r\n\r\n");

    /* Enable clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);
    CLOCK_EnableClock(kCLOCK_Uart2);

    /* Ultrasonic */
    PORT_SetPinMux(TRIG_PORT, TRIG_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(TRIG_GPIO, TRIG_PIN, &outCfg);
    PORT_SetPinMux(ECHO_PORT, ECHO_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(ECHO_GPIO, ECHO_PIN, &inCfg);
    PORT_SetPinInterruptConfig(ECHO_PORT, ECHO_PIN, kPORT_InterruptEitherEdge);
    EnableIRQ(PORTC_PORTD_IRQn);
    PRINTF("[INIT] Ultrasonic: Trig=PTD%d, Echo=PTD%d\r\n", TRIG_PIN, ECHO_PIN);

    /* LED */
    PORT_SetPinMux(LED_PORT, LED_PIN, kPORT_MuxAsGpio);
    gpio_pin_config_t ledCfg = { .pinDirection = kGPIO_DigitalOutput, .outputLogic = 0 };
    GPIO_PinInit(LED_GPIO, LED_PIN, &ledCfg);
    LED_ON();
    PRINTF("[INIT] LED: PTD%d (ON)\r\n", LED_PIN);

    /* Servo */
    /* --- Servo PWM (Port C) --- */
    CLOCK_SetTpmClock(1U);

        /* MUST set pin mux AFTER BOARD_InitBootPins to override any conflicts */
	PORT_SetPinMux(SERVO_PORT, FOOD_SERVO_PIN,  kPORT_MuxAlt4);  /* PTC1 -> TPM0_CH0 */
	PORT_SetPinMux(SERVO_PORT, LASER_SERVO_PIN, kPORT_MuxAlt4);  /* PTC2 -> TPM0_CH1 */

	TPM_GetDefaultConfig(&tpmCfg);
	tpmCfg.prescale = kTPM_Prescale_Divide_128;
	TPM_Init(SERVO_TPM, &tpmCfg);

	/* Food servo channel */
	SERVO_TPM->CONTROLS[FOOD_SERVO_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	SERVO_TPM->CONTROLS[FOOD_SERVO_CHANNEL].CnV = SERVO_CENTER / 16;

	/* Laser servo channel */
	SERVO_TPM->CONTROLS[LASER_SERVO_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	SERVO_TPM->CONTROLS[LASER_SERVO_CHANNEL].CnV = SERVO_CENTER / 16;

	SERVO_TPM->MOD = SERVO_MOD;
	TPM_StartTimer(SERVO_TPM, kTPM_SystemClock);
	PRINTF("[INIT] Food servo: PTC%d (CH%d)\r\n", FOOD_SERVO_PIN, FOOD_SERVO_CHANNEL);
	PRINTF("[INIT] Laser servo: PTC%d (CH%d)\r\n", LASER_SERVO_PIN, LASER_SERVO_CHANNEL);

    /* UART2 */
    PORT_SetPinMux(ESP_TX_PORT, ESP_TX_PIN, kPORT_MuxAlt4);
    PORT_SetPinMux(ESP_RX_PORT, ESP_RX_PIN, kPORT_MuxAlt4);
    UART_GetDefaultConfig(&uartCfg);
    uartCfg.baudRate_Bps = ESP_UART_BAUD;
    uartCfg.enableTx = true;
    uartCfg.enableRx = true;
    UART_Init(ESP_UART, &uartCfg, CLOCK_GetBusClkFreq());
    UART_EnableInterrupts(ESP_UART, kUART_RxDataRegFullInterruptEnable |
                                     kUART_RxOverrunInterruptEnable);
    EnableIRQ(UART2_FLEXIO_IRQn);
    PRINTF("[INIT] UART2: TX=PTE%d, RX=PTE%d\r\n\r\n", ESP_TX_PIN, ESP_RX_PIN);

    PRINTF("[RUN] LED ON = no pet. LED OFF = pet detected.\r\n");
    PRINTF("[RUN] Dashboard can send feed/play/stop commands.\r\n\r\n");

    uint32_t cycle = 0;

    while (1) {
        cycle++;
        sysMs += 200;  /* Approximate ms tracking (200ms per loop) */

        /* Trigger ultrasonic */
        GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 1U);
        delay_us(10);
        GPIO_PinWrite(TRIG_GPIO, TRIG_PIN, 0U);
        delay_ms(60);

        /* Send distance to ESP32 */
        if (echoReady) {
            echoReady = 0;
            send_distance(lastDistance);

            PRINTF("#%04lu  Dist:%3dcm  Pet:%s  Mode:%s\r\n",
                   cycle, lastDistance,
                   petDetected ? "YES" : "no ",
                   currentMode == MODE_IDLE ? "IDLE" :
                   currentMode == MODE_FEEDING ? "FEED" : "PLAY");
        } else {
            send_distance(999);
        }

        /* Parse ESP32 responses and commands */
        parse_rx();

        /* === LED: OFF when pet detected, ON otherwise === */
        if (petDetected) {
            LED_OFF();
        } else {
            LED_ON();
        }

        /* === MODE-based servo control === */
        switch (currentMode) {
        case MODE_FEEDING:
            food_servo_set(SERVO_FEED_OPEN);
            if ((sysMs - feedStartMs) >= FEED_DURATION_MS) {
                food_servo_set(SERVO_FEED_CLOSED);
                delay_ms(500);
                food_servo_set(SERVO_CENTER);
                currentMode = MODE_IDLE;
                PRINTF("[FEED] Done, gate closed\r\n");
            }
            break;

        case MODE_PLAYING:
            sweepPos += sweepDir;
            if (sweepPos >= SERVO_RIGHT) { sweepPos = SERVO_RIGHT; sweepDir = -sweepDir; }
            if (sweepPos <= SERVO_LEFT)  { sweepPos = SERVO_LEFT;  sweepDir = -sweepDir; }
            laser_servo_set(sweepPos);
            break;

        case MODE_IDLE:
        default:
            food_servo_set(SERVO_CENTER);
            laser_servo_set(SERVO_CENTER);
            sweepPos = SERVO_CENTER;
            break;
        }
        delay_ms(140);
    }

    return 0;
}
