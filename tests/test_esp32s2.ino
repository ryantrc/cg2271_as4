/*
 * =============================================================================
 * PetPal v2 - ESP32-S2 Mini Test
 * =============================================================================
 * Board: Wemos/LOLIN ESP32-S2 Mini
 *
 * Key differences from regular ESP32:
 *   - No GPIO 25, 26, 27 (don't exist on S2)
 *   - Serial  = USB CDC (debug output)
 *   - Serial1 = UART1 (to MCXC444) — NOT Serial2
 *   - ADC on GPIO 1-10 (ADC1), GPIO 11-20 (ADC2, conflicts with WiFi)
 *
 * Pin assignments:
 *   Shock sensor DO  : GPIO 5   (interrupt, falling edge)
 *   Water level AO   : GPIO 3   (ADC1_CH2)
 *   DHT11 data       : GPIO 11
 *   Passive buzzer S : GPIO 7   (LEDC PWM)
 *   Laser emitter S  : GPIO 9   (digital output)
 *   UART1 TX -> MCX  : GPIO 43  (TX pin on board)
 *   UART1 RX <- MCX  : GPIO 44  (RX pin on board)
 *
 * Protocol:
 *   MCX -> ESP:  [0xAA] [0x01] [dist_hi] [dist_lo]
 *   ESP -> MCX:  [0xBB] [0x01] [status]  (0x01=pet near, 0x00=no pet)
 *
 * Required library: DHT sensor library (Adafruit)
 * Board in Arduino IDE: "LOLIN S2 Mini" or "ESP32S2 Dev Module"
 * =============================================================================
 */

#include <DHT.h>

/* ========================= PIN DEFINITIONS =============================== */

#define SHOCK_PIN       5
#define WATER_PIN       3       /* ADC1_CH2 */
#define DHT_PIN         11
#define BUZZER_PIN      7
#define LASER_PIN       9
#define UART_RX_PIN     44      /* RX pin on board */
#define UART_TX_PIN     43      /* TX pin on board */

/* ========================= CONFIGURATION ================================= */

#define UART_BAUD       115200
#define PET_NEAR_CM     30
#define PET_FAR_CM      60
#define WATER_LOW_THRESHOLD  500
#define SHOCK_DEBOUNCE_MS    200

/* Buzzer LEDC config */
#define BUZZER_LEDC_CHANNEL  0
#define BUZZER_LEDC_RES      8

/* ========================= DHT SENSOR ==================================== */

#define DHT_TYPE        DHT11
DHT dht(DHT_PIN, DHT_TYPE);

/* ========================= STATE ========================================= */

volatile bool     shockTriggered   = false;
volatile uint32_t shockCount       = 0;
volatile unsigned long lastShockMs = 0;

uint16_t lastDistance     = 999;
bool     petNear          = false;
bool     prevPetNear      = false;
float    lastTemp         = 0;
float    lastHumidity     = 0;
uint16_t lastWaterLevel   = 0;

unsigned long lastDHTRead      = 0;
unsigned long lastWaterRead    = 0;
unsigned long lastStatusPrint  = 0;

/* UART parser state */
uint8_t parseState = 0;
uint8_t parseType  = 0;
uint8_t parseHi    = 0;

/* ========================= SHOCK ISR ===================================== */

void IRAM_ATTR shockISR()
{
    unsigned long now = millis();
    if ((now - lastShockMs) >= SHOCK_DEBOUNCE_MS) {
        lastShockMs = now;
        shockCount++;
        shockTriggered = true;
    }
}

/* ========================= BUZZER FUNCTIONS ============================== */

void buzzer_tone(uint16_t freq, uint16_t duration_ms)
{
    if (freq == 0) {
        ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
        return;
    }
    ledcWriteTone(BUZZER_LEDC_CHANNEL, freq);
    if (duration_ms > 0) {
        delay(duration_ms);
        ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
    }
}

void buzzer_off()
{
    ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
}

/* ========================= LASER FUNCTIONS =============================== */

void laser_on()  { digitalWrite(LASER_PIN, HIGH); }
void laser_off() { digitalWrite(LASER_PIN, LOW);  }

/* ========================= UART FUNCTIONS ================================ */

void sendPetStatus(bool near)
{
    uint8_t buf[3];
    buf[0] = 0xBB;
    buf[1] = 0x01;
    buf[2] = near ? 0x01 : 0x00;
    Serial1.write(buf, 3);
}

void parseUARTByte(uint8_t byte)
{
    switch (parseState) {
    case 0:
        if (byte == 0xAA) parseState = 1;
        break;
    case 1:
        parseType = byte;
        parseState = 2;
        break;
    case 2:
        parseHi = byte;
        parseState = 3;
        break;
    case 3:
        if (parseType == 0x01) {
            lastDistance = (parseHi << 8) | byte;

            if (lastDistance < PET_NEAR_CM) {
                petNear = true;
            } else if (lastDistance > PET_FAR_CM) {
                petNear = false;
            }

            sendPetStatus(petNear);
        }
        parseState = 0;
        break;
    default:
        parseState = 0;
        break;
    }
}

/* ========================= SETUP ========================================= */

void setup()
{
    /* USB CDC debug serial */
    Serial.begin(115200);
    delay(1000);  /* S2 Mini needs a moment for USB CDC to enumerate */
    Serial.println();
    Serial.println("========================================");
    Serial.println("  PetPal v2 - ESP32-S2 Mini Test");
    Serial.println("========================================");
    Serial.println();

    /* UART1 to MCXC444 */
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[INIT] UART1: TX=GPIO%d, RX=GPIO%d (%d baud)\n",
                  UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    /* Shock sensor */
    pinMode(SHOCK_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);
    Serial.printf("[INIT] Shock: GPIO%d (interrupt, falling)\n", SHOCK_PIN);

    /* Water level */
    pinMode(WATER_PIN, INPUT);
    analogReadResolution(12);
    Serial.printf("[INIT] Water: GPIO%d (ADC1_CH2, 12-bit)\n", WATER_PIN);

    /* DHT11 */
    dht.begin();
    Serial.printf("[INIT] DHT11: GPIO%d\n", DHT_PIN);

    /* Buzzer (LEDC PWM) */
    ledcSetup(BUZZER_LEDC_CHANNEL, 1000, BUZZER_LEDC_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
    Serial.printf("[INIT] Buzzer: GPIO%d (LEDC ch%d)\n", BUZZER_PIN, BUZZER_LEDC_CHANNEL);

    /* Laser */
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    Serial.printf("[INIT] Laser: GPIO%d\n", LASER_PIN);

    Serial.println();

    /* Self-test */
    Serial.println("[SELF-TEST] Buzzer chirp...");
    buzzer_tone(1000, 200);
    delay(200);
    buzzer_tone(1500, 200);
    delay(200);

    Serial.println("[SELF-TEST] Laser flash...");
    laser_on();
    delay(500);
    laser_off();

    Serial.println("[SELF-TEST] Done.\n");
    Serial.println("[RUN] Waiting for distance data from MCXC444...");
    Serial.println("[RUN] Tap shock sensor to test interrupt.\n");
}

/* ========================= MAIN LOOP ===================================== */

void loop()
{
    unsigned long now = millis();

    /* 1. Read UART from MCXC444 */
    while (Serial1.available()) {
        parseUARTByte(Serial1.read());
    }

    /* 2. React to pet status changes */
    if (petNear != prevPetNear) {
        prevPetNear = petNear;

        if (petNear) {
            Serial.printf("[PET] *** PET DETECTED *** (dist: %d cm)\n", lastDistance);
            buzzer_tone(1500, 300);
            laser_on();
            Serial.println("[ACT] Laser ON, buzzer chirped");
        } else {
            Serial.printf("[PET] Pet left (dist: %d cm)\n", lastDistance);
            laser_off();
            buzzer_off();
            Serial.println("[ACT] Laser OFF, buzzer off");
        }
    }

    /* 3. Shock sensor */
    if (shockTriggered) {
        shockTriggered = false;
        Serial.printf("[SHOCK] Tap #%lu detected!\n", shockCount);
        buzzer_tone(2000, 100);
    }

    /* 4. Water level every 2s */
    if (now - lastWaterRead >= 2000) {
        lastWaterRead = now;
        lastWaterLevel = analogRead(WATER_PIN);

        const char *wStatus;
        if (lastWaterLevel < 100) wStatus = "CRITICAL";
        else if (lastWaterLevel < WATER_LOW_THRESHOLD) wStatus = "LOW";
        else wStatus = "OK";

        Serial.printf("[WATER] Level: %d / 4095  [%s]\n", lastWaterLevel, wStatus);

        if (lastWaterLevel < WATER_LOW_THRESHOLD && lastWaterLevel > 0) {
            buzzer_tone(500, 200);
        }
    }

    /* 5. DHT11 every 5s */
    if (now - lastDHTRead >= 5000) {
        lastDHTRead = now;
        float t = dht.readTemperature();
        float h = dht.readHumidity();

        if (!isnan(t) && !isnan(h)) {
            lastTemp = t;
            lastHumidity = h;
            Serial.printf("[DHT] Temp: %.1f C, Humidity: %.1f %%\n", t, h);
        } else {
            Serial.println("[DHT] Read failed (check wiring)");
        }
    }

    /* 6. Status summary every 3s */
    if (now - lastStatusPrint >= 3000) {
        lastStatusPrint = now;
        Serial.println("--- STATUS ---");
        Serial.printf("  Distance:    %d cm\n", lastDistance);
        Serial.printf("  Pet near:    %s\n", petNear ? "YES" : "no");
        Serial.printf("  Water:       %d\n", lastWaterLevel);
        Serial.printf("  Temp:        %.1f C\n", lastTemp);
        Serial.printf("  Humidity:    %.1f %%\n", lastHumidity);
        Serial.printf("  Shock taps:  %lu\n", shockCount);
        Serial.printf("  Laser:       %s\n", petNear ? "ON" : "OFF");
        Serial.printf("  Uptime:      %lu s\n", now / 1000);
        Serial.println("--------------\n");
    }

    delay(10);
}
