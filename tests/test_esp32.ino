/*
 * =============================================================================
 * PetPal v2 - ESP32 Test
 * =============================================================================
 * Tests: Shock sensor (interrupt), Water level (ADC), DHT11, Buzzer (PWM),
 *        Laser (GPIO), UART bi-directional with MCXC444
 *
 * Behavior:
 *   - Receives ultrasonic distance packets from MCXC444 via UART
 *   - Determines if pet is near (< 30cm threshold)
 *   - Sends pet status back to MCXC444 (so it can toggle its LED)
 *   - When pet is detected: buzzer chirps, laser turns on
 *   - When pet leaves: laser off, buzzer off
 *   - Shock sensor triggers an interrupt -> logs event, buzzer acknowledges
 *   - Water level read every 2 seconds via analogRead
 *   - DHT11 read every 5 seconds
 *   - All readings printed to Serial Monitor
 *
 * Pin assignments:
 *   Shock sensor DO  : GPIO 13  (interrupt, falling edge)
 *   Water level AO   : GPIO 34  (ADC, input-only pin)
 *   DHT11 data       : GPIO 4
 *   Passive buzzer S : GPIO 25  (LEDC PWM)
 *   Laser emitter S  : GPIO 26  (digital output)
 *   UART TX -> MCX   : GPIO 17  (Serial2 TX)
 *   UART RX <- MCX   : GPIO 16  (Serial2 RX)
 *
 * Protocol (simple for testing):
 *   MCX -> ESP:  [0xAA] [0x01] [dist_hi] [dist_lo]
 *   ESP -> MCX:  [0xBB] [0x01] [status]   (0x01=pet near, 0x00=no pet)
 *
 * Required libraries:
 *   - DHT sensor library (Adafruit)
 * =============================================================================
 */

#include <DHT.h>

/* ========================= PIN DEFINITIONS =============================== */

#define SHOCK_PIN       13
#define WATER_PIN       34      /* ADC input-only pin */
#define DHT_PIN         4
#define BUZZER_PIN      25
#define LASER_PIN       26
#define UART_RX_PIN     16
#define UART_TX_PIN     17

/* ========================= CONFIGURATION ================================= */

#define UART_BAUD       115200
#define PET_NEAR_CM     30      /* Pet detected if distance < this */
#define PET_FAR_CM      60      /* Pet gone if distance > this */
#define WATER_LOW_THRESHOLD  500
#define SHOCK_DEBOUNCE_MS    200

/* Buzzer LEDC config */
#define BUZZER_LEDC_CHANNEL  0
#define BUZZER_LEDC_TIMER    0
#define BUZZER_LEDC_RES      8      /* 8-bit resolution */

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

/* Send pet status to MCXC444: [0xBB][0x01][status] */
void sendPetStatus(bool near)
{
    uint8_t buf[3];
    buf[0] = 0xBB;
    buf[1] = 0x01;
    buf[2] = near ? 0x01 : 0x00;
    Serial2.write(buf, 3);
}

/*
 * Parse incoming UART bytes from MCXC444.
 * Expected: [0xAA][0x01][dist_hi][dist_lo]
 */
void parseUARTByte(uint8_t byte)
{
    switch (parseState) {
    case 0:  /* Wait for start byte */
        if (byte == 0xAA) parseState = 1;
        break;
    case 1:  /* Type byte */
        parseType = byte;
        parseState = 2;
        break;
    case 2:  /* Data high byte */
        parseHi = byte;
        parseState = 3;
        break;
    case 3:  /* Data low byte */
        if (parseType == 0x01) {
            lastDistance = (parseHi << 8) | byte;

            /* Determine pet status */
            if (lastDistance < PET_NEAR_CM) {
                petNear = true;
            } else if (lastDistance > PET_FAR_CM) {
                petNear = false;
            }
            /* Between PET_NEAR and PET_FAR: keep previous state (hysteresis) */

            /* Send status back to MCXC444 */
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
    /* Debug serial */
    Serial.begin(115200);
    Serial.println();
    Serial.println("========================================");
    Serial.println("  PetPal v2 - ESP32 Test");
    Serial.println("  Shock + Water + DHT + Buzzer + Laser");
    Serial.println("========================================");
    Serial.println();

    /* UART to MCXC444 */
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[INIT] UART: TX=GPIO%d, RX=GPIO%d (%d baud)\n",
                  UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    /* Shock sensor (interrupt on falling edge) */
    pinMode(SHOCK_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);
    Serial.printf("[INIT] Shock sensor: GPIO%d (interrupt, falling edge)\n", SHOCK_PIN);

    /* Water level (ADC) */
    pinMode(WATER_PIN, INPUT);
    analogReadResolution(12);   /* 12-bit: 0-4095 */
    Serial.printf("[INIT] Water level: GPIO%d (ADC, 12-bit)\n", WATER_PIN);

    /* DHT11 */
    dht.begin();
    Serial.printf("[INIT] DHT11: GPIO%d\n", DHT_PIN);

    /* Buzzer (LEDC PWM) */
    ledcSetup(BUZZER_LEDC_CHANNEL, 1000, BUZZER_LEDC_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
    ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);  /* Start silent */
    Serial.printf("[INIT] Buzzer: GPIO%d (LEDC channel %d)\n", BUZZER_PIN, BUZZER_LEDC_CHANNEL);

    /* Laser (GPIO) */
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    Serial.printf("[INIT] Laser: GPIO%d (digital output)\n", LASER_PIN);

    Serial.println();
    Serial.println("[RUN] Waiting for distance data from MCXC444...");
    Serial.println("[RUN] Tap shock sensor to test interrupt.");
    Serial.println("[RUN] Dip water sensor to test ADC readings.");
    Serial.println();

    /* Startup self-test: quick buzzer chirp + laser flash */
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
}

/* ========================= MAIN LOOP ===================================== */

void loop()
{
    unsigned long now = millis();

    /* ---- 1. Read UART from MCXC444 ---- */
    while (Serial2.available()) {
        parseUARTByte(Serial2.read());
    }

    /* ---- 2. React to pet status changes ---- */
    if (petNear != prevPetNear) {
        prevPetNear = petNear;

        if (petNear) {
            Serial.printf("[PET] *** PET DETECTED *** (dist: %d cm)\n", lastDistance);
            buzzer_tone(1500, 300);    /* Welcome chirp */
            laser_on();                /* Turn on laser for play */
            Serial.println("[ACT] Laser ON, buzzer chirped");
        } else {
            Serial.printf("[PET] Pet left (dist: %d cm)\n", lastDistance);
            laser_off();
            buzzer_off();
            Serial.println("[ACT] Laser OFF, buzzer off");
        }
    }

    /* ---- 3. Handle shock sensor interrupt ---- */
    if (shockTriggered) {
        shockTriggered = false;
        Serial.printf("[SHOCK] Tap #%lu detected!\n", shockCount);
        buzzer_tone(2000, 100);  /* Quick high-pitched ack */
    }

    /* ---- 4. Read water level every 2 seconds ---- */
    if (now - lastWaterRead >= 2000) {
        lastWaterRead = now;
        lastWaterLevel = analogRead(WATER_PIN);

        const char *wStatus;
        if (lastWaterLevel < 100) {
            wStatus = "CRITICAL";
        } else if (lastWaterLevel < WATER_LOW_THRESHOLD) {
            wStatus = "LOW";
        } else {
            wStatus = "OK";
        }

        Serial.printf("[WATER] Level: %d / 4095  [%s]\n", lastWaterLevel, wStatus);

        /* Alert if water is low */
        if (lastWaterLevel < WATER_LOW_THRESHOLD && lastWaterLevel > 0) {
            buzzer_tone(500, 200);  /* Low warning tone */
        }
    }

    /* ---- 5. Read DHT11 every 5 seconds ---- */
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

    /* ---- 6. Status summary every 3 seconds ---- */
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
