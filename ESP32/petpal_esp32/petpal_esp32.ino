/*
 * =============================================================================
 * PetPal v2 - ESP32-S2 Mini Test (Lightweight)
 * =============================================================================
 * NO WiFi, NO Firebase, NO OpenAI — just hardware testing.
 * Tests: Shock, Water level, DHT11, Buzzer, Laser, UART with MCXC444.
 *
 * Receives distance from MCXC444, determines pet status, sends back.
 * When pet near: buzzer chirps, laser on. When pet leaves: all off.
 *
 * Pins:
 *   GPIO 3  - Water level AO (ADC)
 *   GPIO 5  - Shock sensor DO (interrupt)
 *   GPIO 7  - Passive buzzer (LEDC PWM)
 *   GPIO 9  - Laser emitter (digital out)
 *   GPIO 11 - DHT11 data
 *   GPIO 43 - TX pin -> MCXC444 PTE23 (RX)
 *   GPIO 44 - RX pin -> MCXC444 PTE22 (TX)
 *
 * Library needed: "DHT sensor library" by Adafruit
 * Board: "LOLIN S2 Mini" or "ESP32S2 Dev Module"
 * =============================================================================
 */

#include <DHT.h>

/* ========================= PINS ========================================== */
#define SHOCK_PIN       5
#define WATER_PIN       3
#define DHT_PIN         11
#define BUZZER_PIN      7
#define LASER_PIN       9
#define UART_RX_PIN     18
#define UART_TX_PIN     17

/* ========================= CONFIG ======================================== */
#define UART_BAUD       115200
#define PET_NEAR_CM     30
#define PET_FAR_CM      60
#define WATER_LOW       500
#define SHOCK_DEBOUNCE  200

/* ========================= DHT =========================================== */
DHT dht(DHT_PIN, DHT11);

/* ========================= STATE ========================================= */
volatile bool     shockFlag    = false;
volatile uint32_t shockCount   = 0;
volatile unsigned long lastShockMs = 0;

uint16_t lastDistance    = 999;
bool     petNear         = false;
bool     prevPetNear     = false;
float    lastTemp        = 0;
float    lastHumidity    = 0;
uint16_t lastWater       = 0;

unsigned long lastDHT    = 0;
unsigned long lastWaterT = 0;
unsigned long lastStatus = 0;

/* UART parser */
uint8_t pState = 0, pType = 0, pHi = 0;

/* ========================= SHOCK ISR ===================================== */
void IRAM_ATTR shockISR() {
    unsigned long now = millis();
    if ((now - lastShockMs) >= SHOCK_DEBOUNCE) {
        lastShockMs = now;
        shockCount++;
        shockFlag = true;
    }
}

/* ========================= BUZZER ======================================== */
void buzzerTone(uint16_t freq, uint16_t ms) {
    ledcWriteTone(BUZZER_PIN, freq);
    delay(ms);
    ledcWriteTone(BUZZER_PIN, 0);
}

/* ========================= UART ========================================== */
void sendPetStatus(bool near) {
    uint8_t buf[3] = {0xBB, 0x01, near ? (uint8_t)0x01 : (uint8_t)0x00};
    Serial1.write(buf, 3);
}

void parseUART(uint8_t b) {
    switch (pState) {
    case 0: if (b == 0xAA) pState = 1; break;
    case 1: pType = b; pState = 2; break;
    case 2: pHi = b; pState = 3; break;
    case 3:
        if (pType == 0x01) {
            lastDistance = (pHi << 8) | b;
            if (lastDistance < PET_NEAR_CM) petNear = true;
            else if (lastDistance > PET_FAR_CM) petNear = false;
            sendPetStatus(petNear);
        }
        pState = 0;
        break;
    default: pState = 0; break;
    }
}

/* ========================= SETUP ========================================= */
void setup() {
    Serial0.begin(115200);
    delay(3000);  /* USB CDC needs time on S2 Mini */

    Serial0.println("\n========================================");
    Serial0.println("  PetPal v2 - ESP32-S2 Mini Test");
    Serial0.println("  Lightweight (no WiFi/Firebase)");
    Serial0.println("========================================\n");

    /* UART to MCXC444 */
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial0.printf("[INIT] UART1: TX=GPIO%d, RX=GPIO%d\n", UART_TX_PIN, UART_RX_PIN);

    /* Shock */
    pinMode(SHOCK_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);
    Serial0.printf("[INIT] Shock: GPIO%d\n", SHOCK_PIN);

    /* Water */
    pinMode(WATER_PIN, INPUT);
    analogReadResolution(12);
    Serial0.printf("[INIT] Water: GPIO%d\n", WATER_PIN);

    /* DHT */
    dht.begin();
    Serial0.printf("[INIT] DHT11: GPIO%d\n", DHT_PIN);

    /* Buzzer */
    // NEW:
    ledcAttach(BUZZER_PIN, 1000, 8);
    ledcWriteTone(BUZZER_PIN, 0);
    Serial0.printf("[INIT] Buzzer: GPIO%d\n", BUZZER_PIN);

    /* Laser */
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    Serial0.printf("[INIT] Laser: GPIO%d\n", LASER_PIN);

    /* Quick self-test */
    Serial0.println("\n[TEST] Buzzer...");
    buzzerTone(1000, 200);
    delay(100);
    buzzerTone(1500, 200);

    Serial0.println("[TEST] Laser flash...");
    digitalWrite(LASER_PIN, HIGH);
    delay(500);
    digitalWrite(LASER_PIN, LOW);

    Serial0.println("[TEST] Done. Waiting for MCXC444 data...\n");
}

/* ========================= LOOP ========================================== */
void loop() {
    unsigned long now = millis();

    /* 1. Read UART from MCXC444 */
    while (Serial1.available()) {
        parseUART(Serial1.read());
    }

    /* 2. Pet status changed */
    if (petNear != prevPetNear) {
        prevPetNear = petNear;
        if (petNear) {
            Serial0.printf("[PET] *** DETECTED *** dist: %d cm\n", lastDistance);
            buzzerTone(1500, 300);
            digitalWrite(LASER_PIN, HIGH);
        } else {
            Serial0.printf("[PET] Left. dist: %d cm\n", lastDistance);
            digitalWrite(LASER_PIN, LOW);
            ledcWriteTone(BUZZER_PIN, 0);
        }
    }

    /* 3. Shock sensor */
    if (shockFlag) {
        shockFlag = false;
        Serial0.printf("[SHOCK] Tap #%lu\n", shockCount);
        buzzerTone(2000, 100);
    }

    /* 4. Water level every 2s */
    if (now - lastWaterT >= 2000) {
        lastWaterT = now;
        lastWater = analogRead(WATER_PIN);
        Serial0.printf("[WATER] %d %s\n", lastWater,
                      lastWater < WATER_LOW ? "LOW!" : "OK");
    }

    /* 5. DHT every 5s */
    if (now - lastDHT >= 5000) {
        lastDHT = now;
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t) && !isnan(h)) {
            lastTemp = t; lastHumidity = h;
            Serial0.printf("[DHT] %.1fC  %.1f%%\n", t, h);
        } else {
            Serial0.println("[DHT] Read failed");
        }
    }

    /* 6. Status every 3s */
    if (now - lastStatus >= 3000) {
        lastStatus = now;
        Serial0.println("--- STATUS ---");
        Serial0.printf("  Dist:   %d cm\n", lastDistance);
        Serial0.printf("  Pet:    %s\n", petNear ? "YES" : "no");
        Serial0.printf("  Water:  %d\n", lastWater);
        Serial0.printf("  Temp:   %.1fC\n", lastTemp);
        Serial0.printf("  Humid:  %.1f%%\n", lastHumidity);
        Serial0.printf("  Taps:   %lu\n", shockCount);
        Serial0.printf("  Laser:  %s\n", petNear ? "ON" : "OFF");
        Serial0.println("--------------\n");
    }

    delay(10);
}
