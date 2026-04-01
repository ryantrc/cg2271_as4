/*
 * =============================================================================
 * PetPal v2 - ESP32 Individual Component Tests
 * =============================================================================
 * UNCOMMENT ONE TEST AT A TIME, upload, and verify.
 * Once all pass individually, use test_esp32.ino for the integrated test.
 *
 * Pin assignments:
 *   Shock sensor DO  : GPIO 13
 *   Water level AO   : GPIO 34  (ADC, input-only)
 *   DHT11 data       : GPIO 4
 *   Passive buzzer S : GPIO 25
 *   Laser emitter S  : GPIO 26
 * =============================================================================
 */

/* ==================== UNCOMMENT ONE TEST BELOW =========================== */

// #define TEST_BUZZER
// #define TEST_LASER
// #define TEST_WATER
// #define TEST_SHOCK
// #define TEST_DHT

/* ===========================================================================
 * TEST: BUZZER
 * Cycles through different frequencies. You should hear distinct tones.
 * Wiring: Buzzer S -> GPIO 25, Buzzer GND -> GND
 * =========================================================================== */
#ifdef TEST_BUZZER

#define BUZZER_PIN 25

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== BUZZER TEST ===");

    ledcSetup(0, 1000, 8);
    ledcAttachPin(BUZZER_PIN, 0);

    Serial.printf("Buzzer on GPIO %d\n", BUZZER_PIN);
}

void loop() {
    int freqs[] = {500, 800, 1000, 1500, 2000};
    const char* names[] = {"Water low (500)", "Alert (800)", "Food ready (1000)",
                           "Call pet (1500)", "Interaction (2000)"};

    for (int i = 0; i < 5; i++) {
        Serial.printf("Playing: %s Hz\n", names[i]);
        ledcWriteTone(0, freqs[i]);
        delay(500);
        ledcWriteTone(0, 0);
        delay(300);
    }

    Serial.println("--- Cycle done ---\n");
    delay(1000);
}

#endif

/* ===========================================================================
 * TEST: LASER
 * Toggles laser on/off every second.
 * Wiring: Laser S -> GPIO 26, Laser GND -> GND
 * WARNING: Do not look into the beam.
 * =========================================================================== */
#ifdef TEST_LASER

#define LASER_PIN 26

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== LASER TEST ===");
    Serial.println("WARNING: Do not look into the laser beam!");

    pinMode(LASER_PIN, OUTPUT);
    Serial.printf("Laser on GPIO %d\n\n", LASER_PIN);
}

void loop() {
    Serial.println("Laser ON");
    digitalWrite(LASER_PIN, HIGH);
    delay(1000);

    Serial.println("Laser OFF");
    digitalWrite(LASER_PIN, LOW);
    delay(1000);
}

#endif

/* ===========================================================================
 * TEST: WATER LEVEL
 * Reads ADC every 500ms and prints value + bar graph.
 * Use this to calibrate your thresholds.
 * Wiring: Water AO -> GPIO 34, VCC -> 3.3V, GND -> GND
 * =========================================================================== */
#ifdef TEST_WATER

#define WATER_PIN 34

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== WATER LEVEL TEST ===");

    pinMode(WATER_PIN, INPUT);
    analogReadResolution(12);
    Serial.printf("Water level on GPIO %d (ADC, 12-bit)\n", WATER_PIN);
    Serial.println("Calibration guide:");
    Serial.println("  1. Sensor in air (dry)     -> note LOW value");
    Serial.println("  2. Sensor fully submerged   -> note HIGH value");
    Serial.println("  3. Use values as thresholds\n");
}

void loop() {
    uint16_t val = analogRead(WATER_PIN);

    /* Simple bar graph */
    int bar = val * 40 / 4095;
    Serial.printf("Water: %4d  [", val);
    for (int i = 0; i < 40; i++) {
        Serial.print(i < bar ? '#' : ' ');
    }
    Serial.printf("]  %s\n", val < 500 ? "LOW!" : "OK");

    delay(500);
}

#endif

/* ===========================================================================
 * TEST: SHOCK / VIBRATION SENSOR
 * Waits for taps via interrupt. Prints event count and debounce stats.
 * Wiring: Shock DO -> GPIO 13, VCC -> 3.3V, GND -> GND
 * =========================================================================== */
#ifdef TEST_SHOCK

#define SHOCK_PIN 13
#define DEBOUNCE_MS 200

volatile uint32_t tapCount = 0;
volatile uint32_t rejectCount = 0;
volatile bool tapFlag = false;
volatile unsigned long lastTapMs = 0;

void IRAM_ATTR shockISR() {
    unsigned long now = millis();
    if ((now - lastTapMs) >= DEBOUNCE_MS) {
        lastTapMs = now;
        tapCount++;
        tapFlag = true;
    } else {
        rejectCount++;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== SHOCK SENSOR TEST ===");

    pinMode(SHOCK_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

    Serial.printf("Shock sensor on GPIO %d (interrupt, falling edge)\n", SHOCK_PIN);
    Serial.printf("Debounce: %d ms\n", DEBOUNCE_MS);
    Serial.println("Tap the sensor to test...\n");

    /* Show idle pin state */
    Serial.printf("Idle pin state: %d (expect 1 for active-low)\n\n",
                  digitalRead(SHOCK_PIN));
}

void loop() {
    if (tapFlag) {
        tapFlag = false;
        Serial.printf("[TAP #%lu] Detected! (rejected: %lu)\n",
                      tapCount, rejectCount);
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 5000) {
        lastPrint = millis();
        Serial.printf("[STATUS] Pin: %d, Taps: %lu, Rejected: %lu, Uptime: %lu s\n",
                      digitalRead(SHOCK_PIN), tapCount, rejectCount, millis() / 1000);
    }

    delay(10);
}

#endif

/* ===========================================================================
 * TEST: DHT11 TEMPERATURE & HUMIDITY
 * Reads every 2 seconds and prints values.
 * Wiring: DHT data -> GPIO 4, VCC -> 3.3V, GND -> GND
 *         (10K pullup resistor between data and VCC recommended)
 * Library: Install "DHT sensor library" by Adafruit via Library Manager
 * =========================================================================== */
#ifdef TEST_DHT

#include <DHT.h>

#define DHT_PIN  4
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== DHT11 TEST ===");

    dht.begin();
    Serial.printf("DHT11 on GPIO %d\n", DHT_PIN);
    Serial.println("Reading every 2 seconds...\n");
}

void loop() {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h)) {
        Serial.printf("Temp: %.1f C  Humidity: %.1f %%  ", t, h);

        /* Comfort assessment */
        if (t > 33) Serial.print("[TOO HOT]");
        else if (t < 18) Serial.print("[TOO COLD]");
        else Serial.print("[COMFORTABLE]");

        Serial.println();
    } else {
        Serial.println("Read FAILED. Check:");
        Serial.println("  - Wiring (data pin, VCC, GND)");
        Serial.println("  - 10K pullup resistor between data and VCC");
        Serial.println("  - DHT library installed");
    }

    delay(2000);
}

#endif

/* ===========================================================================
 * If nothing is #defined, print a help message
 * =========================================================================== */
#if !defined(TEST_BUZZER) && !defined(TEST_LASER) && !defined(TEST_WATER) && \
    !defined(TEST_SHOCK) && !defined(TEST_DHT)

void setup() {
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("  PetPal v2 - ESP32 Component Tests");
    Serial.println("========================================\n");
    Serial.println("No test selected! Uncomment ONE of these at the top of the file:");
    Serial.println("  #define TEST_BUZZER");
    Serial.println("  #define TEST_LASER");
    Serial.println("  #define TEST_WATER");
    Serial.println("  #define TEST_SHOCK");
    Serial.println("  #define TEST_DHT");
    Serial.println("\nThen upload again.");
}

void loop() { delay(10000); }

#endif
