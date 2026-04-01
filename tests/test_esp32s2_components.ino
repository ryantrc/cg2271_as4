/*
 * =============================================================================
 * PetPal v2 - ESP32-S2 Mini Individual Component Tests
 * =============================================================================
 * Board: Wemos/LOLIN ESP32-S2 Mini
 *
 * UNCOMMENT ONE TEST AT A TIME, upload, verify. Then use test_esp32s2.ino.
 *
 * Pin assignments (S2 Mini compatible):
 *   Shock sensor DO  : GPIO 5
 *   Water level AO   : GPIO 3   (ADC1_CH2)
 *   DHT11 data       : GPIO 11
 *   Passive buzzer S : GPIO 7
 *   Laser emitter S  : GPIO 9
 * =============================================================================
 */

/* ==================== UNCOMMENT ONE TEST BELOW =========================== */

// #define TEST_BUZZER
// #define TEST_LASER
// #define TEST_WATER
// #define TEST_SHOCK
// #define TEST_DHT
// #define TEST_UART_LOOPBACK

/* ===========================================================================
 * TEST: BUZZER (GPIO 7)
 * =========================================================================== */
#ifdef TEST_BUZZER

#define BUZZER_PIN 7

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== BUZZER TEST (ESP32-S2 Mini) ===");
    Serial.printf("Buzzer on GPIO %d\n\n", BUZZER_PIN);

    ledcSetup(0, 1000, 8);
    ledcAttachPin(BUZZER_PIN, 0);
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
 * TEST: LASER (GPIO 9)
 * =========================================================================== */
#ifdef TEST_LASER

#define LASER_PIN 9

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== LASER TEST (ESP32-S2 Mini) ===");
    Serial.println("WARNING: Do not look into the laser beam!");
    Serial.printf("Laser on GPIO %d\n\n", LASER_PIN);

    pinMode(LASER_PIN, OUTPUT);
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
 * TEST: WATER LEVEL (GPIO 3, ADC1_CH2)
 * =========================================================================== */
#ifdef TEST_WATER

#define WATER_PIN 3

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== WATER LEVEL TEST (ESP32-S2 Mini) ===");
    Serial.printf("Water level on GPIO %d (ADC1_CH2, 12-bit)\n\n", WATER_PIN);

    pinMode(WATER_PIN, INPUT);
    analogReadResolution(12);

    Serial.println("Calibration guide:");
    Serial.println("  Sensor in air  -> note LOW value");
    Serial.println("  Sensor in water -> note HIGH value");
    Serial.println();
}

void loop() {
    uint16_t val = analogRead(WATER_PIN);

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
 * TEST: SHOCK SENSOR (GPIO 5)
 * =========================================================================== */
#ifdef TEST_SHOCK

#define SHOCK_PIN 5
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
    delay(1000);
    Serial.println("\n=== SHOCK SENSOR TEST (ESP32-S2 Mini) ===");
    Serial.printf("Shock on GPIO %d (interrupt, falling edge)\n", SHOCK_PIN);
    Serial.printf("Debounce: %d ms\n\n", DEBOUNCE_MS);

    pinMode(SHOCK_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

    Serial.printf("Idle pin state: %d\n", digitalRead(SHOCK_PIN));
    Serial.println("Tap the sensor to test...\n");
}

void loop() {
    if (tapFlag) {
        tapFlag = false;
        Serial.printf("[TAP #%lu] Detected! (rejected: %lu)\n", tapCount, rejectCount);
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 5000) {
        lastPrint = millis();
        Serial.printf("[STATUS] Pin: %d, Taps: %lu, Rejected: %lu\n",
                      digitalRead(SHOCK_PIN), tapCount, rejectCount);
    }
    delay(10);
}

#endif

/* ===========================================================================
 * TEST: DHT11 (GPIO 11)
 * Needs: "DHT sensor library" by Adafruit
 * =========================================================================== */
#ifdef TEST_DHT

#include <DHT.h>

#define DHT_PIN  11
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== DHT11 TEST (ESP32-S2 Mini) ===");
    Serial.printf("DHT11 on GPIO %d\n\n", DHT_PIN);
    dht.begin();
}

void loop() {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h)) {
        Serial.printf("Temp: %.1f C  Humidity: %.1f %%  ", t, h);
        if (t > 33) Serial.print("[TOO HOT]");
        else if (t < 18) Serial.print("[TOO COLD]");
        else Serial.print("[OK]");
        Serial.println();
    } else {
        Serial.println("Read FAILED - check wiring + 10K pullup resistor");
    }
    delay(2000);
}

#endif

/* ===========================================================================
 * TEST: UART LOOPBACK (TX pin GPIO 43, RX pin GPIO 44)
 * Connect the TX and RX pins on the board with a jumper wire.
 * Sends a message and checks if it comes back. Proves UART1 works.
 * =========================================================================== */
#ifdef TEST_UART_LOOPBACK

#define UART_TX 43
#define UART_RX 44

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== UART1 LOOPBACK TEST (ESP32-S2 Mini) ===");
    Serial.printf("TX: GPIO%d (TX pin), RX: GPIO%d (RX pin)\n", UART_TX, UART_RX);
    Serial.println("Connect TX pin -> RX pin with a jumper wire.\n");

    Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
}

void loop() {
    /* Send test bytes */
    uint8_t testData[] = {0xAA, 0x01, 0x00, 0x1E};  /* Distance = 30cm */
    Serial1.write(testData, 4);
    Serial.printf("[TX] Sent: AA 01 00 1E\n");

    delay(100);

    /* Check if we got it back */
    if (Serial1.available() >= 4) {
        Serial.print("[RX] Got:  ");
        bool match = true;
        for (int i = 0; i < 4; i++) {
            uint8_t b = Serial1.read();
            Serial.printf("%02X ", b);
            if (b != testData[i]) match = false;
        }
        Serial.println(match ? " -> PASS" : " -> FAIL (mismatch!)");
    } else {
        int avail = Serial1.available();
        Serial.printf("[RX] Only %d bytes received -> FAIL (check wiring)\n", avail);
        while (Serial1.available()) Serial1.read();  /* Flush */
    }

    Serial.println();
    delay(2000);
}

#endif

/* ===========================================================================
 * No test selected
 * =========================================================================== */
#if !defined(TEST_BUZZER) && !defined(TEST_LASER) && !defined(TEST_WATER) && \
    !defined(TEST_SHOCK) && !defined(TEST_DHT) && !defined(TEST_UART_LOOPBACK)

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("  PetPal v2 - ESP32-S2 Mini Tests");
    Serial.println("========================================\n");
    Serial.println("No test selected! Uncomment ONE:");
    Serial.println("  #define TEST_BUZZER");
    Serial.println("  #define TEST_LASER");
    Serial.println("  #define TEST_WATER");
    Serial.println("  #define TEST_SHOCK");
    Serial.println("  #define TEST_DHT");
    Serial.println("  #define TEST_UART_LOOPBACK");
}

void loop() { delay(10000); }

#endif
