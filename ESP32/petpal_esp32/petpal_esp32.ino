/*
 * =============================================================================
 * PetPal v2 - ESP32-S2 Unified (Final Integration)
 * =============================================================================
 * Sensors + Dashboard API + UART bridge to MCXC444
 *
 * Protocol with MCXC444:
 *   MCX -> ESP:  [0xAA][0x01][dist_hi][dist_lo]    (distance data)
 *   ESP -> MCX:  [0xBB][0x01][status]               (pet detection)
 *   ESP -> MCX:  [0xBB][0x10]                        (feed command)
 *   ESP -> MCX:  [0xBB][0x11]                        (play command)
 *   ESP -> MCX:  [0xBB][0x12]                        (stop command)
 *
 * Pins:
 *   GPIO 3  - Water level AO (ADC)
 *   GPIO 5  - Shock sensor DO (interrupt)
 *   GPIO 7  - Passive buzzer (LEDC PWM)
 *   GPIO 9  - Laser emitter (digital out)
 *   GPIO 11 - DHT11 data
 *   GPIO 17 - UART TX -> MCXC444 PTE23 (RX)
 *   GPIO 18 - UART RX <- MCXC444 PTE22 (TX)
 *
 * Board: ESP32S2 Dev Module, USB CDC On Boot: Enabled
 * Library: DHT sensor library (Adafruit)
 * =============================================================================
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>

/* ========================= CHANGE THESE ================================== */
#define WIFI_SSID "shiras 24 ultra"
#define WIFI_PASSWORD "test1234"
#define API_BASE_URL "http://10.71.16.139:4000" /* Your laptop LAN IP */
#define DEVICE_API_KEY "device_dev_key"
#define DEVICE_ID "esp32s2-petpal"

/* ========================= PINS ========================================== */
#define SHOCK_PIN 5
#define WATER_PIN 3
#define DHT_PIN 11
#define BUZZER_PIN 7
#define LASER_PIN 9
#define UART_RX_PIN 18
#define UART_TX_PIN 17

/* ========================= TIMING ======================================== */
#define UART_BAUD 115200
#define WIFI_TIMEOUT_MS 15000
#define WIFI_RETRY_MS 10000
#define TELEMETRY_INTERVAL 500
#define COMMAND_POLL_MS 500
#define WATER_READ_MS 500
#define DHT_READ_MS 2000
#define STATUS_PRINT_MS 2000
#define SHOCK_DEBOUNCE_MS 200
#define PET_NEAR_CM 30
#define PET_FAR_CM 60

/* ========================= COMMANDS TO MCXC444 =========================== */
#define CMD_PET_STATUS 0x01
#define CMD_FEED 0x10
#define CMD_PLAY 0x11
#define CMD_STOP 0x12

/* ========================= DHT =========================================== */
DHT dht(DHT_PIN, DHT11);

/* ========================= STATE ========================================= */

/* Sensors */
float lastTemp = 0;
float lastHumidity = 0;
uint16_t lastWater = 0;
uint16_t lastDistanceCm = 999;

/* Shock */
volatile bool shockFlag = false;
volatile uint32_t shockCount = 0;
volatile unsigned long lastShockMs = 0;
bool shockLatched = false;

/* Pet detection */
bool petNear = false;
bool prevPetNear = false;

/* Actuators */
bool laserOn = false;
bool buzzerOn = false;
bool playServoMoving = false;
bool feederTriggered = false;

/* Buzzer timed */
bool buzzerTimedMode = false;
uint32_t buzzerOffAt = 0;

/* Events */
String lastEvent = "boot";
String lastFeedTs = "";
String lastPlayTs = "";

/* Timing */
unsigned long lastWaterRead = 0;
unsigned long lastDhtRead = 0;
unsigned long lastTelemetryPost = 0;
unsigned long lastCommandPoll = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastWifiAttempt = 0;
unsigned long bootTime = 0;

/* UART parser state */
uint8_t parseState = 0;
uint8_t parseType = 0;
uint8_t parseHi = 0;

/* Text buffer for MCXC444 debug output */
char textBuf[128];
int textIdx = 0;

/* ========================= SHOCK ISR ===================================== */

void IRAM_ATTR shockISR()
{
    unsigned long now = millis();
    if ((now - lastShockMs) >= SHOCK_DEBOUNCE_MS)
    {
        lastShockMs = now;
        shockCount++;
        shockFlag = true;
    }
}

/* ========================= BUZZER ======================================== */

void buzzerTone(uint16_t freq, uint16_t durationMs)
{
    ledcWriteTone(BUZZER_PIN, freq);
    buzzerOn = true;
    if (durationMs > 0)
    {
        buzzerTimedMode = true;
        buzzerOffAt = millis() + durationMs;
    }
}

void buzzerOff()
{
    ledcWriteTone(BUZZER_PIN, 0);
    buzzerOn = false;
    buzzerTimedMode = false;
}

/* ========================= LASER ========================================= */

void setLaser(bool on)
{
    laserOn = on;
    digitalWrite(LASER_PIN, on ? HIGH : LOW);
}

/* ========================= UART TO MCXC444 =============================== */

void sendToMCX(uint8_t type)
{
    /* Send [0xBB][type] — no data byte */
    Serial1.write(0xBB);
    Serial1.write(type);
}

void sendToMCX(uint8_t type, uint8_t data)
{
    /* Send [0xBB][type][data] */
    Serial1.write(0xBB);
    Serial1.write(type);
    Serial1.write(data);
}

void sendPetStatus(bool near)
{
    sendToMCX(CMD_PET_STATUS, near ? 0x01 : 0x00);
}

/* ========================= UART FROM MCXC444 ============================= */
/*
 * Handles two types of incoming data:
 *   1. Binary: [0xAA][0x01][dist_hi][dist_lo]
 *   2. Text: debug lines from MCXC444 PRINTF ([DBG], [MCX], etc.)
 */

bool inBinaryPacket = false;
uint8_t binStep = 0;

void processSerial1()
{
    while (Serial1.available())
    {
        uint8_t b = Serial1.read();

        /* 0xAA starts a binary packet */
        if (b == 0xAA && !inBinaryPacket)
        {
            if (textIdx > 0)
            {
                textBuf[textIdx] = '\0';
                Serial0.println(textBuf);
                textIdx = 0;
            }
            inBinaryPacket = true;
            binStep = 0;
            continue;
        }

        /* Inside binary packet: [type][hi][lo] */
        if (inBinaryPacket)
        {
            switch (binStep)
            {
            case 0:
                parseType = b;
                binStep = 1;
                break;
            case 1:
                parseHi = b;
                binStep = 2;
                break;
            case 2:
                if (parseType == 0x01)
                {
                    lastDistanceCm = (parseHi << 8) | b;

                    /* Pet detection with hysteresis */
                    if (lastDistanceCm < PET_NEAR_CM)
                    {
                        petNear = true;
                    }
                    else if (lastDistanceCm > PET_FAR_CM)
                    {
                        petNear = false;
                    }

                    /* Send pet status back to MCXC444 */
                    sendPetStatus(petNear);
                }
                inBinaryPacket = false;
                break;
            }
            continue;
        }

        /* Text from MCXC444 PRINTF */
        if (b == '\n')
        {
            textBuf[textIdx] = '\0';
            if (textIdx > 0)
                Serial0.println(textBuf);
            textIdx = 0;
        }
        else if (b != '\r')
        {
            if (textIdx < 126)
                textBuf[textIdx++] = (char)b;
        }
    }
}

/* ========================= WIFI ========================================== */

void setupWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;

    Serial0.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttempt = millis();

    while (WiFi.status() != WL_CONNECTED &&
           (millis() - lastWifiAttempt) < WIFI_TIMEOUT_MS)
    {
        delay(100);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial0.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    }
    else
    {
        Serial0.println("[WIFI] Timeout. Will retry.");
    }
}

/* ========================= TIMESTAMP ===================================== */

String nowIso()
{
    unsigned long sec = millis() / 1000;
    char buf[32];
    snprintf(buf, sizeof(buf), "2026-01-01T%02lu:%02lu:%02luZ",
             (sec / 3600) % 24, (sec / 60) % 60, sec % 60);
    return String(buf);
}

/* ========================= TELEMETRY POST ================================ */

void postTelemetry()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    HTTPClient http;
    String url = String(API_BASE_URL) + "/device/telemetry";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", DEVICE_API_KEY);

    unsigned long uptimeSec = (millis() - bootTime) / 1000;

    String json = "{";
    json += "\"deviceId\":\"" + String(DEVICE_ID) + "\",";
    json += "\"mode\":\"normal\",";
    json += "\"uptimeSec\":" + String(uptimeSec) + ",";

    if (lastTemp != 0 || lastHumidity != 0)
    {
        json += "\"temperatureC\":" + String(lastTemp, 1) + ",";
        json += "\"humidityPct\":" + String(lastHumidity, 1) + ",";
    }
    else
    {
        json += "\"temperatureC\":null,";
        json += "\"humidityPct\":null,";
    }

    json += "\"distanceCm\":" + String(lastDistanceCm) + ",";
    json += "\"shockDetected\":" + String(shockLatched ? "true" : "false") + ",";
    json += "\"waterLevelRaw\":" + String(lastWater) + ",";
    json += "\"laserOn\":" + String(laserOn ? "true" : "false") + ",";
    json += "\"playServoMoving\":" + String(playServoMoving ? "true" : "false") + ",";
    json += "\"feederTriggered\":" + String(feederTriggered ? "true" : "false") + ",";
    json += "\"buzzerOn\":" + String(buzzerOn ? "true" : "false") + ",";
    json += "\"lastEvent\":\"" + lastEvent + "\",";
    json += "\"lastFeedTs\":" + (lastFeedTs.length() > 0 ? ("\"" + lastFeedTs + "\"") : "null") + ",";
    json += "\"lastPlayTs\":" + (lastPlayTs.length() > 0 ? ("\"" + lastPlayTs + "\"") : "null");
    json += "}";

    int code = http.POST(json);
    if (code == 201)
    {
        shockLatched = false;
        feederTriggered = false;
    }
    else if (code > 0)
    {
        Serial0.printf("[API] Telemetry HTTP %d: %s\n", code, http.getString().c_str());
    }
    else
    {
        Serial0.printf("[API] Telemetry fail: %s\n", http.errorToString(code).c_str());
    }
    http.end();
}

/* ========================= COMMAND POLLING ================================ */

void ackCommand(String cmdId, bool success)
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    HTTPClient http;
    String url = String(API_BASE_URL) + "/device/commands/" + cmdId + "/ack";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-api-key", DEVICE_API_KEY);

    String json = "{\"status\":\"" + String(success ? "executed" : "failed") + "\"}";
    int code = http.POST(json);

    if (code == 200)
    {
        Serial0.printf("[API] ACK sent: %s\n", cmdId.c_str());
    }
    else
    {
        Serial0.printf("[API] ACK fail HTTP %d\n", code);
    }
    http.end();
}

void pollCommands()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    HTTPClient http;
    String url = String(API_BASE_URL) + "/device/commands/next";
    http.begin(url);
    http.addHeader("x-api-key", DEVICE_API_KEY);

    int code = http.GET();
    if (code == 204)
    {
        http.end();
        return; /* No pending commands */
    }

    if (code != 200)
    {
        if (code > 0)
            Serial0.printf("[API] Cmd poll HTTP %d\n", code);
        else
            Serial0.printf("[API] Cmd poll fail: %s\n", http.errorToString(code).c_str());
        http.end();
        return;
    }

    String body = http.getString();
    http.end();

    /* Parse command id and type */
    String cmdId = "";
    String cmdType = "";

    int idIdx = body.indexOf("\"id\":\"");
    if (idIdx >= 0)
    {
        idIdx += 6;
        int idEnd = body.indexOf("\"", idIdx);
        if (idEnd > idIdx)
            cmdId = body.substring(idIdx, idEnd);
    }

    int typeIdx = body.indexOf("\"type\":\"");
    if (typeIdx >= 0)
    {
        typeIdx += 8;
        int typeEnd = body.indexOf("\"", typeIdx);
        if (typeEnd > typeIdx)
            cmdType = body.substring(typeIdx, typeEnd);
    }

    if (cmdId.length() == 0 || cmdType.length() == 0)
    {
        Serial0.println("[API] Failed to parse command");
        return;
    }

    Serial0.printf("[CMD] id=%s type=%s\n", cmdId.c_str(), cmdType.c_str());

    bool success = false;

    if (cmdType == "feed_now")
    {
        /* Send feed command to MCXC444 */
        sendToMCX(CMD_FEED);
        buzzerTone(1000, 500);
        feederTriggered = true;
        lastFeedTs = nowIso();
        lastEvent = "feed_now";
        success = true;
        Serial0.println("[CMD] -> MCXC444: FEED");
    }
    else if (cmdType == "play_mode_toggle")
    {
        if (!playServoMoving)
        {
            /* Start play */
            setLaser(true);
            sendToMCX(CMD_PLAY);
            playServoMoving = true;
            lastPlayTs = nowIso();
            lastEvent = "play_start";
            Serial0.println("[CMD] -> MCXC444: PLAY ON + laser ON");
        }
        else
        {
            /* Stop play */
            setLaser(false);
            sendToMCX(CMD_STOP);
            playServoMoving = false;
            lastEvent = "play_stop";
            Serial0.println("[CMD] -> MCXC444: PLAY OFF + laser OFF");
        }
        success = true;
    }

    ackCommand(cmdId, success);
}

/* ========================= SENSOR READING ================================ */

void readSensors()
{
    unsigned long now = millis();

    /* Shock */
    if (shockFlag)
    {
        noInterrupts();
        shockFlag = false;
        interrupts();
        shockLatched = true;
        lastEvent = "shock";
        buzzerTone(2400, 120);
        Serial0.printf("[SHOCK] Tap #%lu\n", shockCount);
    }

    /* Water */
    if (now - lastWaterRead >= WATER_READ_MS)
    {
        lastWaterRead = now;
        lastWater = analogRead(WATER_PIN);
    }

    /* DHT */
    if (now - lastDhtRead >= DHT_READ_MS)
    {
        lastDhtRead = now;
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        if (!isnan(t) && !isnan(h))
        {
            lastTemp = t;
            lastHumidity = h;
        }
    }

    /* Buzzer auto-off */
    if (buzzerTimedMode && now >= buzzerOffAt)
    {
        buzzerOff();
    }

    /* Pet arrival/departure events */
    if (petNear != prevPetNear)
    {
        prevPetNear = petNear;
        if (petNear)
        {
            lastEvent = "pet_arrived";
            buzzerTone(1500, 300);
            Serial0.printf("[PET] *** DETECTED *** dist: %d cm\n", lastDistanceCm);
        }
        else
        {
            lastEvent = "pet_left";
            Serial0.printf("[PET] Left. dist: %d cm\n", lastDistanceCm);
        }
    }
}

/* ========================= SETUP ========================================= */

void setup()
{
    Serial0.begin(115200);
    delay(3000);
    bootTime = millis();

    Serial0.println("\n========================================");
    Serial0.println("  PetPal ESP32-S2 Unified");
    Serial0.println("  Sensors + API + UART Bridge");
    Serial0.println("========================================\n");

    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial0.printf("[INIT] UART: TX=%d RX=%d\n", UART_TX_PIN, UART_RX_PIN);

    pinMode(SHOCK_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);
    pinMode(WATER_PIN, INPUT);
    analogReadResolution(12);
    dht.begin();

    ledcAttach(BUZZER_PIN, 1000, 8);
    ledcWriteTone(BUZZER_PIN, 0);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);

    Serial0.printf("[INIT] Shock=%d Water=%d DHT=%d Buzzer=%d Laser=%d\n",
                   SHOCK_PIN, WATER_PIN, DHT_PIN, BUZZER_PIN, LASER_PIN);

    /* Self-test */
    Serial0.println("[TEST] Buzzer...");
    buzzerTone(1000, 200);
    delay(300);
    buzzerOff();
    Serial0.println("[TEST] Laser...");
    setLaser(true);
    delay(500);
    setLaser(false);

    setupWiFi();

    Serial0.printf("[INIT] API: %s\n", API_BASE_URL);
    Serial0.println("[INIT] Ready.\n");
}

/* ========================= LOOP ========================================== */

void loop()
{
    unsigned long now = millis();

    /* WiFi reconnect */
    if (WiFi.status() != WL_CONNECTED && (now - lastWifiAttempt) >= WIFI_RETRY_MS)
    {
        setupWiFi();
    }

    /* UART from MCXC444 (distance + debug text) */
    processSerial1();

    /* Read local sensors + check pet status changes */
    readSensors();

    /* POST telemetry to dashboard */
    if (now - lastTelemetryPost >= TELEMETRY_INTERVAL)
    {
        lastTelemetryPost = now;
        postTelemetry();
    }

    /* Poll dashboard for commands */
    if (now - lastCommandPoll >= COMMAND_POLL_MS)
    {
        lastCommandPoll = now;
        pollCommands();
    }

    /* Status print */
    if (now - lastStatusPrint >= STATUS_PRINT_MS)
    {
        lastStatusPrint = now;
        Serial0.println("--- STATUS ---");
        Serial0.printf("  Dist:   %d cm\n", lastDistanceCm);
        Serial0.printf("  Pet:    %s\n", petNear ? "YES" : "no");
        Serial0.printf("  Temp:   %.1fC\n", lastTemp);
        Serial0.printf("  Humid:  %.1f%%\n", lastHumidity);
        Serial0.printf("  Water:  %d\n", lastWater);
        Serial0.printf("  Shock:  %lu\n", shockCount);
        Serial0.printf("  Laser:  %s\n", laserOn ? "ON" : "OFF");
        Serial0.printf("  Play:   %s\n", playServoMoving ? "YES" : "no");
        Serial0.printf("  WiFi:   %s\n", WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
        Serial0.printf("  Event:  %s\n", lastEvent.c_str());
        Serial0.println("--------------\n");
    }

    delay(10);
}
