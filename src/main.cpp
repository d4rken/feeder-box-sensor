#include <../Config.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Si7021.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>

//for LED status
#include <Ticker.h>
Ticker ticker;

const char *SSID = WIFI_SSID;
const char *PSK = WIFI_PW;
const char *MQTT_BROKER = MQTT_SERVER;
const char *MQTT_BROKER_IP = MQTT_SERVER_IP;
const char *CLIENT_NAME = "Feeder-Box-Sensor";

Adafruit_Si7021 sensor = Adafruit_Si7021();

WiFiClient espClient;
PubSubClient client(espClient);

// CRC function used to ensure data validity
uint32_t calculateCRC32(const uint8_t *data, size_t length);

struct {
    unsigned long millis;
} rtcDataStruct;

struct {
    uint32_t crc32;
    byte data[508];
} rtcData;

void tick() {
    //toggle state
    int state = digitalRead(BUILTIN_LED); // get the current state of GPIO1 pin
    digitalWrite(BUILTIN_LED, !state);    // set pin to the opposite state
}

void setup() {
    Serial.begin(74880);
    Serial.println("Setup...");

    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, HIGH);

    // Read struct from RTC memory
    if (ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData))) {
        Serial.println("Reading rtcData");

        uint32_t crcOfData = calculateCRC32((uint8_t *)&rtcData.data[0], sizeof(rtcData.data));
        Serial.print("CRC32 of data: ");
        Serial.println(crcOfData, HEX);
        Serial.print("CRC32 read from RTC: ");
        Serial.println(rtcData.crc32, HEX);
        if (crcOfData != rtcData.crc32) {
            Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
        } else {
            Serial.println("CRC32 check ok, data is probably valid.");
            memcpy(&rtcDataStruct, rtcData.data, sizeof(rtcDataStruct));
        }
    }

    Serial.println("Connecting to WiFi" + String(SSID));
    WiFi.setAutoReconnect(true);
    WiFi.hostname(CLIENT_NAME);
    WiFi.begin(SSID, PSK);

    client.setServer(MQTT_BROKER, 1883);

    if (!sensor.begin()) {
        Serial.println("Could not find a valid  sensor, check wiring!");
    }

    pinMode(A0, INPUT);
}

char itoaBuf[64];
char dtostrfBuf[64];

unsigned int deepSleepMillis = 60000;

void updateSystemStats() {
    long rssi = WiFi.RSSI();
    client.publish("atc-adele-box/sensor/rssi", itoa(rssi, itoaBuf, 10));
    Serial.println("RSSI: " + String(rssi));

    unsigned long uptimeMillis = rtcDataStruct.millis + deepSleepMillis + millis();
    client.publish("atc-adele-box/sensor/uptime/milliseconds", itoa(uptimeMillis, itoaBuf, 10));

    Serial.println("Uptime seconds: " + String(uptimeMillis / 1000));
}

void updateSensor() {
    float temperature = sensor.readTemperature();
    client.publish("atc-adele-box/temperature/celsius", dtostrf(temperature, 4, 2, dtostrfBuf));
    Serial.println("Temperature = " + String(temperature) + "*C");

    float humidity = sensor.readHumidity();
    client.publish("atc-adele-box/humidity/percent", dtostrf(humidity, 4, 2, dtostrfBuf));
    Serial.println("Humidity = " + String(humidity) + "%");
}

bool ensureConnection() {
    bool wifiConnected = true;
    int retryWifi = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        if (retryWifi > 10) {
            wifiConnected = false;
            break;
        } else {
            retryWifi++;
        }
        delay(500);
    }

    if (wifiConnected) {
        Serial.println("Connected, my IP is:");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Failed to connect to WiFi");
    }

    bool mqttConnected = wifiConnected;
    int retryMqtt = 0;
    while (wifiConnected && !client.connected()) {
        Serial.println("Connecting to MQTT broker...");
        client.connect(CLIENT_NAME);
        if (retryMqtt > 3) {
            mqttConnected = false;
            Serial.println("Failed to connect to MQTT broker");
            break;
        } else {
            if (retryMqtt == 2) {
                Serial.println("MQTT DNS not resolved, trying IP...");
                client.disconnect();
                client.setServer(MQTT_BROKER_IP, 1883);
            }
            retryMqtt++;
        }
        delay(500);
    }

    return mqttConnected;
}

void loop() {
    ticker.attach(0.8, tick);
    bool isConnected = ensureConnection();

    if (!isConnected) {
        Serial.println("Not connected to WiFi/MQTT");
    }

    updateSystemStats();
    updateSensor();

    client.loop();
    delay(1000);

    Serial.println("Data published, waiting for transmission...");
    client.disconnect();
    espClient.flush();

    // wait until connection is closed completely
    int delayCounter = 0;
    while (client.state() != -1) {
        delay(10);
        if (delayCounter > 100) {
            break;
        } else {
            delayCounter++;
        }
    }
    Serial.println("Network flushed and disconnected, going to deep sleep...");

    // Generate new data set for the struct
    rtcDataStruct.millis = rtcDataStruct.millis + deepSleepMillis + millis();
    memcpy(rtcData.data, &rtcDataStruct, sizeof(rtcDataStruct));
    // Update CRC32 of data
    rtcData.crc32 = calculateCRC32((uint8_t *)&rtcData.data[0], sizeof(rtcData.data));
    // Write struct to RTC memory
    if (ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData))) {
        Serial.println("rtcData written");
    }

    ticker.detach();
    digitalWrite(BUILTIN_LED, LOW);

    unsigned long int actSleepTime = deepSleepMillis * 1000; //microseconds
    Serial.println("Going to sleep for " + String(deepSleepMillis) + "ms");
    ESP.deepSleep(actSleepTime);
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xffffffff;
    while (length--) {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1) {
            bool bit = crc & 0x80000000;
            if (c & i) {
                bit = !bit;
            }
            crc <<= 1;
            if (bit) {
                crc ^= 0x04c11db7;
            }
        }
    }
    return crc;
}