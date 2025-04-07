#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "DHT20.h"
#include "DHT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include "SPI.h"
#include "MFRC522.h"
#include <ArduinoOTA.h>
#include "Server_Side_RPC.h"

#define BUTTON_PIN 0
#define LED_PIN 48

// DHT 11 Set up
#define DHT_PIN 6
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// RFID Set up
#define RFID_RST 17
#define RFID_MISO 10
#define RFID_MOSI 9
#define RFID_SCK 8
#define RFID_SS 7
MFRC522 rfid(RFID_SS, RFID_RST);
MFRC522::MIFARE_Key key;
byte nuidPICC[4];

void printHex(byte *buffer, byte bufferSize);
void printDec(byte *buffer, byte bufferSize);

/// Wifi and Sever Information
const char *ssid = "Redmi Note 11";
const char *password = "12345671";
const char *token_id = "es12alt4fa25ryy4yej4";
const char *thingBoard_Sever = "app.coreiot.io";

// Configure Sever
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;
constexpr const char RPC_SWITCH_METHOD[] = "POWER";
constexpr const char RPC_SWITCH_KEY[] = "State";

//Share Attributes Configure
constexpr const char LED_STATE_KEY[] = "POWER";
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};

// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

bool subscribed = false;

void processSwitchChange(const JsonVariantConst &data, JsonDocument &response) {
    Serial.println("Received the set switch method");
    // Process data
    const int switch_state = data[RPC_SWITCH_KEY];
    if (switch_state == 1)
    {
        Serial.println("Led is On");
        digitalWrite(LED_PIN, HIGH);
    }
    else if (switch_state == 0)
    {
        Serial.println("Led is Off");
        digitalWrite(LED_PIN, LOW);
    }

    if (tb.connected())
        tb.sendAttributeData(LED_STATE_KEY, switch_state);
}

int buttonState = 0;
int lastButtonState = 0; 
bool ledState = false;