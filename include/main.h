#include <Arduino.h>
#define NFC_INTERFACE_I2C
#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532_I2C.cpp>
#include <PN532.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <ArduinoOTA.h>
#include <Server_Side_RPC.h>
#include <string.h>

using namespace std;

#define SDA_PIN 11
#define SCL_PIN 12

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

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
constexpr uint8_t NUMBER_OF_RPC_METHOD = 2;
const char* RPC_METHOD[] = {
    "CHECK_IN",
    "CHECK_OUT",
};
const char* RPC_KEY[] = {
    "REQUEST",
    "ROOM"
};

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
    const string request = data[RPC_KEY[0]];
    const string room = data[RPC_KEY[1]];
    Serial.println(request.c_str());
    if(request == "check-in")
    {
        Serial.println("Check-in request received"); 
        Serial.print("Room: "); Serial.println(room.c_str());
        tb.sendAttributeData("status", true);
    }
    else if (request == "check-out")
    {
        Serial.println("Check-out request received"); 
        Serial.print("Room: "); Serial.println(room.c_str());
        tb.sendAttributeData("status", false);
    }
    else
    {
        Serial.println("Invalid request");
    }
}

void wifiTask(void *pvParameters)
{
  // Initialize WiFi
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(ssid);
    Serial.println(password);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  vTaskDelete(NULL); // Delete the task when done
}

void SeverTask(void *pvParameters)
{
  if (!tb.connected())
  {
    Serial.println("Reconnecting to ThingsBoard...");
    while (!tb.connect(thingBoard_Sever, token_id, THINGSBOARD_PORT))
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      Serial.println("Failed to reconnect");
    }
  }
  if (!subscribed)
  {
    Serial.println("Subscribing for RPC...");
    const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
        RPC_Callback{RPC_METHOD[0], processSwitchChange}, // "CHECK_IN"
        RPC_Callback{RPC_METHOD[1], processSwitchChange}  // "CHECK_OUT"
    };
    // Perform a subscription. All consequent data processing will happen in
    // processTemperatureChange() and processSwitchChange() functions,
    // as denoted by callbacks array.
    if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
    {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }
  vTaskDelete(NULL);
}
void setup();
void loop();    