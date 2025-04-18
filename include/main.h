#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <DHT20.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include <Server_Side_RPC.h>
#include <freertos/semphr.h>
#include <sstream>
#include <iomanip>


#include <vector>
#include <string>

// Khai báo vector để lưu trữ các mã thẻ đã quét
extern std::vector<std::pair<std::string, std::string>> registeredTags;


#define BUTTON_PIN 0
#define LED_PIN 48

// DHT 11 Set up
#define DHT_PIN 6
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

/// Wifi and Sever Information
const char *ssid = "MikaSoCute!!!";
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
constexpr const char RPC_REQUEST_METHOD[] = "CHECK_IN_OUT";
constexpr const char RPC_ROOM_KEY[] = "ROOM";
constexpr const char RPC_REQUEST_KEY[] = "REQUEST";

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
extern bool enableReadTask;

bool writeToNFCTag(const String &textToWrite);
std::vector<std::pair<std::string, std::string>> readFromNFCTag();
bool isUIDRegistered(const std::vector<std::pair<std::string, std::string>> &tags);
void printRegisteredTags();
void displayMenu();
void deleteTag();

void processCheckInOut(const JsonVariantConst &data, JsonDocument &response);
void wifiTask(void *pvParameters);
void SeverTask(void *pvParameters);
void readAndUnlockDoorTask(void *pvParameters);