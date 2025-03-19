#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "DHT20.h"
#include "DHT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>

#define DHT_PIN 6
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
float temperature = 0;
float humidity = 0;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient);

// DHT20 dht;              // Create DHT20 sensor object

const char *ssid = "Redmi Note 11";
const char *password = "12345671";

const char *token_id = "es12alt4fa25ryy4yej4";
const char *thingBoard_Sever = "app.coreiot.io";
const int port = 1883;

void wifiTask(void *pvParameters)
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  vTaskDelete(NULL); // Delete the task when done
}

void SensorTask(void *pvParameters)
{
  dht.begin();
  while (1)
  {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("Failed to read from DHT sensor!");
    }
    else
    {
      Serial.print("Temp: ");
      Serial.print(temperature);
      Serial.print(" *C ");
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    tb.loop();
  }
}

void SeverTask(void *pvParameters)
{
  if (!tb.connected())
  {
    Serial.println("Reconnecting to ThingsBoard...");
    while (!tb.connect(thingBoard_Sever, token_id, port))
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      Serial.println("Failed to reconnect");
    }
  }
  vTaskDelete(NULL);
}

void setup()
{
  // Initialize Serial communication to display information on Serial Monitor
  Serial.begin(115200);

  // Initialize I2C with custom SDA = 11, SCL = 12
  // Wire.begin(11, 12);  // SDA = 11, SCL = 12

  // Initialize DHT20 sensor
  // if (!dht.begin()) {
  //   Serial.println("Failed to initialize DHT20 sensor!");
  //   while (1);  // Halt if sensor initialization fails
  // }
  // Serial.println("DHT20 sensor initialized.");

  xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 1, NULL);
  xTaskCreate(SensorTask, "SensorTask", 4096, NULL, 1, NULL);
  xTaskCreate(SeverTask, "SeverTask", 4096, NULL, 1, NULL);
}

void loop()
{
  // if (!tb.connected())
  // {
  //   Serial.println("Reconnecting to ThingsBoard...");
  //   if (!tb.connect(thingBoard_Sever, token_id, port))
  //   {
  //     Serial.println("Failed to reconnect");
  //     return;
  //   }
  // }
  // tb.sendTelemetryData("temperature", temperature);
  // tb.sendTelemetryData("humidity", humidity);

  // tb.loop();
}
