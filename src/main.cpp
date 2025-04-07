#include "main.h"



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
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("Failed to read from DHT sensor!");
    }
    else
    {
      Serial.print("Temp: ");
      Serial.print(temperature);
      Serial.print(" °C ");
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      if(tb.connected())
      {
        tb.sendTelemetryData("temperature", temperature);
        tb.sendTelemetryData("humidity", humidity);
      }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    tb.loop();
  }
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
  if (!subscribed) {
    Serial.println("Subscribing for RPC...");
    const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
      RPC_Callback{ RPC_SWITCH_METHOD, processSwitchChange }
    };
    // Perform a subscription. All consequent data processing will happen in
    // processTemperatureChange() and processSwitchChange() functions,
    // as denoted by callbacks array.
    if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }
  vTaskDelete(NULL);
}

// void setup()
// {
//   Serial.begin(9600);
//   while (!Serial);
//   pinMode(LED_PIN, OUTPUT);
//   pinMode(BUTTON_PIN, INPUT);

//   xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 1, NULL);
//   xTaskCreate(SensorTask, "SensorTask", 4096, NULL, 1, NULL);
//   xTaskCreate(SeverTask, "SeverTask", 4096, NULL, 1, NULL);
// }

void setup()
{
    Serial.begin(9600);
    SPI.begin();
    rfid.PCD_Init();

    for (byte i = 0; i < 6; i++)
    {
        key.keyByte[i] = 0xFF;
    }

    Serial.println(F("This code scan the MIFARE Classsic NUID."));
    Serial.print(F("Using the following key:"));
    printHex(key.keyByte, MFRC522::MF_KEY_SIZE);
}

void loop()
{
    if (!rfid.PICC_IsNewCardPresent())
    {
        return;
    }

    if (!rfid.PICC_ReadCardSerial())
    {
        return;
    }

    Serial.print(F("PICC type: "));
    MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
    Serial.println(rfid.PICC_GetTypeName(piccType));

    if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&
        piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
        piccType != MFRC522::PICC_TYPE_MIFARE_4K)
    {
        Serial.println(F("Your tag is not of type MIFARE Classic."));
        return;
    }

    if (rfid.uid.uidByte[0] != nuidPICC[0] ||
        rfid.uid.uidByte[1] != nuidPICC[1] ||
        rfid.uid.uidByte[2] != nuidPICC[2] ||
        rfid.uid.uidByte[3] != nuidPICC[3])
    {
        Serial.println(F("A new card has been detected."));

        for (byte i = 0; i < 4; i++)
        {
            nuidPICC[i] = rfid.uid.uidByte[i];
        }

        Serial.println(F("The NUID tag is:"));
        Serial.print(F("In hex: "));
        printHex(rfid.uid.uidByte, rfid.uid.size);
        Serial.println();
        Serial.print(F("In dec: "));
        printDec(rfid.uid.uidByte, rfid.uid.size);
        Serial.println();
    }
    else
        Serial.println(F("Card read previously."));

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
}

void printHex(byte *buffer, byte bufferSize)
{
    for (byte i = 0; i < bufferSize; i++)
    {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}

void printDec(byte *buffer, byte bufferSize)
{
    for (byte i = 0; i < bufferSize; i++)
    {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], DEC);
    }
}