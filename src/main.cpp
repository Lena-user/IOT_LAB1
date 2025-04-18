/**************************************************************************/
/*!
    This example will wait for any ISO14443A card or tag, and
    depending on the size of the UID will attempt to read from it.

    If the card has a 4-byte UID it is probably a Mifare
    Classic card, and the following steps are taken:

    - Authenticate block 4 (the first block of Sector 1) using
      the default KEYA of 0XFF 0XFF 0XFF 0XFF 0XFF 0XFF
    - If authentication succeeds, we can then read any of the
      4 blocks in that sector (though only block 4 is read here)

    If the card has a 7-byte UID it is probably a Mifare
    Ultralight card, and the 4 byte pages can be read directly.
    Page 4 is read by default since this is the first 'general-
    purpose' page on the tags.

    To enable debug message, define DEBUG in PN532/PN532_debug.h
*/
/**************************************************************************/
#include <Arduino.h>
#define NFC_INTERFACE_I2C
#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532_I2C.cpp>
#include <PN532.h>

#include "main.h"

#define DEBOUNCE_DELAY 50 // Thời gian chống rung (ms)
std::vector<std::pair<std::string, std::string>> registeredTags;
std::vector<std::string> statusList = {
    "Admin",
    "status_Room_1",
    "status_Room_2",
    "status_Room_3",
    "status_Room_4"};

bool enableReadTask = true; // Biến toàn cục để kiểm soát việc đọc thẻ NFC

TaskHandle_t readTaskHandle = NULL; // Handle cho task đọc thẻ NFC
TaskHandle_t wifiTaskHandle = NULL;  // Handle cho task WiFi
TaskHandle_t serverTaskHandle = NULL; // Handle cho task server

#define SDA_PIN 11
#define SCL_PIN 12

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Giảm tốc độ I2C xuống 100kHz
  Wire.setClock(100000); // Thiết lập tốc độ I2C là 100kHz
  pinMode(BUTTON_PIN, INPUT);

  Serial.println("Initializing PN532...");
  nfc.begin();
  nfc.SAMConfig();

  xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 3, &wifiTaskHandle);
  xTaskCreate(SeverTask, "SeverTask", 4096, NULL, 2, &serverTaskHandle);
  xTaskCreate(readAndUnlockDoorTask, "ReadAndUnlockDoorTask", 4096, NULL, 1, &readTaskHandle);

  vTaskSuspend(serverTaskHandle); // Tạm dừng task SeverTask cho đến khi kết nối WiFi thành công
  vTaskSuspend(readTaskHandle); // Tạm dừng task đọc thẻ NFC cho đến khi kết nối WiFi thành công
}

void loop()
{
  tb.loop();
}

void readAndUnlockDoorTask(void* pvParameters) 
{
  while (1) {
      uint8_t uid[7];
      uint8_t uidLength;

      Serial.println("Waiting for NFC tag...");

      // Kiểm tra xem có thẻ NFC được phát hiện không
      if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
          uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

          if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA)) {
              uint8_t dataRead[16];

              if (nfc.mifareclassic_ReadDataBlock(4, dataRead)) {
                  // Chuyển UID thành chuỗi
                  String uidString = "";
                  for (int i = 0; i < uidLength; i++) {
                      char buffer[3];
                      sprintf(buffer, "%02X", uid[i]);
                      uidString += buffer;
                      if (i < uidLength - 1) uidString += ":";
                  }

                  // Chuyển dữ liệu thành chuỗi
                  String roomData = "";
                  for (int i = 0; i < 16; i++) {
                      if (dataRead[i] != ' ') roomData += (char)dataRead[i];
                  }

                  Serial.print("Scanned UID: ");
                  Serial.println(uidString);
                  Serial.print("Room Data: ");
                  Serial.println(roomData);

                  // Kiểm tra nếu thẻ hợp lệ
                  auto it = std::find_if(registeredTags.begin(), registeredTags.end(),
                      [&uidString, &roomData](const std::pair<std::string, std::string>& tag) {
                          return tag.first == std::string(uidString.c_str()) && tag.second == std::string(roomData.c_str());
                      });

                  if (it != registeredTags.end()) {
                      // Mở khóa cửa
                      Serial.println("Access granted. Door unlocked!");
                      vTaskDelay(5000 / portTICK_PERIOD_MS); // Giữ cửa mở trong 5 giây
                      Serial.println("Door locked.");
                  } else {
                      Serial.println("Access denied. Invalid tag.");
                  }
              } else {
                  Serial.println("Failed to read from the NFC tag.");
              }
          } else {
              Serial.println("Failed to authenticate block 4.");
          }
      } else {
          Serial.println("No NFC tag detected.");
      }

      vTaskDelay(5000 / portTICK_PERIOD_MS); // Chờ 1 giây trước khi quét lại
  }
}

std::vector<std::pair<std::string, std::string>> readFromNFCTag() {
  std::vector<std::pair<std::string, std::string>> tagData; 

  uint8_t uid[7];
  uint8_t uidLength;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
      Serial.println("No NFC tag detected.");
      return tagData;
  }

  uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA)) {
      Serial.println("Failed to authenticate block 4.");
      return tagData;
  }

  uint8_t dataRead[16];
  if (!nfc.mifareclassic_ReadDataBlock(4, dataRead)) {
      Serial.println("Failed to read from the NFC tag.");
      return tagData;
  }

  // Chuyển UID thành chuỗi Hex
  std::ostringstream uidStream;
  for (int i = 0; i < uidLength; i++) {
      uidStream << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(uid[i]);
      if (i < uidLength - 1) uidStream << ":";
  }
  std::string uidString = uidStream.str();

  // Chuyển nội dung thành chuỗi, bỏ qua khoảng trắng
  std::string content(dataRead, dataRead + 16);
  content.erase(std::remove(content.begin(), content.end(), ' '), content.end());

  tagData.emplace_back(uidString, content);

  Serial.print("Scanned UID: ");
  Serial.println(uidString.c_str());
  Serial.print("Content: ");
  Serial.println(content.c_str());

  return tagData;
}

void displayMenu()
{
  Serial.println("\nMenu:");
  Serial.println("1. Write to NFC tag");
  Serial.println("2. Read from NFC tag");
  Serial.println("3. View all registered tags");
  Serial.println("4. Delete a tag");
  Serial.println("Enter your choice:\n");
}

void printRegisteredTags()
{
  if (registeredTags.empty())
  {
    Serial.println("No registered tags found.");
    return;
  }

  Serial.println("Registered Tags:");
  for (const auto &tag : registeredTags)
  {
    Serial.print("UID: ");
    Serial.print(tag.first.c_str());
    Serial.print(" | Content: ");
    Serial.println(tag.second.c_str());
  }
}

void deleteTag()
{
  if (registeredTags.empty())
  {
    Serial.println("No registered tags to delete.");
    return;
  }

  Serial.println("Please scan the NFC tag you want to delete...");

  uint8_t uid[7];
  uint8_t uidLength;

  // Tìm thẻ NFC
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
  {
    // Chuyển UID thành chuỗi
    String uidString = "";
    for (int i = 0; i < uidLength; i++)
    {
      char buffer[3];                  // 2 ký tự HEX + null terminator
      sprintf(buffer, "%02X", uid[i]); // Định dạng thành 2 ký tự HEX
      uidString += buffer;
      if (i < uidLength - 1)
      {
        uidString += ":"; // Thêm dấu ":" giữa các byte
      }
    }

    // Tìm UID trong danh sách
    auto it = std::find_if(registeredTags.begin(), registeredTags.end(),
                           [&uidString](const std::pair<std::string, std::string> &tag)
                           {
                             return tag.first == uidString.c_str();
                           });

    if (it != registeredTags.end())
    {
      // Xóa UID khỏi danh sách
      registeredTags.erase(it);
      Serial.print("UID ");
      Serial.print(uidString);
      Serial.println(" has been deleted.");

      // Ghi dữ liệu rỗng vào thẻ NFC
      uint8_t emptyData[16] = {0}; // Dữ liệu rỗng (16 byte toàn 0)
      uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA))
      {
        if (nfc.mifareclassic_WriteDataBlock(4, emptyData))
        {
          Serial.println("The NFC tag has been cleared.");
        }
        else
        {
          Serial.println("Failed to clear the NFC tag.");
        }
      }
      else
      {
        Serial.println("Failed to authenticate block 4.");
      }
    }
    else
    {
      Serial.print("UID ");
      Serial.print(uidString);
      Serial.println(" not found in the registered list.");
    }
  }
  else
  {
    Serial.println("No NFC tag detected. Please try again.");
  }
}

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
  vTaskResume(serverTaskHandle); // Resume SeverTask sau khi kết nối WiFi thành công
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
  else
  {
    Serial.println("Connected to ThingsBoard");
  }
  if (!subscribed)
  {
    Serial.println("Subscribing for RPC...");
    const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
        RPC_Callback{RPC_REQUEST_METHOD, processCheckInOut}};
    if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
    {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }
  vTaskResume(readTaskHandle); // Resume read task after server task is ready
  vTaskDelete(NULL);
}

bool writeToNFCTag(const String &textToWrite) {
  Serial.println("Starting writeToNFCTag...");

  if (textToWrite.length() > 16) {
      Serial.println("Error: Input text is too long. Maximum 16 characters allowed.");
      return false;
  }

  uint8_t dataToWrite[16];
  memset(dataToWrite, ' ', sizeof(dataToWrite)); // Đảm bảo dữ liệu có khoảng trắng mặc định
  textToWrite.getBytes(dataToWrite, sizeof(dataToWrite));

  uint8_t uid[7];
  uint8_t uidLength;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
      Serial.println("No NFC tag detected.");
      return false;
  }

  std::ostringstream uidStream;
  for (int i = 0; i < uidLength; i++) {
      uidStream << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(uid[i]);
      if (i < uidLength - 1) uidStream << ":";
  }
  std::string uidString = uidStream.str();
  Serial.print("UID: ");
  Serial.println(uidString.c_str());

  std::vector<std::pair<std::string, std::string>> tags = {{uidString, ""}};
  if (isUIDRegistered(tags)) {
      Serial.println("UID is already registered.");
      return false;
  }

  uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA)) {
      Serial.println("Failed to authenticate block 4.");
      return false;
  }

  if (!nfc.mifareclassic_WriteDataBlock(4, dataToWrite)) {
      Serial.println("Failed to write to NFC tag.");
      return false;
  }
  registeredTags.emplace_back(uidString, textToWrite.c_str());
  return true;
}

bool isUIDRegistered(const std::vector<std::pair<std::string, std::string>> &tags) {
  if (tags.empty()) {
      Serial.println("No tags provided for checking.");
      return false;
  }

  const std::string &uidString = tags[0].first;

  // Kiểm tra UID bằng std::any_of() thay vì std::find_if()
  bool isRegistered = std::any_of(registeredTags.begin(), registeredTags.end(),
  [&uidString](const std::pair<std::string, std::string> &tag) {
      return tag.first == uidString;
  });

Serial.print("UID ");
Serial.print(uidString.c_str());
Serial.println(isRegistered ? " is registered." : " is not registered.");

  return isRegistered;
}

void processCheckInOut(const JsonVariantConst &data, JsonDocument &response)
{
  vTaskDelete(readTaskHandle); // Tạm dừng task đọc thẻ NFC để xử lý yêu cầu RPC
  Serial.println("Stopping read task...");
  Serial.println("Processing Check-In/Out request...");
  // Kiểm tra dữ liệu RPC
  if (data.containsKey(RPC_ROOM_KEY) && data.containsKey(RPC_REQUEST_KEY))
  {
    int room = data[RPC_ROOM_KEY];
    const char *requestType = data[RPC_REQUEST_KEY];

    if (strcmp(requestType, "CHECK_IN") == 0)
    {
      Serial.print("Check-In for ROOM: ");
      Serial.println(room);

      String roomString = String(room);

      if (writeToNFCTag(roomString))
      {
        tb.sendAttributeData(statusList.at(room).c_str(), true);
        response["status"] = "success";
        response["message"] = "Check-In completed successfully";
        Serial.println("Check-In completed successfully.");
      }
      else
      {
        response["status"] = "error";
        response["message"] = "Failed to write to NFC tag.";
        Serial.println("Failed to write to NFC tag.");
      }
    }
    else if (strcmp(requestType, "CHECK_OUT") == 0)
    {
      Serial.print("Check-Out for ROOM: ");
      Serial.println(room);

      // Đọc dữ liệu từ thẻ NFC
      std::vector<std::pair<std::string, std::string>> tags = readFromNFCTag();

      if (tags.empty())
      {
        response["status"] = "error";
        response["message"] = "No NFC tag detected.";
        Serial.println("No NFC tag detected.");
        return;
      }

      // Kiểm tra nếu UID tồn tại
      if (isUIDRegistered(tags))
      {
        // Xóa UID khỏi danh sách và thực hiện Check-Out
        auto it = std::find_if(registeredTags.begin(), registeredTags.end(),
                               [&tags](const std::pair<std::string, std::string> &tag)
                               {
                                 return tag.first == tags[0].first;
                               });

        if (it != registeredTags.end())
        {
          registeredTags.erase(it);
        }

        deleteTag();
        tb.sendAttributeData(statusList.at(room).c_str(), false);
        response["status"] = "success";
        response["message"] = "Check-Out completed successfully";
        Serial.println("Check-Out completed successfully.");
      }
      else
      {
        response["status"] = "error";
        response["message"] = "UID not found in the system.";
        Serial.println("UID not found in the system.");
      }
    }
    else
    {
      response["status"] = "error";
      response["message"] = "Invalid Request type.";
      Serial.println("Invalid Request type.");
    }
  }
  else
  {
    response["status"] = "error";
    response["message"] = "ROOM or Request key not found in RPC data.";
    Serial.println("ROOM or Request key not found in RPC data.");
  }
  delay(2000); // Đợi 2 giây trước khi tiếp tục 
  xTaskCreate(readAndUnlockDoorTask, "ReadTask", 4096, NULL, 1, &readTaskHandle);
  Serial.print("Resuming read task...");
}