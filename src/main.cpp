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

bool enableReadTask = false; // Biến toàn cục để kiểm soát việc đọc thẻ NFC

#define SDA_PIN 11
#define SCL_PIN 12

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Giảm tốc độ I2C xuống 100kHz
  pinMode(BUTTON_PIN, INPUT);

  Serial.println("Initializing PN532...");
  nfc.begin();
  nfc.SAMConfig();

  xTaskCreate(wifiTask, "WiFiTask", 4096, NULL, 1, NULL);
  xTaskCreate(SeverTask, "SeverTask", 4096, NULL, 1, NULL);
}

void loop()
{
  if (enableReadTask)
  {
    Serial.println("Reading NFC tags...");
    // Tạo một task mới để đọc thẻ NFC
  }
  else
  {
    Serial.println("Waiting for NFC tag...");
  }
  tb.loop();
  delay(1000);
}

bool writeToNFCTag(const String &textToWrite)
{
  if (textToWrite.length() > 16)
  {
    Serial.println("Error: Input text is too long. Maximum 16 characters allowed.");
    return false;
  }

  uint8_t dataToWrite[16] = {' '}; // Khởi tạo với khoảng trắng
  textToWrite.getBytes(dataToWrite, 16);

  uint8_t uid[7];
  uint8_t uidLength;

  // Tìm thẻ NFC
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
  {
    uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA))
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

      // Kiểm tra nếu UID đã tồn tại trong vector
      auto it = std::find_if(registeredTags.begin(), registeredTags.end(),
                             [&uidString](const std::pair<std::string, std::string> &tag)
                             {
                               return tag.first == uidString.c_str();
                             });

      if (it == registeredTags.end())
      {
        // Ghi dữ liệu vào thẻ NFC
        if (nfc.mifareclassic_WriteDataBlock(4, dataToWrite))
        {
          // Thêm UID và nội dung vào vector
          registeredTags.emplace_back(uidString.c_str(), textToWrite.c_str());
          Serial.print("New UID added: ");
          Serial.println(uidString);
          return true;
        }
        else
        {
          Serial.println("Failed to write to the NFC tag.");
          return false;
        }
      }
      else
      {
        Serial.print("UID already exists: ");
        Serial.println(uidString);
        return false;
      }
    }
    else
    {
      Serial.println("Failed to authenticate block 4.");
      return false;
    }
  }
  else
  {
    Serial.println("No NFC tag detected.");
    return false;
  }
}

void readAndUnlockDoorTask()
{
  while (true)
  {
    uint8_t uid[7];
    uint8_t uidLength;

    // Tìm thẻ NFC
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
    {
      uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA))
      {
        uint8_t dataRead[16];
        if (nfc.mifareclassic_ReadDataBlock(4, dataRead))
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

          // Chuyển dữ liệu đọc được thành chuỗi
          String roomData = "";
          for (int i = 0; i < 16; i++)
          {
            if (dataRead[i] != ' ')
            { // Bỏ qua khoảng trắng
              roomData += (char)dataRead[i];
            }
          }

          Serial.print("Scanned UID: ");
          Serial.println(uidString);
          Serial.print("Room Data: ");
          Serial.println(roomData);

          // Kiểm tra nếu UID và nội dung khớp với danh sách đã đăng ký
          auto it = std::find_if(registeredTags.begin(), registeredTags.end(),
                                 [&uidString, &roomData](const std::pair<std::string, std::string> &tag)
                                 {
                                   return tag.first == uidString.c_str() && tag.second == roomData.c_str();
                                 });

          if (it != registeredTags.end())
          {
            // Mở khóa cửa
            Serial.println("Access granted. Door unlocked!");
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Giả lập cửa mở trong 5 giây

            // Đóng cửa
            Serial.println("Door locked.");
          }
          else
          {
            Serial.println("Access denied. Invalid tag.");
          }
        }
        else
        {
          Serial.println("Failed to read from the NFC tag.");
        }
      }
      else
      {
        Serial.println("Failed to authenticate block 4.");
      }
    }
    else
    {
      Serial.println("No NFC tag detected.");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Chờ 1 giây trước khi quét lại
  }
}

std::vector<std::pair<std::string, std::string>> readFromNFCTag()
{
  std::vector<std::pair<std::string, std::string>> tagData; // Vector để lưu UID và nội dung

  uint8_t uid[7];
  uint8_t uidLength;

  // Tìm thẻ NFC
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
  {
    uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA))
    {
      uint8_t dataRead[16];
      if (nfc.mifareclassic_ReadDataBlock(4, dataRead))
      {
        // Chuyển UID thành chuỗi
        std::string uidString = "";
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

        // Chuyển dữ liệu đọc được thành chuỗi
        std::string content = "";
        for (int i = 0; i < 16; i++)
        {
          if (dataRead[i] != ' ')
          { // Bỏ qua khoảng trắng
            content += static_cast<char>(dataRead[i]);
          }
        }

        // Thêm UID và nội dung vào vector
        tagData.emplace_back(uidString, content);

        Serial.print("Scanned UID: ");
        Serial.println(uidString.c_str());
        Serial.print("Content: ");
        Serial.println(content.c_str());
      }
      else
      {
        Serial.println("Failed to read from the NFC tag.");
      }
    }
    else
    {
      Serial.println("Failed to authenticate block 4.");
    }
  }
  else
  {
    Serial.println("No NFC tag detected.");
  }

  return tagData; // Trả về vector chứa UID và nội dung
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
  vTaskDelete(NULL);
}

bool isUIDRegistered(const std::vector<std::pair<std::string, std::string>> &tags)
{
  // Kiểm tra nếu vector đầu vào rỗng
  if (tags.empty())
  {
    Serial.println("No tags provided for checking.");
    return false;
  }

  // Lấy UID từ vector đầu vào (giả sử chỉ kiểm tra thẻ đầu tiên)
  const std::string &uidString = tags[0].first;

  // Kiểm tra nếu UID đã tồn tại trong danh sách registeredTags
  auto it = std::find_if(registeredTags.begin(), registeredTags.end(),
                         [&uidString](const std::pair<std::string, std::string> &tag)
                         {
                           return tag.first == uidString;
                         });

  if (it != registeredTags.end())
  {
    Serial.print("UID is registered: ");
    Serial.println(uidString.c_str());
    return true;
  }
  else
  {
    Serial.print("UID is not registered: ");
    Serial.println(uidString.c_str());
    return false;
  }
}

void processCheckInOut(const JsonVariantConst &data, JsonDocument &response)
{
  enableReadTask = false; // Tắt chế độ đọc thẻ NFC khi nhận yêu cầu RPC
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

      // Đọc dữ liệu từ thẻ NFC
      std::vector<std::pair<std::string, std::string>> tags = readFromNFCTag();

      if (tags.empty())
      {
        response["status"] = "error";
        response["message"] = "No NFC tag detected.";
        Serial.println("No NFC tag detected.");
        return;
      }

      // Kiểm tra nếu UID đã tồn tại
      if (isUIDRegistered(tags))
      {
        response["status"] = "error";
        response["message"] = "UID is already registered.";
        Serial.println("UID is already registered.");
      }
      else
      {
        // Ghi số phòng vào thẻ NFC
        if (writeToNFCTag(roomString))
        {
          registeredTags.emplace_back(tags[0].first, roomString.c_str());
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
}