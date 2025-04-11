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

#define SDA_PIN 11
#define SCL_PIN 12

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing PN532...");
  nfc.begin();
  nfc.SAMConfig();

  Serial.println("Waiting for an ISO14443A Card...");
  Serial.println("Menu:");
  Serial.println("1. Write to NFC tag");
  Serial.println("2. Read from NFC tag");
  Serial.println("Enter your choice:");
}

void loop() {
  // Kiểm tra nếu có dữ liệu nhập từ Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Đọc chuỗi từ Serial Monitor
    input.trim(); // Loại bỏ khoảng trắng hoặc ký tự xuống dòng

    if (input == "1") {
      // Chức năng ghi thẻ
      Serial.println("Enter the text you want to write to the NFC tag (max 16 characters):");
      while (!Serial.available()); // Chờ người dùng nhập dữ liệu
      String textToWrite = Serial.readStringUntil('\n');
      textToWrite.trim();

      if (textToWrite.length() > 16) {
        Serial.println("Error: Input text is too long. Maximum 16 characters allowed.");
        return;
      }

      uint8_t dataToWrite[16] = { ' ' }; // Khởi tạo với khoảng trắng
      textToWrite.getBytes(dataToWrite, 16);

      uint8_t success;
      uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
      uint8_t uidLength;

      // Tìm thẻ NFC
      success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

      if (success) {
        Serial.println("Found an ISO14443A card");
        Serial.print("UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
        Serial.print("UID Value: "); nfc.PrintHex(uid, uidLength); Serial.println("");

        // Xác thực block 4 với KEY A mặc định
        uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA)) {
          Serial.println("Authenticated block 4!");

          // Ghi dữ liệu vào block 4
          if (nfc.mifareclassic_WriteDataBlock(4, dataToWrite)) {
            Serial.println("Data written to block 4!");
          } else {
            Serial.println("Failed to write to block 4.");
          }
        } else {
          Serial.println("Failed to authenticate block 4.");
        }
      } else {
        Serial.println("Didn't find an ISO14443A card.");
      }
    } else if (input == "2") {
      // Chức năng đọc thẻ
      uint8_t success;
      uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
      uint8_t uidLength;

      // Tìm thẻ NFC
      success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

      if (success) {
        Serial.println("Found an ISO14443A card");
        Serial.print("UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
        Serial.print("UID Value: "); nfc.PrintHex(uid, uidLength); Serial.println("");

        // Xác thực block 4 với KEY A mặc định
        uint8_t keyA[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        if (nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA)) {
          Serial.println("Authenticated block 4!");

          // Đọc dữ liệu từ block 4
          uint8_t dataRead[16]; // Buffer để lưu dữ liệu đọc được
          if (nfc.mifareclassic_ReadDataBlock(4, dataRead)) {
            Serial.println("Data read from block 4:");
            nfc.PrintHex(dataRead, 16); // In dữ liệu ra Serial Monitor

            // In dữ liệu dưới dạng chuỗi
            Serial.print("Data as string: ");
            for (int i = 0; i < 16; i++) {
              Serial.print((char)dataRead[i]); // Ép kiểu từng byte thành ký tự
            }
            Serial.println();
          } else {
            Serial.println("Failed to read from block 4.");
          }
        } else {
          Serial.println("Failed to authenticate block 4.");
        }
      } else {
        Serial.println("Didn't find an ISO14443A card.");
      }
    } else {
      Serial.println("Invalid choice. Please enter 1 or 2.");
    }

    // Hiển thị lại menu
    Serial.println("Menu:");
    Serial.println("1. Write to NFC tag");
    Serial.println("2. Read from NFC tag");
    Serial.println("Enter your choice:");
  }
}