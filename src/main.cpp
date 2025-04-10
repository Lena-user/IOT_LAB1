#include <Adafruit_PN532.h>
#include <Arduino.h>
#include <Wire.h>

// Chân I2C mặc định
#define SDA_PIN 11
#define SCL_PIN 12

Adafruit_PN532 nfc(PN532_I2C_ADDRESS);

void setup() {
  Serial.begin(115200); // Khởi tạo Serial với tốc độ 115200 bps
  while (!Serial) {
    // Chờ Serial khởi tạo
  } 
  Serial.println("Initializing I2C...");
  // Khởi tạo I2C với các chân mặc định
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C initialized successfully.");
}

void loop() {
  // Kiểm tra xem I2C có khởi tạo thành công không
  if (!nfc.begin()) {
    Serial.println("ERROR: Didn't find PN532 board. Check connections.");
    return;
  }
  else
    Serial.println("Found chip PN532!");
  // Đặt chế độ SAM (Secure Access Module) để đọc thẻ NFC 
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 }; // UID của thẻ
  uint8_t uidLength;

  // Kiểm tra xem có thẻ NFC nào trong phạm vi đọc không
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  Serial.println(success);
  if (success) {
    Serial.println("Found an NFC card!");

    // In UID của thẻ
    Serial.print("UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
    Serial.print("UID Value: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      Serial.print(" 0x"); Serial.print(uid[i], HEX);
    }
    Serial.println();
    delay(1000); // Chờ 1 giây trước khi kiểm tra lại
  } else {
    Serial.println("No NFC card found.");
    Serial.println("Waiting for an NFC card...");
    // Không tìm thấy thẻ
    delay(500); // Chờ 0.5 giây trước khi kiểm tra lại
  }
}