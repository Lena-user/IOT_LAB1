#include <SPI.h>
#include <MFRC522.h>

// Define RFID pins
#define RFID_RST 17 // Chân RST của module RFID
#define RFID_SS 5   // Chân SDA (SS) của module RFID
#define RFID_SCK 6  // Chân SCK của module RFID
#define RFID_MOSI 7 // Chân MOSI của module RFID  
#define RFID_MISO 8 // Chân MISO của module RFID

MFRC522 rfid(RFID_SS, RFID_RST);

void setup()
{
  Serial.begin(9600);
  delay(1000);                          // Chờ 1 giây để Serial Monitor khởi động
  Serial.println("Serial is working!"); // Dòng kiểm tra
  SPI.begin();                          // Khởi tạo giao tiếp SPI
  rfid.PCD_Init();                      // Khởi tạo module RFID
  Serial.println("RFID module is ready. Please scan a card...");
  delay(1000); // Chờ thêm 1 giây để đảm bảo thông báo được hiển thị
}

void loop()
{
  // Đọc thanh ghi VersionReg để kiểm tra kết nối SPI
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);

  // Kiểm tra giá trị trả về từ VersionReg
  if (version == 0x92)
  {
    Serial.println("SPI is working. Chip is MFRC522.");
  }
  else if (version == 0x00 || version == 0xFF)
  {
    Serial.println("SPI communication failed or no RFID chip detected.");
  }
  else
  {
    Serial.print("Unknown chip detected. Version: 0x");
    Serial.println(version, HEX);
  }

  delay(1000); // Chờ 1 giây trước khi kiểm tra lại
}