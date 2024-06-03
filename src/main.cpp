#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>

#define NET_PIN 0

#define O1_PIN 25
#define O2_PIN 14
#define O3_PIN 12
#define O4_PIN 13

struct {
  uint32_t output : 1;
  struct {
    uint32_t D1 : 1;
    uint32_t D2 : 1;
    uint32_t D3 : 1;
    uint32_t A1 : 1;
    uint32_t A2 : 1;
  } input;
  uint32_t rs485 : 1;
  uint32_t wifi : 1;
  uint32_t display : 1;
  uint32_t i2c_and_rtc : 1;
} qc;

void TestOutput() {
  xTaskCreate([](void*) {
    pinMode(NET_PIN, OUTPUT);
    pinMode(O1_PIN, OUTPUT);
    pinMode(O2_PIN, OUTPUT);
    pinMode(O3_PIN, OUTPUT);
    pinMode(O4_PIN, OUTPUT);

    while(1) {
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O1_PIN, HIGH);
      digitalWrite(O2_PIN, HIGH);
      digitalWrite(O3_PIN, HIGH);
      digitalWrite(O4_PIN, HIGH);
      delay(500);
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O1_PIN, LOW);
      digitalWrite(O2_PIN, LOW);
      digitalWrite(O3_PIN, LOW);
      digitalWrite(O4_PIN, LOW);
      delay(500);
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O1_PIN, HIGH);
      delay(500);
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O2_PIN, HIGH);
      delay(500);
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O3_PIN, HIGH);
      delay(500);
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O4_PIN, HIGH);
      delay(500);
      digitalWrite(NET_PIN, !digitalRead(NET_PIN));
      digitalWrite(O1_PIN, LOW);
      digitalWrite(O2_PIN, LOW);
      digitalWrite(O3_PIN, LOW);
      digitalWrite(O4_PIN, LOW);
      delay(500);
    }

    vTaskDelete(NULL);
  }, "TestOutput", 5 * 1024, NULL, 10, NULL);
}

#define D1_PIN 34
#define D2_PIN 35
#define D3_PIN 26
#define A1_PIN 36
#define A2_PIN 39

void TestInput() {
  xTaskCreate([](void*) {
    pinMode(D1_PIN, INPUT);
    pinMode(D2_PIN, INPUT);
    pinMode(D3_PIN, INPUT);
    pinMode(A1_PIN, INPUT);
    pinMode(A2_PIN, INPUT);

    while(1) {
      /*Serial.printf("%3d %3d %3d %6.02f %6.02f\n",
        digitalRead(D1_PIN), digitalRead(D2_PIN), digitalRead(D3_PIN),
        analogRead(A1_PIN) / 4095.0 * 9.9, analogRead(A2_PIN) / 4095.0 * 9.9
      );*/
      if (digitalRead(D1_PIN) == 0) {
        qc.input.D1 = 1;
      }
      if (digitalRead(D2_PIN) == 0) {
        qc.input.D2 = 1;
      }
      if (digitalRead(D3_PIN) == 0) {
        qc.input.D3 = 1;
      }
      if (analogRead(A1_PIN) == 0) {
        qc.input.A1 = 1;
      }
      if (analogRead(A2_PIN) == 0) {
        qc.input.A2 = 1;
      }
      delay(100);
    }

    vTaskDelete(NULL);
  }, "TestInput", 5 * 1024, NULL, 11, NULL);
}

#define DE_RE_PIN 2
#define MODE_SEND HIGH
#define MODE_RECV LOW

float temp = 0, humi = 0;

uint16_t CRC16(uint8_t *buf, int len) {  
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];    // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                           // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }

  return crc;
}

void readXY_MD02() {
  uint8_t buff[] = {
    0x02, // Devices Address
    0x04, // Function code
    0x00, // Start Address HIGH
    0x01, // Start Address LOW
    0x00, // Quantity HIGH
    0x02, // Quantity LOW
    0, // CRC LOW
    0  // CRC HIGH
  };

  uint16_t crc = CRC16(&buff[0], 6);
  buff[6] = crc & 0xFF;
  buff[7] = (crc >> 8) & 0xFF;

  digitalWrite(DE_RE_PIN, MODE_SEND);
  //delay(1);
  Serial2.write(buff, sizeof(buff));
  Serial2.flush(); // wait MODE_SEND completed
  //delay(1);
  digitalWrite(DE_RE_PIN, MODE_RECV);

  delay(100);
  
  if (Serial2.find("\x02\x04")) {
    uint8_t n = Serial2.read();
    if (n != 4) {
      // Serial.printf("Error data size : %d\n", n);
      return;
    }

    temp = ((uint16_t)(Serial2.read() << 8) | Serial2.read()) / 10.0;
    humi = ((uint16_t)(Serial2.read() << 8) | Serial2.read()) / 10.0;
    qc.rs485 = 1;
  } else {
    // Serial.println("ERROR Timeout");
    return;
  }
}

void TestModbus() {
  xTaskCreate([](void*) {
    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, MODE_RECV);

    Serial2.begin(9600, SERIAL_8N1, 4, 15); // Rx, Tx
    Serial2.setTimeout(200);

    while(1) {
      readXY_MD02();
      // Serial.printf("XY-MD02: %.01f *C\t%.01f %%RH\n", temp, humi);
      delay(2000);
    }

    vTaskDelete(NULL);
  }, "TestModbus", 5 * 1024, NULL, 10, NULL);
}

void TestWiFi() {
  xTaskCreate([](void*) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    while(1) {
      //Serial.println("Scan start");
      int n = WiFi.scanNetworks();
      if (n == 0) {
          //Serial.println("no networks found");
      } else {
        qc.wifi = 1;
          //Serial.print(n);
          //Serial.println(" networks found");
      }
      delay(2000);
    }

    vTaskDelete(NULL);
  }, "TestWiFi", 5 * 1024, NULL, 10, NULL);
}

TFT_eSPI tft = TFT_eSPI();
#define LCD_BL_PIN 32

void TestDisplay() {
  xTaskCreate([](void*) {
    tft.init();
    pinMode(LCD_BL_PIN, OUTPUT);
    digitalWrite(LCD_BL_PIN, LOW);

    while(1) {
      qc.display = 1;
      tft.fillScreen(TFT_BLACK);
      delay(500);
      tft.fillScreen(TFT_RED);
      delay(500);
      tft.fillScreen(TFT_GREEN);
      delay(500);
      tft.fillScreen(TFT_BLUE);
      delay(500);
      tft.fillScreen(TFT_WHITE);
      delay(500);
    }

    vTaskDelete(NULL);
  }, "TestDisplay", 5 * 1024, NULL, 10, NULL);
}


#include <RTC.h>
#include <Wire.h>


void TestI2CAndRTC() {
  xTaskCreate([](void*) {
    Wire.begin();
    while (!RTC_init()) {
      //Serial.println("Init RTC fail !!!");
      delay(500);
    }
    
    struct tm timeinfo_write = {
      .tm_sec = 0,
      .tm_min = 38,
      .tm_hour = 16,
      .tm_mday = 10,
      .tm_mon = 11,
      .tm_year = 2020,
    };

    while (!RTC_write(&timeinfo_write)) {
      //Serial.println("RTC write fail !!!");
      delay(500);
    }
    //Serial.println("RTC writed.");

    while(1) {
      struct tm timeinfo_read = { 0 };
      if (!RTC_read(&timeinfo_read)) {
        ///Serial.println("RTC read fail !!!");
        delay(500);
        continue;
      }
      /*Serial.printf("RTC: %d/%d/%d %02d:%02d:%02d\n",
        timeinfo_read.tm_mday, timeinfo_read.tm_mon, timeinfo_read.tm_year + 1900,
        timeinfo_read.tm_hour, timeinfo_read.tm_min, timeinfo_read.tm_sec);
      */
     qc.i2c_and_rtc = 1;
     delay(1000);
    }

    vTaskDelete(NULL);
  }, "TestI2CAndRTC", 5 * 1024, NULL, 10, NULL);
}

void setup() {
  Serial.begin(115200);

  memset(&qc, 0, sizeof(qc));
  TestOutput();
  TestInput();
  TestModbus();
  TestI2CAndRTC();
  TestDisplay();
  TestWiFi();
}

void loop() {
  Serial.printf(
    "D1: %d\tD2: %d\tD3: %d\tA1: %d\tA2: %d\tRS485: %d\tWiFi: %d\tRTC-I2C: %d\n",
    qc.input.D1, qc.input.D2, qc.input.D3, qc.input.A1, qc.input.A2, qc.rs485, qc.wifi, qc.i2c_and_rtc
  );
  delay(500);
}

