/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

 Based on the code by Randy Mackay. DIYDrones.com

 This code is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code.  If not, see <http://www.gnu.org/licenses/>.

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#include <SPI.h>

SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3); // 2 MHz, mode 3

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17

static const uint8_t RESET_PIN = 9;
static const uint8_t SS_PIN = SS; // Pin 10

static int32_t x, y;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to open

  SPI.begin();

  // Set SS and reset pin as output
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  reset();

  uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
  if (id == ADNS3080_PRODUCT_ID_VALUE)
    Serial.println(F("ADNS-3080 found"));
  else {
    Serial.print(F("Could not find ADNS-3080: "));
    Serial.println(id, HEX);
    while (1);
  }
}

void loop() {
#if 1
  updateSensor();
#else
  Serial.println(F("image data --------------"));
  printPixelData();
  Serial.println(F("-------------------------"));
  delay(1000);
#endif
}

// TODO: Implement configuration: https://github.com/diydrones/ardupilot/blob/5ddbcc296dd6dd9ac9ed6316ac3134c736ae8a78/libraries/AP_OpticalFlow/AP_OpticalFlow_ADNS3080.cpp#L68-L98
// Also implement "update_conversion_factors"

void printPixelData(void) {
  bool isFirstPixel = true;

  // Write to frame capture register to force capture of frame
  spiWrite(ADNS3080_FRAME_CAPTURE, 0x83);

  // Wait 3 frame periods + 10 nanoseconds for frame to be captured
  delayMicroseconds(1510);  // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

  // Display the pixel data
  for (uint8_t i = 0; i < ADNS3080_PIXELS_Y; i++) {
    for (uint8_t j = 0; j < ADNS3080_PIXELS_X; j++) {
      uint8_t regValue = spiRead(ADNS3080_FRAME_CAPTURE);
      if (isFirstPixel && !(regValue & 0x40))
        Serial.println(F("Failed to find first pixel"));
      isFirstPixel = false;
      uint8_t pixelValue = regValue << 2;
      Serial.print(pixelValue);
      if (j != ADNS3080_PIXELS_X - 1)
        Serial.write(',');
      delayMicroseconds(50);
    }
    Serial.println();
  }

  // Hardware reset to restore sensor to normal operation
  reset();
}

void updateSensor(void) {
  // Read sensor
  uint8_t buf[4];
  spiRead(ADNS3080_MOTION_BURST, buf, 4);
  uint8_t motion = buf[0];
  //Serial.print(motion & 0x01); // Resolution

  if (motion & 0x10) // Check if we've had an overflow
    Serial.println(F("ADNS-3080 overflow\n"));
  else if (motion & 0x80) {
    int8_t dx = buf[1];
    int8_t dy = buf[2];
    uint8_t surfaceQuality = buf[3];

    x += dx;
    y += dy;

    // Print values
    Serial.print(x);
    Serial.write(',');
    Serial.print(dx);
    Serial.write('\t');
    Serial.print(y);
    Serial.write(',');
    Serial.print(dy);
    Serial.write('\t');
    Serial.println(surfaceQuality);
    Serial.flush();
  }
#if 0
  else
    Serial.println(motion, HEX);
#endif

  delay(10);
}

void reset(void) {
  digitalWrite(RESET_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RESET_PIN, LOW);
}

// Will cause the Delta_X, Delta_Y, and internal motion registers to be cleared
void clearMotion() {
  spiWrite(ADNS3080_MOTION_CLEAR, 0xFF); // Writing anything to this register will clear the sensor's motion registers
  x = y = 0;
}

void spiWrite(uint8_t reg, uint8_t data) {
  spiWrite(reg, &data, 1);
}

void spiWrite(uint8_t reg, uint8_t *data, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  delayMicroseconds(50);
  SPI.transfer(data, length);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
}

uint8_t spiRead(uint8_t reg) {
  uint8_t buf;
  spiRead(reg, &buf, 1);
  return buf;
}

void spiRead(uint8_t reg, uint8_t *data, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);
  SPI.transfer(reg);
  delayMicroseconds(50);
  memset(data, 0, length);
  SPI.transfer(data, length);
  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
}

