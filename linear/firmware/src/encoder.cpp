#include "encoder.h"
#include "config.h"
#include "encoder.h"
#include "shell.h"
#include <Arduino.h>
#include <SPI.h>

void encoder_init() {
  SPI.begin(ENCODER_SCK, ENCODER_DO, MOSI, ENCODER_SS);
  SPI.setBitOrder(MSBFIRST);
  SPI.setFrequency(1000000);
  SPI.setDataMode(SPI_MODE1);
  pinMode(ENCODER_SS, OUTPUT);
  digitalWrite(ENCODER_SCK, HIGH);
}

static void encoder_clock() {
  delayMicroseconds(1);
  digitalWrite(ENCODER_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(ENCODER_SCK, LOW);
}

uint16_t encoder_read() {
  digitalWrite(SS, LOW);
  delayMicroseconds(1);
  uint16_t value = SPI.transfer16(0);
  digitalWrite(SS, HIGH);
  uint16_t angle = (value >> 6) & 0x3FF;

  return angle;
}

SHELL_COMMAND(mag, "Read the magnetic encoder") {
  while (!shell_stream()->available()) {
    shell_stream()->printf("Value: %d\r\n", encoder_read());
    vTaskDelay(10);
  }
}