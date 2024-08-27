#include "servo.h"
#include "motor.h"
#include "encoder.h"
#include "config.h"
#include "encoder.h"
#include "shell.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void setup() {
  shell_init(921600);
  encoder_init();
  motor_init();
}

void loop() { 
  shell_tick();
  encoder_tick();
  servo_tick();
}
