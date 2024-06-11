#include "encoder.h"
#include "motor.h"
#include "shell.h"
#include <Arduino.h>

void setup() {
  shell_init(115200);
  encoder_init();
  motor_init();
}

void loop() { shell_tick(); }
