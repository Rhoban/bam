#include "motor.h"
#include "config.h"
#include "shell.h"
#include <Arduino.h>

void motor_init() {
  ledcSetup(0, 200, 10);
  ledcAttachPin(MOTOR_PIN1, 0);

  ledcSetup(1, 200, 10);
  ledcAttachPin(MOTOR_PIN2, 1);
}

void motor_set_pwm(int pwm) {
  if (pwm > 0) {
    ledcWrite(0, 1023);
    ledcWrite(1, 1023-pwm);
  } else {
    ledcWrite(0, 1023+pwm);
    ledcWrite(1, 1023);
  }
}

SHELL_COMMAND(pwm, "Set motor PWMs") {
  if (argc != 1) {
    shell_stream()->println("Usage: pwm <pwm>");
    return;
  }

  motor_set_pwm(atoi(argv[0]));
}