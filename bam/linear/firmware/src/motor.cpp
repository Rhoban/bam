#include "motor.h"
#include "config.h"
#include "shell.h"
#include <Arduino.h>

#include <Arduino.h>

void motor_init() {
  ledcSetup(0, 20000, 10);
  ledcAttachPin(MOTOR_PIN1, 0);

  ledcSetup(1, 20000, 10);
  ledcAttachPin(MOTOR_PIN2, 1);
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
}

void motor_set_pwm(int pwm) {
  if (pwm > 0) {
    ledcWrite(0, pwm);
    ledcWrite(1, 0);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, -pwm);
  }
}


SHELL_COMMAND(pwm, "Set motor PWMs") {
  if (argc != 1) {
    shell_stream()->println("Usage: pwm <pwm>");
    return;
  }
  motor_set_pwm(atoi(argv[0]));
}

SHELL_COMMAND(relay, "Set relay") {
  if (argc != 1) {
    shell_stream()->println("Usage: relay <0 | 1>");
    return;
  }
  if (atoi(argv[0])){
    digitalWrite(RELAY_PIN,HIGH);
  }else{
    digitalWrite(RELAY_PIN,LOW);
  }
}