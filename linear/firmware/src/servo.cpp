#include "servo.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "shell.h"

// Servoing enabled
static bool enabled = false;

// Target length [m]
static float target = 0.0;

static unsigned long last_update = 0;

float angle_wrap(float angle) { return fmod(angle + M_PI, 2 * M_PI) - M_PI; }

float servo_l() {
  // Reading alpha
  uint16_t alpha_raw = encoder_read();
  float alpha = (float)alpha_raw / 1024.0;
  alpha -= ALPHA_OFFSET;
  alpha = angle_wrap(alpha);

  alpha = fmin(fmax(alpha, 0.0), M_PI / 2);
  shell_stream()->printf("alpha_raw: %d\r\n", alpha_raw);
  shell_stream()->printf("alpha: %f\r\n", alpha);

  return sqrt(pow(L1, 2) + pow(L3, 2) - 2 * L1 * L3 * cos(alpha));
}

void servo_init() {}

void servo_tick() {
  unsigned long elapsed = millis() - last_update;
  if (elapsed > 1) {
    last_update = millis();

    // TODO: Control
  }
}

SHELL_COMMAND(servo, "Set servo target") {
  if (argc != 1) {
    shell_stream()->println("Usage: servo <target>");
    return;
  }

  target = atof(argv[0]);
  enabled = true;
}

SHELL_COMMAND(l, "Get l") { shell_stream()->printf("l: %f\r\n", servo_l()); }

SHELL_COMMAND(em, "Emergency stop") {
  enabled = false;
  motor_set_pwm(0);
}