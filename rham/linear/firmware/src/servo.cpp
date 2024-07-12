#include "servo.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "shell.h"

// Servoing enabled
static bool enabled = false;

// Target length [m]
static float target = 0.2;

static unsigned long last_update = 0;

float angle_wrap(float angle) { return fmod(angle + M_PI, 2 * M_PI) - M_PI; }

float cosine_law_angle(float a, float b, float c) {
  return acos((a * a + b * b - c * c) / (2 * a * b));
}

float cosine_law_length(float angle, float a, float b) {
  return sqrt(a * a + b * b - 2 * a * b * cos(angle));
}

float servo_l(bool print = false) {
  // Reading alpha
  uint16_t alpha_raw = encoder_read() - ALPHA_OFFSET;
  float alpha = (float)alpha_raw / 1024.0;
  alpha = alpha * 2 * M_PI;
  float dist = sqrt(L1 * L1 + L2 * L2);
  float lmin_angle = cosine_law_angle(dist, L2, L_Min);
  alpha += lmin_angle;

  alpha = angle_wrap(alpha);
  if (print) {
    shell_stream()->printf("alpha: %f\r\n", alpha);
    // shell_stream()->printf("alpha_raw: %d\r\n", alpha_raw);
    // shell_stream()->printf("lmin_angle: %f\r\n", lmin_angle);
  }
  return cosine_law_length(alpha, L2, dist);
}

void servo_init() {}

SHELL_PARAMETER_INT(kp, "Proportional gain", 18.5);

void servo_tick() {
  unsigned long elapsed = millis() - last_update;
  if (elapsed >= 1 && enabled) {
    last_update = millis();

    float current_position = servo_l(false);
    float difference = target - current_position;

    float pwm = kp * 1023 * difference;

    if (pwm > 1024) {
      pwm = 1024;
    } else if (pwm < -1024) {
      pwm = -1024;
    }
    motor_set_pwm(pwm);
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

SHELL_COMMAND(l, "Get l") {
  shell_stream()->printf("l: %f\r\n", servo_l(true));
}

SHELL_COMMAND(em, "Emergency stop") {
  enabled = false;
  motor_set_pwm(0);
}
