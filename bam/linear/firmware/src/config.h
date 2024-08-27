#ifndef _CONFIG_H
#define _CONFIG_H

// Encoder pinout configuration
#define ENCODER_SCK 18
#define ENCODER_DO 19
#define ENCODER_SS 5

// Motors pinout configuration
#define MOTOR_PIN1 32
#define MOTOR_PIN2 33
#define RELAY_PIN 15

// Actuator tip to axis distance [m]
#define L1 0.2

// Actuator base to axis distance [m]
#define L2 0.055

#define L_Min 0.1545

#define L_Max 0.2545

// Offset for alpha [encoder step]
#define ALPHA_OFFSET 999




#endif