#include <Arduino.h>
#include "config.h"
#include "ndebug.h"

#define HCSR04_I2C_Address 0x14

#define HCSR04_I2C_REGISTRY_STATUS          0xF0
#define HCSR04_I2C_REGISTRY_DISTANCE_HIGH   0xF1
#define HCSR04_I2C_REGISTRY_DISTANCE_LOW    0xF2

#define trigPin 3                                     // Pin 12 trigger output
#define echoPin 2                                     // Pin 2 Echo input
#define echo_int 0                                    // Interrupt id for echo pulse

#define SONAR_OUT_OF_RANGE (-1)
#define DISTANCE_SAMPLES_MEDIAN 5
#define HCSR04_MAX_RANGE_CM 400 // 4m, from HC-SR04 spec sheet

enum eEchoState { START_MEASURE = 0, MEASURING = 1, ECHO = 2 };

void updateSonarDistance(int32_t cm);


