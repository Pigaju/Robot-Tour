#include <stdint.h>

// Central definitions for motor speed globals referenced across the project.
// Use `volatile int` to match existing usage patterns (signed, volatile access).
volatile int motor_l_speed = 0;
volatile int motor_r_speed = 0;
