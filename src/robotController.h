#ifndef __robotController_header
#define __robotController_header

#include "API/btcomm.h"
#include <time.h>

#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D2:17"
#endif

#define COLOUR_PORT PORT_1
#define GYRO_PORT PORT_2
#define TOUCH_PORT PORT_3
#define LEFT_MOTOR MOTOR_B
#define RIGHT_MOTOR MOTOR_C
#define SLIDE_MOTOR MOTOR_D

// Movement
int drive_straight_pid(int max_ms, int direction);
int turn_by_gyro(int target_deg);
void turnLeft();
void turnRight();
void stopRobot();
void shoot();

long elapsed_ms_since(struct timespec start);

#endif
