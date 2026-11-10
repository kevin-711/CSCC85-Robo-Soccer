#include "robotController.h"

#define BASE_POWER 25
#define KP 2.0
#define KI 0.0
#define KD 0.0
#define I_LIMIT 20.0
#define GYRO_SIGN 1

#define OVERSHOOT 10

//****************** Turning *******************//

// Turning to target degree helper
int turn_by_gyro(int target_deg)
{
    int angle = 0, rate = 0;
    int dir = (target_deg > 0) ? 1 : -1;
    BT_all_stop(0);
    usleep(500000);
    if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1)
        return (-1);
    for (int i = 0; i < 200; i++)
    {
        if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1)
        {
            BT_all_stop(1);
            return (-1);
        }
        if (dir * GYRO_SIGN * angle >= abs(target_deg) - OVERSHOOT)
            break;
        BT_turn(LEFT_MOTOR, dir * 25, RIGHT_MOTOR, -dir * 25);
    }
    BT_all_stop(1);
    return (0);
}

void turnLeft(void)
{
    turn_by_gyro(-90);
}

void turnRight(void)
{
    turn_by_gyro(90);
}

//****************** PID *******************//

/*
 * PID Controller
 *
 * - Use this controller for driving in a straight line for x amount of time (good for small shifts in alignment)
 * - direction = 1 to drive fowards, direction = -1 to drive backwards
 *
 */
int drive_straight_pid(int max_ms, int direction)
{
    int angle = 0, rate = 0;
    double integral = 0, u;
    int lp, rp;
    int base = direction * BASE_POWER;

    struct timespec start;
    BT_all_stop(0);
    usleep(500000);

    if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1)
    {
        BT_all_stop(1);
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &start);

    while (elapsed_ms_since(start) < max_ms)
    {
        if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1)
        {
            BT_all_stop(1);
            return -1;
        }

        double err = GYRO_SIGN * angle;

        integral += err;
        if (integral > I_LIMIT)
            integral = I_LIMIT;
        if (integral < -I_LIMIT)
            integral = -I_LIMIT;

        u = KP * err + KI * integral + KD * GYRO_SIGN * rate;

        lp = (int)(base - u);
        rp = (int)(base + u);

        if (lp > 100)
            lp = 100;
        if (lp < -100)
            lp = -100;
        if (rp > 100)
            rp = 100;
        if (rp < -100)
            rp = -100;

        BT_turn(LEFT_MOTOR, lp, RIGHT_MOTOR, rp);
        usleep(10000);
    }

    BT_all_stop(1);
    return 0;
}
