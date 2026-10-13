/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Autonomous Mobile Robot - Main FSM
 ******************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>

#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "uart.h"
#include "adc.h"
#include "Ultrasonic.h"

/* Button masks */
#define BUTTON_0_START      0x01U
#define BUTTON_1_STOP       0x02U
#define BUTTON_2_ROTATE     0x04U

/* Motor speeds */
#define SEARCH_SPEED_MM_S   ((int16_t)75)
#define FORWARD_SPEED_MM_S  ((int16_t)100)
#define REVERSE_SPEED_MM_S  ((int16_t)-100)
#define ROTATE_SPEED_MM_S   ((int16_t)100)

/* Approximate delay for the AVOID state.
 * Adjust this value if the rotation is too short or too long.
 */
#define AVOID_DELAY_COUNT 300000U

typedef enum {
    IDLE = 0,
    SEARCH,
    ORIENT,
    PATHFIND,
    STORAGE,
    DEPOSIT
} RobotState; //check 2

/*
 * TODO:
 * ORIENT requires a reliable way to measure a 90-degree turn.
 * Add odometry/gyro variables once the relevant Kobuki interface
 * is confirmed.
 */

/*
 * Send a raw Kobuki base-control packet.
 *
 * speed: linear velocity in mm/s
 * radius: turning radius in mm
 *
 * NOTE:
 * A radius of zero is a special Kobuki command for rotation
 * in place. A non-zero radius specifies a curved trajectory.
 */

static void Send_Kobuki_Move(int16_t speed, int16_t radius)
{
    uint8_t payload[6];

    payload[0] = 0x01U;
    payload[1] = 0x04U;

    payload[2] = (uint8_t)(speed & 0xFF);
    payload[3] = (uint8_t)((uint16_t)speed >> 8);

    payload[4] = (uint8_t)(radius & 0xFF);
    payload[5] = (uint8_t)((uint16_t)radius >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}

int main(void)
{
    Kobuki_Typedef my_kobuki = {0};
    US_Typedef ultrasonic_object = {0};

    RobotState state = IDLE;

    /*
     * ORIENT bookkeeping.
     *
     * These variables track the logical progress of the search.
     * They do NOT measure the robot's physical rotation.
     */
    bool orient_started = false;

    /* Initialise drivers using the existing working sequence. */
    UART_Init();
    ADC_Init();
    Ultrasonic_Init();

    /* Start safely with the robot stopped. */
    Send_Kobuki_Move(0, 0);

    while (1)
    {
        /* Update sensor data. */
        Kobuki_Read(&my_kobuki);
        Ultrasonic_Detect(&ultrasonic_object);

        /*
         * GLOBAL STOP
         *
         * Button 1 takes priority over all normal state actions.
         */
        if ((my_kobuki.button & BUTTON_1_STOP) != 0U)
        {
            state = IDLE;
            orient_started = false;

            Send_Kobuki_Move(0, 0);
        }
        else
        {
            /*
             * FSM TRANSITIONS
             */
            switch (state)
            {
                case IDLE:

                    /*
                     * Remain stopped until Button 0 is pressed.
                     */
                    if ((my_kobuki.button & BUTTON_0_START) != 0U)
                    {
                        state = SEARCH;
                        orient_started = false;
                    }

                    break;


                case SEARCH:

                    /*
                     * Rotate until the ultrasonic sensor detects
                     * a bollard or other object.
                     */
                    if (ultrasonic_object.detected != 0U)
                    {
                        Send_Kobuki_Move(0, 0);

                        state = ORIENT;
                        orient_started = false;
                    }

                    break;


                case ORIENT:

                    /*
                     * TODO: Implement the 90-degree orientation check.
                     *
                     * Required behaviour:
                     *
                     * 1. Rotate the robot by 90 degrees.
                     * 2. Determine when the turn has completed.
                     * 3. Check the ultrasonic sensor in the new direction.
                     *
                     * If an object is detected:
                     *     Begin another 90-degree turn.
                     *
                     * If no object is detected:
                     *     Transition to PATHFIND.
                     *
                     * IMPORTANT:
                     * Do not implement this using an arbitrary blocking
                     * delay if sensor readings must be checked during
                     * the rotation.
                     */

                    break;


                case PATHFIND:

                    /*
                     * TODO: Implement pathfinding transitions.
                     *
                     * Define how the robot follows the route and
                     * determine when it should enter STORAGE.
                     */

                    break;


                case STORAGE:

                    /*
                     * TODO: Implement storage-related transitions.
                     *
                     * Define the conditions for moving to DEPOSIT
                     * or returning to another state.
                     */

                    break;


                case DEPOSIT:

                    /*
                     * TODO: Implement deposit-related transitions.
                     *
                     * Define what signals successful completion and
                     * whether the robot should return to PATHFIND.
                     */

                    break;


                default:

                    /* Recover safely from an invalid state. */
                    state = IDLE;
                    orient_started = false;

                    break;
            }


            /*
             * STATE ACTIONS
             */
            switch (state)
            {
                case IDLE:

                    /* Keep the robot stopped. */
                    Send_Kobuki_Move(0, 0);

                    break;


                case SEARCH:

                    /*
                     * Rotate counter-clockwise while searching.
                     *
                     */
                    Kobuki_Rotate(SEARCH_SPEED_MM_S);

                    break;


                case ORIENT:

                    /*
                     * TODO: Implement the physical 90-degree turn.
                     *
                     * Use the robot's odometry or gyro feedback to
                     * determine when the turn is complete.
                     *
                     * Stop the robot before evaluating the next
                     * direction.
                     */

                    Send_Kobuki_Move(0, 0);

                    break;


                case PATHFIND:

                    /*
                     * TODO: Implement pathfinding movement.
                     *
                     * FORWARD_SPEED_MM_S is available for use once
                     * the required pathfinding behaviour is defined.
                     */

                    Send_Kobuki_Move(0, 0);

                    break;


                case STORAGE:

                    /*
                     * TODO: Implement storage behaviour.
                     */

                    Send_Kobuki_Move(0, 0);

                    break;


                case DEPOSIT:

                    /*
                     * TODO: Implement deposit behaviour.
                     */

                    Send_Kobuki_Move(0, 0);

                    break;


                default:

                    /* Safe fallback. */
                    Send_Kobuki_Move(0, 0);
                    state = IDLE;

                    break;
            }
        }

        /*
         * Small control-loop delay.
         */
        for (volatile uint32_t i = 0U;
             i < 5000U;
             ++i)
        {
            __NOP();
        }
    }
}