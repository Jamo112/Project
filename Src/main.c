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

// Button Mask
#define BUTTON_0_START      0x01U
#define BUTTON_1_STOP       0x02U
#define BUTTON_2_ROTATE     0x04U

// motor speed limits
#define SEARCH_SPEED_MM_S   ((int16_t)75)
#define FORWARD_SPEED_MM_S  ((int16_t)100)
#define REVERSE_SPEED_MM_S  ((int16_t)-100)
#define ROTATE_SPEED_MM_S   ((int16_t)100)

//Approximate delay for the AVOID state.
//Adjust this value if the rotation is too short or too long.
#define AVOID_DELAY_COUNT 300000U

// FSM states
typedef enum {
    IDLE = 0,
    SEARCH,
    ORIENT,
    PATHFIND,
    STORAGE,
    DEPOSIT
} RobotState;

// Logic
# define NULL 0
# define TRUE 1

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

int main ( void ) {
    // Both of these only for testing purposes . Not required
    volatile uint16_t ultrasonic_read;
    volatile uint16_t adc_read;
    Kobuki_Typedef kobuki;
    US_Typedef object;
    Kobuki_LED_Enum led_1, led_2;
    UART_Init ();
    ADC_Init ();
    Ultrasonic_Init () ;
    state = IDLE;
    led_1 = OFF;
    led_2 = OFF;
    while (1) {
        Kobuki_Read (&kobuki );
        Ultrasonic_Detect (&object ) ;
        // Both of these only for testing purposes . Not required
        ultrasonic_read = Ultrasonic_Read();
        adc_read = ADC_Read ( ADC_CHANNEL_1);
        Kobuki_Update_LEDs ( led_1 , led_2);
        // A TEST FSM
        switch (state) {
        case IDLE:
            if (kobuki.button == BUTTON_0_START){
            state = SEARCH;
            led_1 = RED;
            Kobuki_Rotate_Angle_Setpoint( kobuki.angle , 450) ;
            // Kobuki_Drive_Distance_Setpoint ( kobuki .distance ,100) ;
            }else if (kobuki.button == BUTTON_1_STOP) {
            state = SEARCH;
            led_1 = RED;
            Kobuki_Rotate_Angle_Setpoint( kobuki.angle, 270) ;
            // Kobuki_Drive_Distance_Setpoint ( kobuki .distance , -100) ;
            }else if (kobuki.button == BUTTON_2_ROTATE) {
            state = SEARCH;
            led_1 = RED;
            Kobuki_Rotate_Angle_Setpoint( kobuki.angle , 90);
            // Kobuki_Drive_Distance_Setpoint ( kobuki .distance ,1000) ;
            } else {
                Kobuki_Drive(STOP) ;
            }
            break;
        case SEARCH :
            if((kobuki.wheeldrop != NULL)|((kobuki.bumper+kobuki.cliff) != NULL)|(kobuki.rotation_complete == TRUE)|(kobuki.distance_complete == TRUE)){
            state = IDLE;
            led_1 = YELLOW;
            }else {
            Kobuki_Rotate_Angle( kobuki.angle);
            // Kobuki_Drive_Distance ( kobuki . distance );
            }
            break ;
        case ORIENT:
            break;
        case PATHFIND:
            break;
        case STORAGE:
            break;
        case DEPOSIT:
            break;
        }
    }
}
