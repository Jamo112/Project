/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Autonomous Mobile Robot - Main FSM
 ******************************************************************************
 */

#include "ultrasonic.h"
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

//Common distances
#define SENSOR_OFFSET_MM        200U // A value that allows for easy adjustments of offset
#define TARGET_DISTANCE_350_MM  (350U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_400_MM  (400U + SENSOR_OFFSET_MM)

#define SEARCH_ROTATION_INCREMENT_DEG 17

// ORIENT (bollard alignment) tuning ///////////////////////////////////
#define ORIENT_SWEEP_SPEED_MM_S   ((int16_t)-50)  // slow so the ultrasonic keeps up
#define ORIENT_CONFIRM_SAMPLES    3U              // consecutive readings to confirm an edge
#define ORIENT_FIND_LIMIT_CDEG    4500L           // max sweep (0.01 deg) to find left edge (45 deg)
#define ORIENT_SPAN_LIMIT_CDEG    9000L           // max object width (90 deg) - wider = wall, not bollard


// FSM states
typedef enum {
    IDLE = 0,
    SEARCH,
    ORIENT,
    PATHFIND,
    STORAGE,
    DEPOSIT
} RobotState;

typedef enum {
    Course_Undefined = 0,
    Course_Left,
    Course_Right
} CourseDirection;

typedef enum {
    First = 0,
    Second
}PathfindState;

CourseDirection course_direction = Course_Undefined;

// Logic
# define NULL 0
# define TRUE 1

// CUSTOM FUNCTIONS //////////////////////////////////////////
/////////////////////////////////////////////////////////////
bool DETECT_BOLLARD(US_Typedef *object, uint16_t distance)
{
    Ultrasonic_Detect(object);  // Update ultrasonic measurement

    if (object->range_mm <= distance)
    {
        return true;  // Object detected within the target distance
    }

    return false;  // No object detected within the target distance
}

static void Rotate_Reset(void)
{
    rot_started = false;
}
 
bool Rotate_Angle(Kobuki_Typedef *kobuki, int16_t angle)
{
    // Set the target once when a new rotation starts
    if (!rot_started)
    {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, angle);
        // kobuki->rotation_complete is only refreshed in Kobuki_Read(), so it still
        // holds the PREVIOUS rotation's result here. Clear our copy or this call
        // would report "finished" straight away on every rotation after the first.
        kobuki->rotation_complete = 0;
        rot_started = true;
        return false;
    }
 
    // Keep rotating towards the target
    Kobuki_Rotate_Angle(kobuki->angle);
 
    // Check if the rotation has finished
    if (kobuki->rotation_complete)
    {
        Kobuki_Drive(STOP);
        rot_started = false;
        return true;
    }
 
    return false;
}
 
/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

// ORIENT sub-FSM: centre the robot on a bollard ///////////////////////////
// Entered just after SEARCH has swept past the bollard's edge. Sweeps CW across
// the bollard, records the heading where it enters the beam (left edge) and where
// it leaves (right edge), then turns to the average of the two (the centreline).





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
    bool search_done = false; 
    bool rotating = false;
    bool found_bollard = false;
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
            Kobuki_Drive(STOP);

            led_1 = GREEN;
            led_2 = GREEN;
            if (kobuki.button == BUTTON_0_START){
            state = SEARCH;
            }
            break;


        case SEARCH:
 
            led_1 = RED;
            led_2 = OFF;
 
             // If currently rotating, continue until rotation completes
            if (rotating)
            {
                if (Rotate_Angle(&kobuki, SEARCH_ROTATION_INCREMENT_DEG))
                {
                    rotating = false;
                }
 
                break;
            }
                // Only check the sensor when stationary
            if (DETECT_BOLLARD(&object, TARGET_DISTANCE_350_MM))
            {
                // Bollard found: hand straight over to ORIENT to centre on it
                Kobuki_Drive(STOP);
                rotating = false;
                Rotate_Reset();
                Orient_Reset();
                state = ORIENT;
                break;
            }
 
            // Nothing in range: rotate another 17 deg and look again
            rotating = true;
            Rotate_Reset();
            break;
 
            break;
 

        case ORIENT:
            if    


            break;
        case PATHFIND:
            if (course_direction == Course_Right){
                Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1200);
                Kobuki_Drive_Distance(kobuki.distance);
                Kobuki_Rotate_Distance_Setpoint(kobuki.distance, -90);
                Kobuki_Rotate_Distance(kobuki.distance);
            } else if (course_direction == Course_Left){
                Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1200);
                Kobuki_Drive_Distance(kobuki.distance);
                Kobuki_Rotate_Distance_Setpoint(kobuki.distance, 90);
                Kobuki_Rotate_Distance(kobuki.distance);
            }
            break;
        case STORAGE:
            break;
        case DEPOSIT:
            break;
        }
    }
}
