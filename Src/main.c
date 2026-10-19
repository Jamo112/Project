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
#define TARGET_DISTANCE_150_MM  (150U + SENSOR_OFFSET_MM)

#define SEARCH_ROTATION_INCREMENT_DEG 17

// ORIENT (bollard alignment) tuning ///////////////////////////////////
#define ORIENT_SWEEP_SPEED_MM_S   ((int16_t)-50)  // slow so the ultrasonic keeps up
#define ORIENT_CONFIRM_SAMPLES    3U              // consecutive readings to confirm an edge
#define ORIENT_FIND_LIMIT_CDEG    4500L           // max sweep (0.01 deg) to find left edge (45 deg)
#define ORIENT_SPAN_LIMIT_CDEG    9000L           // max object width (90 deg) - wider = wall, not bollard

///// FOR BOLLARD CENTRE
typedef enum
{
    ORIENT_FIND_EDGE,
    ORIENT_FIND_EXIT,
    ORIENT_RETURN_CENTRE,
    ORIENT_COMPLETE
} OrientStage;

static OrientStage orient_stage = ORIENT_FIND_EDGE;

static int16_t theta_left = 0;
static int16_t theta_right = 0;
static int16_t theta_target = 0;

static uint8_t orient_samples = 0;
static bool orient_rotating = false;

// FSM states
typedef enum {
    IDLE = 0,
    SEARCH,
    ORIENT,
    PATHFIND,
    STORAGE,
    DEPOSIT
} RobotState;

RobotState state = IDLE;

typedef enum {
    Course_Undefined = 0,
    Course_Left,
    Course_Right
} CourseDirection;

CourseDirection course_direction = Course_Undefined;

// Pathfind variables
uint8_t path_step = 0;
bool path_started = false;

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

bool rot_started = false;
static void Rotate_Reset(void)
{
    rot_started = false;
}

bool centre_found = false; 
static void Orient_Reset(void)
{
    centre_found = false;
}
 
bool Rotate_Angle(Kobuki_Typedef *kobuki, int16_t angle)
{
    // Set the target once when a new rotation starts
    if (!rot_started)
    {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, angle);
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
 
void Bollard_centre(Kobuki_Typedef *kobuki, US_Typedef *object)
{
    switch (orient_stage)
    {
        case ORIENT_FIND_EDGE:
            if (!orient_rotating)
            {
                Rotate_Reset();
                orient_rotating = true;
            }

            if (Rotate_Angle(kobuki, -5))
                orient_rotating = false;

            if (!DETECT_BOLLARD(object, TARGET_DISTANCE_350_MM))
            {
                orient_samples++;

                if (orient_samples >= ORIENT_CONFIRM_SAMPLES)
                {
                    theta_left = kobuki->angle / 100.0;
                    orient_samples = 0;
                    orient_stage = ORIENT_FIND_EXIT;
                }
            }
            else
                orient_samples = 0;

            break;

        case ORIENT_FIND_EXIT:
            if (!orient_rotating)
            {
                Rotate_Reset();
                orient_rotating = true;
            }

            if (Rotate_Angle(kobuki, -5))
                orient_rotating = false;

            if (DETECT_BOLLARD(object, TARGET_DISTANCE_350_MM))
            {
                orient_samples++;

                if (orient_samples >= ORIENT_CONFIRM_SAMPLES)
                {
                    theta_right = kobuki->angle / 100.0;
                    orient_samples = 0;
                    theta_target = (theta_left + theta_right) / 2.0;

                    Rotate_Reset();
                    orient_stage = ORIENT_RETURN_CENTRE;
                }
            }
            else
                orient_samples = 0;

            break;

        case ORIENT_RETURN_CENTRE:
            if (Rotate_Angle(kobuki, (int16_t)(theta_target - kobuki->angle / 100.0)))
            {
                centre_found = true;
                orient_stage = ORIENT_COMPLETE;
            }
            break;

        case ORIENT_COMPLETE:
            centre_found = true;
            break;
    }
}

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
 

        case ORIENT:
            if (!centre_found){
                Bollard_centre();
            }
            else {
                if (DETECT_BOLLARD(&object, TARGET_DISTANCE_150_MM))
                Rotate_Angle(&kobuki, -90);
                course_direction = Course_Left;
                State = PATHFIND;
             
                else {
                Rotate_Angle(&Kobuki, 90);
                course_direction = Course_Right;
                State = PATHFIND;
                }
            }
            break;
            

        case PATHFIND:
            // Drive forward 1200mm
            if (path_step == 0 || path_step == 2) {

                if (!path_started) {
                    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 1200);
                    kobuki.distance_complete = 0;
                    path_started = true;
                }

                Kobuki_Drive_Distance(kobuki.distance);

                if (kobuki.distance_complete) {
                    Kobuki_Drive(STOP);
                    path_started = false;
                    path_step++;
                }
            }

            // 90 degree rotation depennding on the course direction (left or right)
            else if (path_step == 1) {

                if (course_direction == Course_Left) {
                    if (Rotate_Angle(&kobuki, -90)) {
                        path_step++;
                    }
                }
                else if (course_direction == Course_Right) {
                    if (Rotate_Angle(&kobuki, 90)) {
                        path_step++;
                    }
                }
            }

            // 135 degree rotation to face storage area
            else if (path_step == 3) {

                if (course_direction == Course_Left) {
                    if (Rotate_Angle(&kobuki, -135)) {
                        path_step++;
                    }
                }
                else if (course_direction == Course_Right) {
                    if (Rotate_Angle(&kobuki, 135)) {
                        path_step++;
                    }
                }
            }

            // Finished path
            else if (path_step == 4) {
                Kobuki_Drive(STOP);
                path_step = 0;
                path_started = false;
                state = STORAGE;
            }
            break;
        case STORAGE:
            break;
        case DEPOSIT:
            break;
        }
    }
}
