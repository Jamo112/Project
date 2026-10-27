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


/* ===========================================================================
 * DEFINES
 * =========================================================================*/

// Button Mask
#define BUTTON_0_START      0x01U
#define BUTTON_1_STOP       0x02U
#define BUTTON_2_ROTATE     0x04U

// Motor speed limits
#define SEARCH_SPEED_MM_S   ((int16_t)75)
#define FORWARD_SPEED_MM_S  ((int16_t)100)
#define REVERSE_SPEED_MM_S  ((int16_t)-100)
#define ROTATE_SPEED_MM_S   ((int16_t)100)

// Approximate delay for the AVOID state.
// Adjust this value if the rotation is too short or too long.
#define AVOID_DELAY_COUNT   300000U

// Common distances
#define SENSOR_OFFSET_MM        500U    // A value that allows for easy adjustments of offset
#define TARGET_DISTANCE_350_MM  (350U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_400_MM  (400U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_150_MM  (150U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_100_MM  (100U + SENSOR_OFFSET_MM)
#define DEPOSIT_APPROACH_SPEED_MM_S ((int16_t)50)

// SEARCH tuning
#define SEARCH_ROTATION_INCREMENT_DEG   11

// ORIENT (bollard centring) tuning
// Gyro angle increases CCW: a NEGATIVE step is CW, a POSITIVE step is CCW.
// If your robot turns the wrong way, flip the signs of the steps in Bollard_centre().
#define ORIENT_STEP_DEG           2       // each sweep step (deg)
#define ORIENT_CONFIRM_SAMPLES    3U      // consecutive readings to confirm an edge
#define ORIENT_FIND_LIMIT_CDEG    4500L   // max sweep to re-enter the bollard (0.01 deg = 45 deg)
#define ORIENT_SPAN_LIMIT_CDEG    9000L   // max object width (0.01 deg = 90 deg), wider = wall
#define ORIENT_MIN_SPAN_CDEG      300L    // narrower than 3 deg = noise

// Logic
# define NULL 0
# define TRUE 1


// Bumper and Cliff sensor layouts
#define BUMPER_RIGHT  (1U << 0)
#define BUMPER_FRONT (1U << 1)
#define BUMPER_LEFT   (1U << 2)

#define CLIFF_RIGHT   (1U << 0)
#define CLIFF_FRONT  (1U << 1)
#define CLIFF_LEFT    (1U << 2)

/* ===========================================================================
 * TYPES AND GLOBAL VARIABLES
 * =========================================================================*/

// Main FSM states
typedef enum {
    IDLE = 0,
    SEARCH,
    ORIENT,
    PATHFIND,
    STORAGE,
    DEPOSIT
} RobotState;

RobotState state = IDLE;

// Which way the course goes
typedef enum {
    Course_Undefined = 0,
    Course_Left,
    Course_Right
} CourseDirection;

CourseDirection course_direction = Course_Undefined;

// Bollard centring sub-FSM stages
typedef enum {
    ORIENT_FIND_EDGE,       // step CW until the bollard is lost       -> theta_right
    ORIENT_REENTER,         // step CCW until the bollard is seen again
    ORIENT_FIND_EXIT,       // keep stepping CCW until it is lost      -> theta_left
    ORIENT_RETURN_CENTRE,   // turn to the middle of the two edges
    ORIENT_COMPLETE
} OrientStage;

static OrientStage orient_stage = ORIENT_FIND_EDGE;

// Edge headings are CUMULATIVE angles in 0.01 deg (no +/-180 wrap problems)
static int32_t theta_right  = 0;    // CW edge
static int32_t theta_left   = 0;    // CCW edge
static int32_t theta_target = 0;    // centreline = (theta_left + theta_right) / 2

static int16_t orient_last_angle = 0;   // previous gyro reading
static int32_t orient_cum        = 0;   // heading change since ORIENT started
static int32_t orient_edge_cand  = 0;   // heading at the first "lost" sample
static uint8_t orient_samples    = 0;
static bool    orient_first      = true;

bool centre_found = false;

// Rotation state
bool rot_started = false;

// Pathfind variables
typedef enum {
    PATH_DRIVE_FIRST = 0,
    PATH_BACKUP,
    PATH_TURN_90,
    PATH_DRIVE_SECOND,
    PATH_FACE_STORAGE,
    PATH_COMPLETE
} PathStage;

static PathStage path_stage = PATH_DRIVE_FIRST;
static PathStage path_after_backup = PATH_TURN_90;
bool path_started = false;

// Storage variables
uint8_t storage_step = 0;
bool storage_started = false;
static bool path_front_cliff_armed = true;

typedef enum {
    DEPOSIT_SEARCH = 0,
    DEPOSIT_CENTRE,
    DEPOSIT_APPROACH,
    DEPOSIT_COMPLETE
} DepositStage;

static DepositStage deposit_stage = DEPOSIT_SEARCH;


/* ===========================================================================
 * CUSTOM FUNCTIONS
 * =========================================================================*/

// ---------------------------------------------------------------------------
// DETECT_BOLLARD: refresh the ultrasonic reading, true if within distance
// ---------------------------------------------------------------------------
bool DETECT_BOLLARD(US_Typedef *object, uint16_t distance) {
    Ultrasonic_Detect(object);  // Update ultrasonic measurement

    if (object->range_mm <= distance) {
        return true;  // Object detected within the target distance
    }

    return false;  // No object detected within the target distance
}

// ---------------------------------------------------------------------------
// Rotate helpers. Rotate_Angle() is non-blocking: call it every loop until
// it returns true.
// ---------------------------------------------------------------------------
static void Rotate_Reset(void) {
    rot_started = false;
}

bool Rotate_Angle(Kobuki_Typedef *kobuki, int16_t angle) {
    // Set the target once when a new rotation starts
    if (!rot_started) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, angle);
        kobuki->rotation_complete = 0;  // clear stale flag from the last rotation
        rot_started = true;
        return false;
    }

    // Keep rotating towards the target
    Kobuki_Rotate_Angle(kobuki->angle);

    // Check if the rotation has finished
    if (kobuki->rotation_complete) {
        Kobuki_Drive(STOP);
        rot_started = false;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// ORIENT helpers
// ---------------------------------------------------------------------------

// Cancel any rotation step that is still in flight
static void Orient_Stop(void) {
    Kobuki_Drive(STOP);
    Rotate_Reset();
}

// Reset the WHOLE centring sub-FSM (otherwise a second run starts in COMPLETE)
static void Orient_Reset(void) {
    centre_found     = false;
    course_direction = Course_Undefined;
    orient_stage     = ORIENT_FIND_EDGE;
    orient_samples   = 0;
    orient_cum       = 0;
    orient_first     = true;
    Rotate_Reset();
}

static bool Emergency_Stop_Check(Kobuki_Typedef *kobuki, bool *rotating) {
    if ((state == IDLE) ||
        (((kobuki->button & BUTTON_1_STOP) == 0U) &&
         (kobuki->wheeldrop == 0U))) {
        return false;
    }

    Kobuki_Drive(STOP);
    state = IDLE;
    *rotating = false;
    Rotate_Reset();
    Orient_Reset();
    path_stage = PATH_DRIVE_FIRST;
    path_started = false;
    path_front_cliff_armed = true;
    storage_step = 0;
    storage_started = false;
    deposit_stage = DEPOSIT_SEARCH;

    return true;
}

// Lost the bollard / bad reading: stop and go back to SEARCH
static void Orient_Fail(void) {
    Orient_Stop();
    Orient_Reset();
    if (state == DEPOSIT) {
        deposit_stage = DEPOSIT_SEARCH;
    } else {
        state = SEARCH;
    }
}

// ---------------------------------------------------------------------------
// Bollard_centre: centre the robot on the bollard.
// Entered just after SEARCH first sees the bollard, so we start inside the beam.
//   1) step CW until the bollard is lost          -> theta_right
//   2) reverse, step CCW until it is seen again
//   3) keep going CCW until it is lost again      -> theta_left
//   4) turn to theta_target = (theta_left + theta_right) / 2
// ---------------------------------------------------------------------------
void Bollard_centre(Kobuki_Typedef *kobuki, US_Typedef *object) {
    int32_t delta;
    bool    seen = DETECT_BOLLARD(object, TARGET_DISTANCE_350_MM);

    // First call after a reset: take a reference heading
    if (orient_first) {
        orient_last_angle = kobuki->angle;
        orient_cum        = 0;
        orient_samples    = 0;
        orient_first      = false;
    }

    // Accumulate the heading change (gyro is 0.01 deg and wraps at +/-18000)
    delta = (int32_t)kobuki->angle - (int32_t)orient_last_angle;
    if (delta > 18000) {
        delta -= 36000;
    } else if (delta < -18000) {
        delta += 36000;
    }
    orient_cum += delta;
    orient_last_angle = kobuki->angle;

    switch (orient_stage) {
        // -------- 1) CW until the bollard is lost --------
        case ORIENT_FIND_EDGE:
            Rotate_Angle(kobuki, -ORIENT_STEP_DEG);

            if (!seen) {
                if (orient_samples == 0U) {
                    orient_edge_cand = orient_cum;      // first lost sample
                }
                orient_samples++;

                if (orient_samples >= ORIENT_CONFIRM_SAMPLES) {
                    theta_right    = orient_edge_cand;
                    orient_samples = 0;
                    Orient_Stop();                      // cancel the CW step in flight
                    orient_stage   = ORIENT_REENTER;
                }
            } else {
                orient_samples = 0;
            }

            // Never lost it: too wide to be a bollard
            if ((-orient_cum) > ORIENT_SPAN_LIMIT_CDEG) {
                Orient_Fail();
            }
            break;

        // -------- 2) reverse until the bollard is back in the beam --------
        case ORIENT_REENTER:
            Rotate_Angle(kobuki, ORIENT_STEP_DEG);

            if (seen) {
                orient_samples++;

                if (orient_samples >= ORIENT_CONFIRM_SAMPLES) {
                    orient_samples = 0;
                    orient_stage   = ORIENT_FIND_EXIT;  // same direction, keep going
                }
            } else {
                orient_samples = 0;
            }

            // Lost it for good
            if ((orient_cum - theta_right) > ORIENT_FIND_LIMIT_CDEG) {
                Orient_Fail();
            }
            break;

        // -------- 3) CCW until the bollard is lost again --------
        case ORIENT_FIND_EXIT:
            Rotate_Angle(kobuki, ORIENT_STEP_DEG);

            if (!seen) {
                if (orient_samples == 0U) {
                    orient_edge_cand = orient_cum;      // first lost sample
                }
                orient_samples++;

                if (orient_samples >= ORIENT_CONFIRM_SAMPLES) {
                    theta_left     = orient_edge_cand;
                    orient_samples = 0;
                    Orient_Stop();

                    if ((theta_left - theta_right) < ORIENT_MIN_SPAN_CDEG) {
                        Orient_Fail();                  // noise spike
                    } else {
                        theta_target = (theta_left + theta_right) / 2;
                        orient_stage = ORIENT_RETURN_CENTRE;
                    }
                }
            } else {
                orient_samples = 0;
            }

            // Too wide to be a bollard (wall?)
            if ((orient_stage == ORIENT_FIND_EXIT) &&
                ((orient_cum - theta_right) > ORIENT_SPAN_LIMIT_CDEG)) {
                Orient_Fail();
            }
            break;

        // -------- 4) turn to the centreline --------
        case ORIENT_RETURN_CENTRE:
            // Angle to turn is evaluated every call, but only used when the rotation starts
            if (Rotate_Angle(kobuki, (int16_t)((theta_target - orient_cum) / 100))) {
                centre_found = true;
                orient_stage = ORIENT_COMPLETE;
            }
            break;

        case ORIENT_COMPLETE:
            centre_found = true;
            break;

        default:
            orient_stage = ORIENT_FIND_EDGE;
            break;
    }
}

static void Send_Kobuki_Move(int16_t speed, int16_t radius) {
    uint8_t payload[6];

    payload[0] = 0x01U;
    payload[1] = 0x04U;

    payload[2] = (uint8_t)(speed & 0xFF);
    payload[3] = (uint8_t)((uint16_t)speed >> 8);

    payload[4] = (uint8_t)(radius & 0xFF);
    payload[5] = (uint8_t)((uint16_t)radius >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}


/* ===========================================================================
 * MAIN
 * =========================================================================*/
int main ( void ) {
    // Both of these only for testing purposes . Not required
    volatile uint16_t ultrasonic_read;
    volatile uint16_t adc_read;
    bool search_done = false;
    bool rotating = false;
    bool found_bollard = false;
    Kobuki_Typedef kobuki = {0};
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

        // ---------------------------------------------------------------
        case IDLE:
            Kobuki_Drive(STOP);
            Kobuki_Update_LEDs(GREEN, GREEN);
            // led_1 = GREEN;
            // led_2 = GREEN;

            if ((kobuki.button & BUTTON_0_START) != 0U) {
                state = SEARCH;
            }
            break;

        // ---------------------------------------------------------------
        case SEARCH:
            Kobuki_Update_LEDs(RED, OFF);

            if (Emergency_Stop_Check(&kobuki, &rotating)) {
                state = IDLE;
                break;
            }

            // If currently rotating, continue until rotation completes
            if (rotating) {
                if (Rotate_Angle(&kobuki, SEARCH_ROTATION_INCREMENT_DEG)) {
                    rotating = false;
                }

                break;
            }

            // Only check the sensor when stationary
            if (DETECT_BOLLARD(&object, TARGET_DISTANCE_350_MM)) {
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

        // ---------------------------------------------------------------
        case ORIENT:
            Kobuki_Update_LEDs(YELLOW, OFF);
            if (Emergency_Stop_Check(&kobuki, &rotating)) {
                state = IDLE;
                break;
            }
            

            if (!centre_found) {
                // Step 1: centre on the bollard
                Bollard_centre(&kobuki, &object);
            } else {
                // Step 2: decide the course ONCE, now that we are centred and stationary
                if (course_direction == Course_Undefined) {
                    if (DETECT_BOLLARD(&object, TARGET_DISTANCE_150_MM)) {
                        course_direction = Course_Left;
                    } else {
                        course_direction = Course_Right;
                    }
                    // if we see a bollard within 150mm we go left route else we go the right route. 

                    Rotate_Reset();
                }

                // Step 3: rotate 90 deg (called every loop until it finishes)
                if (course_direction == Course_Left) {
                    if (Rotate_Angle(&kobuki, 90)) {
                        state = PATHFIND;
                    }
                } else {
                    if (Rotate_Angle(&kobuki, -90)) {
                        state = PATHFIND;
                    }
                }
            }
            break;

        // ---------------------------------------------------------------
        case PATHFIND:
            Kobuki_Update_LEDs(GREEN, OFF);
            if (Emergency_Stop_Check(&kobuki, &rotating)) {
                state = IDLE;
                break;
            }

            if ((kobuki.bumper != 0U) ||
                ((kobuki.cliff & (CLIFF_LEFT | CLIFF_RIGHT)) != 0U)) {
                Kobuki_Drive(STOP);
                break;
            }

            if ((kobuki.cliff & CLIFF_FRONT) == 0U) {
                path_front_cliff_armed = true;
            } else if (path_front_cliff_armed &&((path_stage == PATH_DRIVE_FIRST) ||(path_stage == PATH_DRIVE_SECOND))) {
                Kobuki_Drive(STOP);
                path_started = false;
                path_front_cliff_armed = false;

                if (path_stage == PATH_DRIVE_FIRST) {
                    path_stage = PATH_TURN_90;
                } else {
                    path_stage = PATH_FACE_STORAGE;
                }
            }

            switch (path_stage) {
            case PATH_DRIVE_FIRST:
            case PATH_DRIVE_SECOND:
                // Approach until the centre (front) cliff sensor triggers.
                if ((kobuki.cliff & 0x02U) != 0U) {
                    Kobuki_Drive(STOP);
                    path_started = false;
                    if (path_stage == PATH_DRIVE_FIRST) {
                        path_after_backup = PATH_TURN_90;
                    } else {
                        path_after_backup = PATH_FACE_STORAGE;
                    }
                    path_stage = PATH_BACKUP;
                } else {
                    Kobuki_Drive(FORWARD_SPEED_MM_S);
                }
                break;

            case PATH_BACKUP:
                // Shared 100 mm backup after front cliff detection.
                if (!path_started) {
                    Kobuki_Drive_Distance_Setpoint(kobuki.distance, -100);
                    kobuki.distance_complete = 0;
                    path_started = true;
                }

                Kobuki_Drive_Distance(kobuki.distance);

                if (kobuki.distance_complete) {
                    Kobuki_Drive(STOP);
                    path_started = false;
                    path_stage = path_after_backup;
                }
                break;

            case PATH_TURN_90:
                // 90 degree rotation depending on the course direction (left or right)
                if (course_direction == Course_Left) {
                    if (Rotate_Angle(&kobuki, 90)) {
                        path_stage = PATH_DRIVE_SECOND;
                    }
                } else if (course_direction == Course_Right) {
                    if (Rotate_Angle(&kobuki, -90)) {
                        path_stage = PATH_DRIVE_SECOND;
                    }
                }
                break;

            case PATH_FACE_STORAGE:
                // 135 degree rotation to face storage area
                if (course_direction == Course_Left) {
                    if (Rotate_Angle(&kobuki, 135)) {
                        path_stage = PATH_COMPLETE;
                    }
                } else if (course_direction == Course_Right) {
                    if (Rotate_Angle(&kobuki, -135)) {
                        path_stage = PATH_COMPLETE;
                    }
                }
                break;

            case PATH_COMPLETE:
                // Finished path
                Kobuki_Drive(STOP);
                path_stage = PATH_DRIVE_FIRST;
                path_started = false;
                path_front_cliff_armed = true;
                state = STORAGE;
                break;

            default:
                Kobuki_Drive(STOP);
                path_stage = PATH_DRIVE_FIRST;
                path_started = false;
                break;
            }
            break;

        // ---------------------------------------------------------------
        case STORAGE:
            Kobuki_Update_LEDs(YELLOW, GREEN);
            if (Emergency_Stop_Check(&kobuki, &rotating)) {
                break;
            }

            if (storage_step == 0) {
                // Drive into the storage area
                if (!storage_started) {
                    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 800);
                    kobuki.distance_complete = 0;
                    storage_started = true;
                }

                Kobuki_Drive_Distance(kobuki.distance);

                if (kobuki.distance_complete) {
                    Kobuki_Drive(STOP);
                    storage_started = false;
                    storage_step = 1;
                }
            }
            else if (storage_step == 1) {
                // Finished entering storage
                storage_step = 0;
                deposit_stage = DEPOSIT_SEARCH;
                Orient_Reset();
                state = DEPOSIT;
            }

            break;
        // ---------------------------------------------------------------
        case DEPOSIT:
            Kobuki_Update_LEDs(GREEN, YELLOW);
            if (Emergency_Stop_Check(&kobuki, &rotating)) {
                break;
            }

            switch (deposit_stage) {
                case DEPOSIT_SEARCH:
                    // Turn until the bollard enters the ultrasonic beam.
                    if (DETECT_BOLLARD(&object, TARGET_DISTANCE_350_MM)) {
                        Orient_Stop();
                        Orient_Reset();
                        deposit_stage = DEPOSIT_CENTRE;
                    } else {
                        Kobuki_Rotate(SEARCH_SPEED_MM_S);
                    }
                    break;

                case DEPOSIT_CENTRE:
                    Bollard_centre(&kobuki, &object);
                    if (centre_found) {
                        deposit_stage = DEPOSIT_APPROACH;
                    }
                    break;

                case DEPOSIT_APPROACH:
                    Ultrasonic_Detect(&object);
                    if (object.range_mm <= TARGET_DISTANCE_100_MM) {
                        Kobuki_Drive(STOP);
                        deposit_stage = DEPOSIT_COMPLETE;
                    } else if (object.range_mm > TARGET_DISTANCE_350_MM) {
                        // Centre again if the target is lost.
                        Orient_Stop();
                        Orient_Reset();
                        deposit_stage = DEPOSIT_SEARCH;
                    } else {
                        Kobuki_Drive(DEPOSIT_APPROACH_SPEED_MM_S);
                    }
                    break;

                case DEPOSIT_COMPLETE:
                    Kobuki_Drive(STOP);
                    state = IDLE;
                    break;

                default:
                    Kobuki_Drive(STOP);
                    break;
            }
            break;
        }
    }
}
