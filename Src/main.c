/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Kuboki_Project
 ******************************************************************************
    Written by:
    Jesse Jamieson: A1925902
    Lucas Shaw: A1886336
    Lachlan Smart: A1924193


 */

#include "Ultrasonic.h"
#include <stdint.h>
#include <stdbool.h>

#define STM32G474xx
#include "stm32g4xx.h"
#include "kobuki.h"
#include "uart.h"
#include "ADC.h"
#include "Ultrasonic.h"


/* ===========================================================================
 * DEFINES
 * =========================================================================*/

// Button definitions
#define BUTTON_0_START      0x01U
#define BUTTON_1_STOP       0x02U
#define BUTTON_2_ROTATE     0x04U

// Motor speed limits
#define SEARCH_SPEED_MM_S   ((int16_t)75)
#define FORWARD_SPEED_MM_S  ((int16_t)100)
#define REVERSE_SPEED_MM_S  ((int16_t)-100)
#define ROTATE_SPEED_MM_S   ((int16_t)100)

// Approximate delay for the AVOID state.
#define AVOID_DELAY_COUNT   300000U

// Common distances
#define SENSOR_OFFSET_MM        500U    // Offset added to ultrasonic distance measurements
#define TARGET_DISTANCE_350_MM  (350U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_400_MM  (400U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_150_MM  (150U + SENSOR_OFFSET_MM)
#define TARGET_DISTANCE_100_MM  (100U + SENSOR_OFFSET_MM)
#define DEPOSIT_APPROACH_SPEED_MM_S ((int16_t)50)
#define DEPOSIT_DETECT_DISTANCE_MM  (50U + SENSOR_OFFSET_MM)
#define DEPOSIT_SENSOR_OFFSET_MM   200U
#define DEPOSIT_FRONT_CLEARANCE_MM  200U
#define DEPOSIT_STOP_DISTANCE_MM   (DEPOSIT_SENSOR_OFFSET_MM + DEPOSIT_FRONT_CLEARANCE_MM)

// SEARCH tuning
#define SEARCH_ROTATION_INCREMENT_DEG   -11

// Bumper recovery distances and turn angle.
#define FRONT_BUMPER_REVERSE_MM  100
#define SIDE_BUMPER_REVERSE_MM   80
#define SIDE_BUMPER_TURN_DEG     6

// Kobuki bumper and cliff sensor bit positions.
#define BUMPER_RIGHT  (1U << 0)
#define BUMPER_FRONT  (1U << 1)
#define BUMPER_LEFT   (1U << 2)

#define CLIFF_RIGHT   (1U << 0)
#define CLIFF_FRONT   (1U << 1)
#define CLIFF_LEFT    (1U << 2)

// ORIENT (bollard centring) tuning
// Gyro angle increases CCW: a NEGATIVE step is CW, a POSITIVE step is CCW.
#define ORIENT_STEP_DEG           2       // each sweep step (deg)
#define ORIENT_CONFIRM_SAMPLES    3U      // consecutive readings to confirm an edge
#define ORIENT_FIND_LIMIT_CDEG    4500L   // max sweep to re-enter the bollard (0.01 deg = 45 deg)
#define ORIENT_MIN_SPAN_CDEG      300L    // narrower than 3 deg = noise

// Logic
# define NULL 0
# define TRUE 1


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

RobotState Robot_State = IDLE;

// Which way the course goes
typedef enum {
    Course_Undefined = 0,
    Course_Left,
    Course_Right
} CourseDirection;

CourseDirection Course_Direction = Course_Undefined;

// Bollard centring sub-FSM stages
typedef enum {
    ORIENT_FIND_EDGE,       // step CW until the bollard is lost       -> theta_right
    ORIENT_REENTER,         // step CCW until the bollard is seen again
    ORIENT_FIND_EXIT,       // keep stepping CCW until it is lost      -> theta_left
    ORIENT_RETURN_CENTRE,   // turn to the middle of the two edges
    ORIENT_COMPLETE
} OrientStage;

static OrientStage Orient_Stage = ORIENT_FIND_EDGE;

// Edge headings are CUMULATIVE angles in 0.01 deg (no +/-180 wrap problems)
static int32_t Theta_Right  = 0;    // CW edge
static int32_t Theta_Left   = 0;    // CCW edge
static int32_t Theta_Target = 0;    // centreline = (theta_left + theta_right) / 2

static int16_t Orient_Last_Angle = 0;   // previous gyro reading
static int32_t Orient_Cumulative_Angle        = 0;   // heading change since ORIENT started
static int32_t Orient_Edge_Angle  = 0;   // heading at the first "lost" sample
static uint8_t Orient_Samples    = 0;
static bool    Orient_First      = true;

bool Centre_Found = false;

// Rotation state
bool Rotation_Started = false;

// Pathfind variables
typedef enum {
    PATH_DRIVE_FIRST = 0,
    PATH_BACKUP,
    PATH_TURN_90,
    PATH_DRIVE_SECOND,
    PATH_FACE_STORAGE,
    PATH_COMPLETE
} PathStage;

static PathStage Path_Stage = PATH_DRIVE_FIRST;
static PathStage Path_After_Backup = PATH_TURN_90;
bool Path_Started = false;

// Storage variables
uint8_t Storage_Step = 0;
bool Storage_Started = false;
static bool Path_Front_Cliff_Armed = true;

typedef enum {
    DEPOSIT_SEARCH = 0,
    DEPOSIT_CENTRE,
    DEPOSIT_APPROACH,
    DEPOSIT_COMPLETE
} DepositStage;

static DepositStage Deposit_Stage = DEPOSIT_SEARCH;
static bool Deposit_Rotating = false;

typedef enum {
    BUMPER_IDLE = 0,
    BUMPER_REVERSE,
    BUMPER_TURN,
    BUMPER_WAIT_RELEASE
} BumperStage;

typedef enum {
    BUMPER_ACTION_NONE = 0,
    BUMPER_ACTION_FRONT,
    BUMPER_ACTION_SIDE
} BumperAction;

static BumperStage Bumper_Stage = BUMPER_IDLE;
static BumperAction Bumper_Action = BUMPER_ACTION_NONE;
static int16_t Bumper_Turn_Angle = 0;
static int32_t Bumper_Resume_Distance_Setpoint = 0;
static bool Bumper_Resume_Distance = false;
static uint8_t Bumper_Release_Bumper_Mask = 0U;
static uint8_t Bumper_Release_Cliff_Mask = 0U;


/* ===========================================================================
 * CUSTOM FUNCTIONS
 * =========================================================================*/

// ---------------------------------------------------------------------------
// Detect_Bollard: refresh the ultrasonic reading, true if within distance
// ---------------------------------------------------------------------------
bool Detect_Bollard(US_Typedef *Ultrasonic_Object, uint16_t Detection_Distance) {
    Ultrasonic_Detect(Ultrasonic_Object);  // Update ultrasonic measurement

    if (Ultrasonic_Object->range_mm <= Detection_Distance) {
        return true;  // Object detected within the target distance
    }

    return false;  // No object detected within the target distance
}

// ---------------------------------------------------------------------------
// Rotate helpers. Rotation_Angle() is non-blocking: call it every loop until
// it returns true.
// ---------------------------------------------------------------------------
static void Rotation_Reset(void) {
    Rotation_Started = false;
}

bool Rotation_Angle(Kobuki_Typedef *kobuki, int16_t Rotation_Degrees) {
    // Set the target once when a new rotation starts
    if (!Rotation_Started) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Rotation_Degrees);
        kobuki->rotation_complete = 0;  // clear stale flag from the last rotation
        Rotation_Started = true;
        return false;
    }

    // Keep rotating towards the target
    Kobuki_Rotate_Angle(kobuki->angle);

    // Check if the rotation has finished
    if (kobuki->rotation_complete) {
        Kobuki_Drive(STOP);
        Rotation_Started = false;
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// ORIENT helpers
// ---------------------------------------------------------------------------

// Cancel any rotation step that is still in going
static void Orient_Stop(void) {
    Kobuki_Drive(STOP);
    Rotation_Reset();
}

// Reset the WHOLE centring sub-FSM so it can be called again from scratch
static void Orient_Reset(void) {
    Centre_Found     = false;
    Course_Direction = Course_Undefined;
    Orient_Stage     = ORIENT_FIND_EDGE;
    Orient_Samples   = 0;
    Orient_Cumulative_Angle       = 0;
    Orient_First     = true;
    Rotation_Reset();
}

// 
static void Bumper_Save_Distance_Setpoint(void) {
    Bumper_Resume_Distance = false;

    if (((Robot_State == PATHFIND) &&
         Path_Started &&
         ((Path_Stage == PATH_DRIVE_FIRST) ||
          (Path_Stage == PATH_DRIVE_SECOND) ||
          (Path_Stage == PATH_BACKUP))) ||
        ((Robot_State == STORAGE) && Storage_Started && (Storage_Step == 0U))) {
        Bumper_Resume_Distance_Setpoint = Kobuki_Drive_Distance_Get_Setpoint();
        Bumper_Resume_Distance = true;
    }
}

static void Front_Bumper(Kobuki_Typedef *kobuki) {
    Bumper_Save_Distance_Setpoint();
    Bumper_Action = BUMPER_ACTION_FRONT;
    Bumper_Turn_Angle = 0;
    Bumper_Release_Bumper_Mask = BUMPER_FRONT;
    Bumper_Release_Cliff_Mask = 0U;
    Kobuki_Drive_Distance_Setpoint(kobuki->distance, -FRONT_BUMPER_REVERSE_MM);
    kobuki->distance_complete = 0;
    Bumper_Stage = BUMPER_REVERSE;
}

static void Side_Bumper(
    Kobuki_Typedef *kobuki,
    int16_t Turn_Angle,
    uint8_t Bumper_Mask,
    uint8_t Cliff_Mask)
{
    Bumper_Save_Distance_Setpoint();
    Bumper_Action = BUMPER_ACTION_SIDE;
    Bumper_Turn_Angle = Turn_Angle;
    Bumper_Release_Bumper_Mask = Bumper_Mask;
    Bumper_Release_Cliff_Mask = Cliff_Mask;
    Kobuki_Drive_Distance_Setpoint(kobuki->distance, -SIDE_BUMPER_REVERSE_MM);
    kobuki->distance_complete = 0;
    Bumper_Stage = BUMPER_REVERSE;
}

static bool Bumper_Handle(Kobuki_Typedef *kobuki) {
    if (Bumper_Stage == BUMPER_IDLE) {
        if (((kobuki->bumper & BUMPER_LEFT) != 0U) ||
            ((kobuki->cliff & CLIFF_LEFT) != 0U)) {
            // A negative gyro angle is clockwise.
            Side_Bumper(
                kobuki,
                -SIDE_BUMPER_TURN_DEG,
                BUMPER_LEFT,
                CLIFF_LEFT
            );
        } else if (((kobuki->bumper & BUMPER_RIGHT) != 0U) ||
                   ((kobuki->cliff & CLIFF_RIGHT) != 0U)) {
            Side_Bumper(
                kobuki,
                SIDE_BUMPER_TURN_DEG,
                BUMPER_RIGHT,
                CLIFF_RIGHT
            );
        } else if ((kobuki->bumper & BUMPER_FRONT) != 0U) {
            Front_Bumper(kobuki);
        } else {
            return false;
        }
    }

    switch (Bumper_Stage) {
    case BUMPER_REVERSE:
        Kobuki_Drive_Distance(kobuki->distance);
        if (kobuki->distance_complete) {
            Kobuki_Drive(STOP);
            if (Bumper_Action == BUMPER_ACTION_SIDE) {
                Rotation_Reset();
                Bumper_Stage = BUMPER_TURN;
            } else {
                Bumper_Stage = BUMPER_WAIT_RELEASE;
            }
        }
        break;

    case BUMPER_TURN:
        if (Rotation_Angle(kobuki, Bumper_Turn_Angle)) {
            Bumper_Stage = BUMPER_WAIT_RELEASE;
        }
        break;

    case BUMPER_WAIT_RELEASE:
        Kobuki_Drive(STOP);
        if (((kobuki->bumper & Bumper_Release_Bumper_Mask) == 0U) &&
            ((kobuki->cliff & Bumper_Release_Cliff_Mask) == 0U)) {
            if (Bumper_Resume_Distance) {
                Kobuki_Drive_Distance_Restore_Setpoint(Bumper_Resume_Distance_Setpoint);
                Bumper_Resume_Distance = false;
            }
            Bumper_Stage = BUMPER_IDLE;
            Bumper_Action = BUMPER_ACTION_NONE;
            Bumper_Release_Bumper_Mask = 0U;
            Bumper_Release_Cliff_Mask = 0U;
        }
        break;

    case BUMPER_IDLE:
    default:
        break;
    }

    return true;
}

static bool Emergency_Stop_Check(Kobuki_Typedef *kobuki, bool *Rotation_In_Progress) {
    if ((Robot_State == IDLE) ||
        (((kobuki->button & BUTTON_1_STOP) == 0U) &&
         (kobuki->wheeldrop == 0U))) {
        return false;
    }

    Kobuki_Drive(STOP);
    Robot_State = IDLE;
    *Rotation_In_Progress = false;
    Rotation_Reset();
    Orient_Reset();
    Bumper_Stage = BUMPER_IDLE;
    Bumper_Action = BUMPER_ACTION_NONE;
    Bumper_Turn_Angle = 0;
    Bumper_Resume_Distance = false;
    Bumper_Release_Bumper_Mask = 0U;
    Bumper_Release_Cliff_Mask = 0U;
    Path_Stage = PATH_DRIVE_FIRST;
    Path_Started = false;
    Path_Front_Cliff_Armed = true;
    Storage_Step = 0;
    Storage_Started = false;
    Deposit_Stage = DEPOSIT_SEARCH;
    Deposit_Rotating = false;

    return true;
}

// Lost the bollard / bad reading: stop and go back to SEARCH
static void Orient_Fail(void) {
    Orient_Stop();
    Orient_Reset();
    if (Robot_State == DEPOSIT) {
        Deposit_Stage = DEPOSIT_SEARCH;
    } else {
        Robot_State = SEARCH;
    }
}

// ---------------------------------------------------------------------------
// Bollard_Centre: centre the robot on the bollard.
// Entered after SEARCH detects a bollard, with the bollard inside the sensor beam.
//   1) go Clockwise until the bollard is lost          -> right
//   2) reverse, go CCW until it is Bollard_Seen again
//   3) keep going CCW until it is lost again      -> left
//   4) turn to target = (left + right) / 2
// ---------------------------------------------------------------------------
void Bollard_Centre(Kobuki_Typedef *kobuki, US_Typedef *Ultrasonic_Object) {
    int32_t Angle_Delta; // change in angle since last
    uint16_t Detection_Distance = TARGET_DISTANCE_350_MM; // so usable for SEARCH stage
    if (Robot_State == DEPOSIT) {
        Detection_Distance = DEPOSIT_DETECT_DISTANCE_MM; //so usable for depositing stage
    }
    bool    Bollard_Seen = Detect_Bollard(Ultrasonic_Object, Detection_Distance);

    // First call after a reset: take a reference heading
    if (Orient_First) {
        Orient_Last_Angle = kobuki->angle; // angle in 0.01 deg, wraps at +/-18000
        Orient_Cumulative_Angle        = 0; // cumulative heading change since the start of centring
        Orient_Samples    = 0; // consecutive readings to confirm an edge
        Orient_First      = false; // clear the flag so this only happens once
    }

    // Accumulate the heading change (angle is 0.01 deg and wraps at +/-18000)
    Angle_Delta = (int32_t)kobuki->angle - (int32_t)Orient_Last_Angle;
    if (Angle_Delta > 18000) {
        Angle_Delta -= 36000; // wrap around
    } else if (Angle_Delta < -18000) {
        Angle_Delta += 36000; // wrap around
    }
    Orient_Cumulative_Angle += Angle_Delta; // cumulative heading change since the start of centring
    Orient_Last_Angle = kobuki->angle; // save for the next loop

    switch (Orient_Stage) {
        // -------- 1) CW until the bollard is lost --------
        case ORIENT_FIND_EDGE:
            Rotation_Angle(kobuki, -ORIENT_STEP_DEG);

            if (!Bollard_Seen) {
                if (Orient_Samples == 0U) {
                    Orient_Edge_Angle = Orient_Cumulative_Angle;      // first lost sample
                }
                Orient_Samples++;

                if (Orient_Samples >= ORIENT_CONFIRM_SAMPLES) {
                    Theta_Right    = Orient_Edge_Angle; // save the CW edge
                    Orient_Samples = 0; // reset for the next stage
                    Orient_Stop(); // cancel the CW rotation
                    Orient_Stage   = ORIENT_REENTER; // start CCW to re-enter the bollard
                }
            } else {
                Orient_Samples = 0; // reset if the bollard is seen again
            }

            break;

        // -------- 2) reverse until the bollard is back in the beam --------
        case ORIENT_REENTER:
            Rotation_Angle(kobuki, ORIENT_STEP_DEG);

            if (Bollard_Seen) {
                Orient_Samples++; // count consecutive readings to confirm the bollard is back in view

                if (Orient_Samples >= ORIENT_CONFIRM_SAMPLES) {
                    Orient_Samples = 0; // reset for the next stage
                    Orient_Stage   = ORIENT_FIND_EXIT;  // same direction, keep going
                }
            } else {
                Orient_Samples = 0;
            }

            // Lost it for good
            if ((Orient_Cumulative_Angle - Theta_Right) > ORIENT_FIND_LIMIT_CDEG) {
                Orient_Fail(); // Probably noise: go back to SEARCH
            }
            break;

        // -------- 3) CCW until the bollard is lost again --------
        case ORIENT_FIND_EXIT:
            Rotation_Angle(kobuki, ORIENT_STEP_DEG);

            if (!Bollard_Seen) {
                if (Orient_Samples == 0U) {
                    Orient_Edge_Angle = Orient_Cumulative_Angle;      // first lost sample
                }
                Orient_Samples++; // count consecutive readings to confirm the bollard is lost

                if (Orient_Samples >= ORIENT_CONFIRM_SAMPLES) {
                    Theta_Left     = Orient_Edge_Angle; // save the CCW edge
                    Orient_Samples = 0; // reset for the next stage
                    Orient_Stop();

                    if ((Theta_Left - Theta_Right) < ORIENT_MIN_SPAN_CDEG) {
                        Orient_Fail();                  // noise spike
                    } else {
                        Theta_Target = (Theta_Left + Theta_Right) / 2; // target = middle of the two edges
                        Orient_Stage = ORIENT_RETURN_CENTRE; // turn to the centre
                    }
                }
            } else {
                Orient_Samples = 0; // reset if the bollard is seen again
            }

            break;

        // -------- 4) turn to the centreline --------
        case ORIENT_RETURN_CENTRE:
            // Angle to turn is evaluated every call, but only used when the rotation starts
            if (Rotation_Angle(kobuki, (int16_t)((Theta_Target - Orient_Cumulative_Angle) / 100))) {
                Centre_Found = true; // finished centring
                Orient_Stage = ORIENT_COMPLETE; // go to the final stage
            }
            break;

        case ORIENT_COMPLETE:
            Centre_Found = true; // finished centring
            break;

        default:
            Orient_Stage = ORIENT_FIND_EDGE;
            break;
    }
}




/* ===========================================================================
 * MAIN
 * =========================================================================*/
int main ( void ) {
    // Both of these only for testing purposes . Not required
    bool Rotation_In_Progress = false;
    Kobuki_Typedef kobuki = {0};
    US_Typedef Ultrasonic_Object;
    Kobuki_LED_Enum led_1, led_2;

    UART_Init ();
    ADC_Init ();
    Ultrasonic_Init () ;
    Robot_State = IDLE;
    led_1 = OFF;
    led_2 = OFF;


    while (1) {
        Kobuki_Read (&kobuki );
        Ultrasonic_Detect (&Ultrasonic_Object ) ;
        // Both of these only for testing purposes . Not required
        Kobuki_Update_LEDs ( led_1 , led_2);

        // A TEST FSM
        switch (Robot_State) {

        // ---------------------------------------------------------------
        case IDLE:
            Kobuki_Drive(STOP);
            Kobuki_Update_LEDs(GREEN, GREEN);
            // led_1 = GREEN;
            // led_2 = GREEN;

            if ((kobuki.button & BUTTON_0_START) != 0U) {
                Robot_State = SEARCH;
            }
            break;

        // ---------------------------------------------------------------
        case SEARCH:
            Kobuki_Update_LEDs(RED, OFF);

            if (Emergency_Stop_Check(&kobuki, &Rotation_In_Progress)) {
                Robot_State = IDLE;
                break;
            }

            if (Bumper_Handle(&kobuki)) {
                break;
            }

            // If currently rotating, continue until rotation completes
            if (Rotation_In_Progress) {
                if (Rotation_Angle(&kobuki, SEARCH_ROTATION_INCREMENT_DEG)) {
                    Rotation_In_Progress = false;
                }

                break;
            }

            // Only check the sensor when stationary
            if (Detect_Bollard(&Ultrasonic_Object, TARGET_DISTANCE_350_MM)) {
                // Bollard found: hand straight over to ORIENT to centre on it
                Kobuki_Drive(STOP);
                Rotation_In_Progress = false;
                Rotation_Reset();
                Orient_Reset();
                Robot_State = ORIENT;
                break;
            }

            // Nothing in range: rotate another 11 deg and look again
            Rotation_In_Progress = true;
            Rotation_Reset();
            break;

        // ---------------------------------------------------------------
        case ORIENT:
            Kobuki_Update_LEDs(YELLOW, OFF);
            if (Emergency_Stop_Check(&kobuki, &Rotation_In_Progress)) {
                Robot_State = IDLE;
                break;
            }

            if (Bumper_Handle(&kobuki)) {
                break;
            }
            

            if (!Centre_Found) {
                // Step 1: centre on the bollard
                Bollard_Centre(&kobuki, &Ultrasonic_Object);
            } else {
                // Step 2: decide the course once centring is complete.
                if (Course_Direction == Course_Undefined) {
                    if (Detect_Bollard(&Ultrasonic_Object, TARGET_DISTANCE_150_MM)) {
                        Course_Direction = Course_Left;
                    } else {
                        Course_Direction = Course_Right;
                    }
                    // A bollard within 150 mm selects the left route; otherwise, the right route is selected.

                    Rotation_Reset();
                }

                // Step 3: rotate 90 deg (called every loop until it finishes)
                if (Course_Direction == Course_Left) {
                    if (Rotation_Angle(&kobuki, 89)) {
                        Robot_State = PATHFIND;
                    }
                } else {
                    if (Rotation_Angle(&kobuki, -89)) {
                        Robot_State = PATHFIND;
                    }
                }
            }
            break;

        // ---------------------------------------------------------------
        case PATHFIND:
            Kobuki_Update_LEDs(RED, RED);
            if (Emergency_Stop_Check(&kobuki, &Rotation_In_Progress)) {
                Robot_State = IDLE;
                break;
            }

            if (Bumper_Handle(&kobuki)) {
                break;
            }

            if ((kobuki.cliff & CLIFF_FRONT) == 0U) {
                Path_Front_Cliff_Armed = true;
            } else if (Path_Front_Cliff_Armed &&((Path_Stage == PATH_DRIVE_FIRST) ||(Path_Stage == PATH_DRIVE_SECOND))) {
                Kobuki_Drive(STOP);
                Path_Started = false;
                Path_Front_Cliff_Armed = false;

                if (Path_Stage == PATH_DRIVE_FIRST) {
                    Path_After_Backup = PATH_TURN_90;
                } else {
                    Path_After_Backup = PATH_FACE_STORAGE;
                }
                Path_Stage = PATH_BACKUP;
            }

            switch (Path_Stage) {
            case PATH_DRIVE_FIRST:
            case PATH_DRIVE_SECOND:
                // Cliff detection above selects the backup and subsequent turn.
                Kobuki_Drive(FORWARD_SPEED_MM_S);
                break;

            case PATH_BACKUP:
                // Shared 100 mm backup after front cliff detection.
                if (!Path_Started) {
                    Kobuki_Drive_Distance_Setpoint(kobuki.distance, -100);
                    kobuki.distance_complete = 0;
                    Path_Started = true;
                }

                Kobuki_Drive_Distance(kobuki.distance);

                if (kobuki.distance_complete) {
                    Kobuki_Drive(STOP);
                    Path_Started = false;
                    Path_Stage = Path_After_Backup;
                }
                break;

            case PATH_TURN_90:
                // 90 degree rotation depending on the course direction (left or right) .
                if (Course_Direction == Course_Left) {
                    if (Rotation_Angle(&kobuki, -89)) {
                        Path_Stage = PATH_DRIVE_SECOND;
                    }
                } else if (Course_Direction == Course_Right) {
                    if (Rotation_Angle(&kobuki, 89)) {
                        Path_Stage = PATH_DRIVE_SECOND;
                    }
                }
                break;

            case PATH_FACE_STORAGE:
                // 135 degree rotation to face storage area
                if (Course_Direction == Course_Left) {
                    if (Rotation_Angle(&kobuki, -133)) {
                        Path_Stage = PATH_COMPLETE;
                    }
                } else if (Course_Direction == Course_Right) {
                    if (Rotation_Angle(&kobuki, 133)) {
                        Path_Stage = PATH_COMPLETE;
                    }
                }
                break;

            case PATH_COMPLETE:
                // Finished path
                Kobuki_Drive(STOP);
                Path_Stage = PATH_DRIVE_FIRST;
                Path_Started = false;
                Path_Front_Cliff_Armed = true;
                Robot_State = STORAGE;
                break;

            default:
                Kobuki_Drive(STOP);
                Path_Stage = PATH_DRIVE_FIRST;
                Path_Started = false;
                break;
            }
            break;

        // ---------------------------------------------------------------
        case STORAGE:
            Kobuki_Update_LEDs(YELLOW, RED);
            if (Emergency_Stop_Check(&kobuki, &Rotation_In_Progress)) {
                break;
            }

            if (Bumper_Handle(&kobuki)) {
                break;
            }

            if (Storage_Step == 0) {
                // Drive into the storage area
                if (!Storage_Started) {
                    Kobuki_Drive_Distance_Setpoint(kobuki.distance, 850);
                    kobuki.distance_complete = 0;
                    Storage_Started = true;
                }

                Kobuki_Drive_Distance(kobuki.distance);

                if (kobuki.distance_complete) {
                    Kobuki_Drive(STOP);
                    Storage_Started = false;
                    Storage_Step = 1;
                }
            }
            else if (Storage_Step == 1) {
                // Finished entering storage
                Storage_Step = 0;
                Deposit_Stage = DEPOSIT_SEARCH;
                Deposit_Rotating = false;
                Orient_Reset();
                Robot_State = DEPOSIT;
            }

            break;
        // ---------------------------------------------------------------
        case DEPOSIT:
            Kobuki_Update_LEDs(RED, YELLOW);
            if (Emergency_Stop_Check(&kobuki, &Rotation_In_Progress)) {
                break;
            }

            if (Bumper_Handle(&kobuki)) {
                break;
            }

            switch (Deposit_Stage) {
                case DEPOSIT_SEARCH:
                    // Scan in steps so each reading is taken while stationary.
                    if (Deposit_Rotating) {
                        if (Rotation_Angle(&kobuki, SEARCH_ROTATION_INCREMENT_DEG)) {
                            Deposit_Rotating = false;
                        }
                        break;
                    }

                    if (Detect_Bollard(&Ultrasonic_Object, DEPOSIT_DETECT_DISTANCE_MM)) {
                        Orient_Stop();
                        Orient_Reset();
                        Deposit_Stage = DEPOSIT_CENTRE;
                        Centre_Found = false;
                    } else {
                        Deposit_Rotating = true;
                        Rotation_Reset();
                    }
                    break;

                case DEPOSIT_CENTRE:
                    Bollard_Centre(&kobuki, &Ultrasonic_Object);
                    if (Centre_Found) {
                        Deposit_Stage = DEPOSIT_APPROACH;
                    }

                    break;

                case DEPOSIT_APPROACH:
                    Ultrasonic_Detect(&Ultrasonic_Object);
                    if (Ultrasonic_Object.range_mm > DEPOSIT_STOP_DISTANCE_MM) {
                        Kobuki_Drive(DEPOSIT_APPROACH_SPEED_MM_S);
                    } else {
                        Kobuki_Drive(STOP);
                        Deposit_Stage = DEPOSIT_COMPLETE;
                    }
                    break;

                case DEPOSIT_COMPLETE:
                    Kobuki_Drive(STOP);
                    Robot_State = IDLE;
                    break;

                default:
                    Kobuki_Drive(STOP);
                    break;
            }
            break;
        }
    }
}
