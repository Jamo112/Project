#define STM32G474xx
#include "kobuki.h"
#include "uart.h"
#include "stm32g4xx.h"
#include <stdlib.h>
#include <stdint.h>


/* External power control */
#define EXTERN_PWR 0x30

/* Movement setpoints */
static volatile int32_t setpoint_distance_tick;
static volatile int16_t setpoint_angle_deg;

/* Movement completion flags */
static uint8_t rotation_complete = 0;
static uint8_t distance_complete = 0;


/* =========================================================
 * UART TRANSMISSION
 * ========================================================= */

void Kobuki_Tx(uint8_t *payload, uint8_t size_payload)
{
    uint8_t header[3] = {0xAA, 0x55, size_payload};
    uint8_t checksum = size_payload;

    /* Calculate XOR checksum */
    for (uint8_t i = 0; i < size_payload; i++) {
        checksum ^= payload[i];
    }

    /* Send packet header */
    for (uint8_t i = 0; i < sizeof(header); i++) {
        UART_Tx(header[i]);
    }

    /* Send payload */
    for (uint8_t i = 0; i < size_payload; i++) {
        UART_Tx(payload[i]);
    }

    /* Send checksum */
    UART_Tx(checksum);
}


/* =========================================================
 * UART RECEPTION
 * ========================================================= */

uint8_t Kobuki_Rx(uint8_t *feedback, uint8_t size_feedback)
{
    uint8_t header[3];
    uint8_t checksum_received;
    uint8_t checksum_calculated;
    uint8_t size_payload;
    uint8_t bytes_to_copy;

    /* Find packet header */
    do {
        header[0] = UART_Rx();
    } while (header[0] != 0xAA);

    /* Receive second header byte */
    header[1] = UART_Rx();

    if (header[1] != 0x55) {
        return 1;
    }

    /* Receive payload size */
    header[2] = UART_Rx();

    size_payload = header[2];
    checksum_calculated = size_payload;

    /* Clear feedback buffer */
    for (uint8_t i = 0; i < size_feedback; i++) {
        feedback[i] = 0;
    }

    /* Copy only as many bytes as fit in the supplied buffer */
    bytes_to_copy = size_payload;

    if (bytes_to_copy > size_feedback) {
        bytes_to_copy = size_feedback;
    }

    /* Receive payload bytes */
    for (uint8_t i = 0; i < size_payload; i++) {
        uint8_t received_byte;

        received_byte = UART_Rx();

        checksum_calculated ^= received_byte;

        if (i < bytes_to_copy) {
            feedback[i] = received_byte;
        }
    }

    /* Receive checksum */
    checksum_received = UART_Rx();

    /* Return non-zero if checksum is incorrect */
    if (checksum_calculated != checksum_received) {
        return 1;
    }

    /* Extra sensor data may follow the fields copied into feedback. */
    if (size_payload < size_feedback) {
        return 1;
    }

    return 0;
}


/* =========================================================
 * READ KOBUKI FEEDBACK
 * ========================================================= */

void Kobuki_Read(Kobuki_Typedef *kobuki)
{
    uint8_t kobuki_rx_buffer[SIZE_FEEDBACK];
    uint8_t checksum_result;

    uint16_t distance_lsb;
    uint16_t distance_msb;

    uint16_t angle_lsb;
    uint16_t angle_msb;

    checksum_result = Kobuki_Rx(
        kobuki_rx_buffer,
        SIZE_FEEDBACK
    );

    if (checksum_result == 0) {

        /* Read distance bytes */
        distance_lsb = kobuki_rx_buffer[DIST_LSB];

        distance_msb =
            (uint16_t)kobuki_rx_buffer[DIST_MSB] << 8;

        kobuki->distance =
            (uint16_t)(distance_lsb | distance_msb);

        /* Read angle bytes */
        angle_lsb = kobuki_rx_buffer[ANGLE_LSB];

        angle_msb =
            (uint16_t)kobuki_rx_buffer[ANGLE_MSB] << 8;

        kobuki->angle =
            (int16_t)(angle_lsb | angle_msb);

        /* Read sensor information */
        kobuki->bumper = kobuki_rx_buffer[BUMPER];
        kobuki->wheeldrop = kobuki_rx_buffer[WHEEL_DROP];
        kobuki->cliff = kobuki_rx_buffer[CLIFF];
        kobuki->button = kobuki_rx_buffer[BUTTON];
    }

    /* Update movement completion flags */
    kobuki->rotation_complete = rotation_complete;
    kobuki->distance_complete = distance_complete;
}


/* =========================================================
 * BASIC DRIVE
 * ========================================================= */

void Kobuki_Drive(int16_t speed)
{
    uint8_t payload[] = {
        0x01, 0x04,
        0x00, 0x00,
        0x00, 0x00
    };

    /* Limit forward and reverse speeds */
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = -SPEED_LIMIT;
    }

    /* Encode speed as a 16-bit signed value */
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}


/* =========================================================
 * BASIC ROTATE
 * ========================================================= */

void Kobuki_Rotate(int16_t speed)
{
    uint8_t payload[] = {
        0x01, 0x04,
        0x00, 0x00,
        0x01, 0x00
    };

    /* Limit rotation speed */
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = -SPEED_LIMIT;
    }

    /* Encode speed as a 16-bit signed value */
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}


/* =========================================================
 * SET DISTANCE SETPOINT
 * ========================================================= */

void Kobuki_Drive_Distance_Setpoint(
    uint16_t current_distance_tick,
    int16_t distance)
{
    distance_complete = 0;

    setpoint_distance_tick =
        (int32_t)current_distance_tick
        + (int32_t)(distance / MM_PER_TICK);
}


/* =========================================================
 * DRIVE TO DISTANCE SETPOINT
 * ========================================================= */

void Kobuki_Drive_Distance(uint16_t current_distance_tick)
{
    volatile int16_t speed;
    volatile int16_t error;
    volatile uint16_t setpoint_alias;

    /* Calculate distance remaining in millimetres */
    error = (int16_t)(
        (double)(
            setpoint_distance_tick
            - current_distance_tick
        ) * MM_PER_TICK
    );

    /*
     * Handle wraparound of the 16-bit distance counter.
     */
    setpoint_alias = (uint16_t)(
        (setpoint_distance_tick + 0x10000) % 0x10000
    );

    if (
        (
            (setpoint_distance_tick > 0xFFFF)
            && (current_distance_tick < setpoint_alias)
        )
        ||
        (
            (setpoint_distance_tick < 0)
            && (current_distance_tick > setpoint_alias)
        )
    ) {
        error = (int16_t)(
            (double)(
                (int32_t)setpoint_alias
                - current_distance_tick
            ) * MM_PER_TICK
        );
    }

    /* Stop when sufficiently close to the setpoint */
    if (abs(error) <= 2) {
        distance_complete = 1;
        Kobuki_Drive(STOP);
    } else {
        speed =
            (sign(error) * MIN_SPEED)
            + (int16_t)(GAIN_DRIVE * error);

        Kobuki_Drive(speed);
    }
}


/* =========================================================
 * SET ROTATION ANGLE SETPOINT
 * ========================================================= */

void Kobuki_Rotate_Angle_Setpoint(
    int16_t current_angle_tick,
    int16_t rot_angle_deg)
{
    volatile int32_t init_angle;

    rotation_complete = 0;

    /*
     * Convert the current angle from [-18000, 18000]
     * ticks to [0, 360) degrees.
     */
    init_angle = (int32_t)(
        (double)(
            current_angle_tick + (int32_t)18000
        ) * DEG_PER_TICK
    );

    /* Calculate the target angle */
    setpoint_angle_deg =
        (int16_t)(
            (init_angle + rot_angle_deg + 360) % 360
        );
}


/* =========================================================
 * ROTATE TO ANGLE SETPOINT
 * ========================================================= */

void Kobuki_Rotate_Angle(int16_t current_angle_tick)
{
    volatile int16_t speed;
    volatile int16_t error;
    volatile int32_t current_angle_deg;

    /* Convert current angle to degrees */
    current_angle_deg = (int32_t)(
        (double)(
            current_angle_tick + (int32_t)18000
        ) * DEG_PER_TICK
    );

    /* Calculate angular error */
    error = (int16_t)(
        setpoint_angle_deg - current_angle_deg
    );

    /*
     * Choose the shorter rotation direction when
     * the angular difference exceeds 180 degrees.
     */
    if (abs(error) > 180) {

        error = -sign(error) * (abs(error) % 180);

        if (
            (error < 0)
            && (current_angle_deg < setpoint_angle_deg)
        ) {
            error = -(error + 180);
        } else if (
            (error > 0)
            && (current_angle_deg > setpoint_angle_deg)
        ) {
            error = -(error - 180);
        }
    }

    /* Stop when the target angle is reached */
    if (abs(error) <= 0) {
        rotation_complete = 1;
        Kobuki_Rotate(STOP);
    } else {
        speed =
            (sign(error) * MIN_SPEED)
            + (int16_t)(GAIN_ROTATE * error);

        Kobuki_Rotate(speed);
    }
}


/* =========================================================
 * UPDATE KOBUKI LEDS
 * ========================================================= */

void Kobuki_Update_LEDs(
    Kobuki_LED_Enum led_1_colour,
    Kobuki_LED_Enum led_2_colour)
{
    uint8_t GPO_lsb = EXTERN_PWR;

    uint8_t GPO_msb =
        (uint8_t)(
            led_1_colour
            | (uint8_t)(led_2_colour << 2)
        );

    uint8_t payload[] = {
        0x0C,
        0x02,
        GPO_lsb,
        GPO_msb
    };

    Kobuki_Tx(payload, sizeof(payload));
}
