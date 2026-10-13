#include <stdint.h>
#include <stdlib.h>

#define STM32G474xx

#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"

// Movement setpoints
static volatile int32_t setpoint_distance_tick = 0;
static volatile int16_t setpoint_angle_deg = 0;

// Movement completion flags
static uint8_t rotation_complete = 0;
static uint8_t distance_complete = 0;


// ============================================================
// Kobuki communication
// ============================================================

void Kobuki_Tx(uint8_t *payload, uint8_t size_payload)
{
    uint8_t checksum = 0;
    uint8_t i;

    UART_Tx(0xAA);
    UART_Tx(0x55);
    UART_Tx(size_payload);

    checksum ^= size_payload;

    for (i = 0; i < size_payload; i++)
    {
        UART_Tx(payload[i]);
        checksum ^= payload[i];
    }

    UART_Tx(checksum);
}


uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback)
{
    uint8_t feedback_header = 0;
    uint8_t size_payload = 0;
    uint8_t checksum = 0;
    uint8_t rx_checksum = 0;

    uint32_t i;

    // Find packet header: 0xAA 0x55
    while (1)
    {
        feedback_header = UART_Rx();

        if (feedback_header == 0xAA)
        {
            feedback_header = UART_Rx();

            if (feedback_header == 0x55)
            {
                break;
            }
        }
    }

    // Read payload size
    size_payload = UART_Rx();
    checksum ^= size_payload;

    // Avoid writing beyond the feedback buffer
    uint32_t limit = (size_payload < size_feedback)
                   ? size_payload
                   : size_feedback;

    // Read payload into buffer
    for (i = 0; i < limit; i++)
    {
        feedback[i] = UART_Rx();
        checksum ^= feedback[i];
    }

    // Consume any remaining payload bytes
    for (i = limit; i < size_payload; i++)
    {
        checksum ^= UART_Rx();
    }

    // Read and validate checksum
    rx_checksum = UART_Rx();

    return (checksum != rx_checksum);
}


// ============================================================
// Read Kobuki feedback
// ============================================================

void Kobuki_Read(Kobuki_Typedef *kobuki)
{
    uint8_t kobuki_rx_buffer[SIZE_FEEDBACK];
    uint8_t checksum_result;

    checksum_result = Kobuki_Rx(
        kobuki_rx_buffer,
        SIZE_FEEDBACK
    );

    if (checksum_result == 0)
    {
        // Read bumper, wheel drop, cliff and button feedback
        kobuki->bumper = kobuki_rx_buffer[BUMPER];
        kobuki->wheeldrop = kobuki_rx_buffer[WHEEL_DROP];
        kobuki->cliff = kobuki_rx_buffer[CLIFF];
        kobuki->button = kobuki_rx_buffer[BUTTON];

        // Combine distance bytes into signed 16-bit ticks
        kobuki->distance = (int16_t)(
            (uint16_t)kobuki_rx_buffer[DIST_LSB] |
            ((uint16_t)kobuki_rx_buffer[DIST_MSB] << 8)
        );

        // Combine angle bytes into signed 16-bit ticks
        kobuki->angle = (int16_t)(
            (uint16_t)kobuki_rx_buffer[ANGLE_LSB] |
            ((uint16_t)kobuki_rx_buffer[ANGLE_MSB] << 8)
        );
    }

    // Always expose the current movement flags
    kobuki->rotation_complete = rotation_complete;
    kobuki->distance_complete = distance_complete;
}


// ============================================================
// Basic driving functions
// ============================================================

void Kobuki_Drive(int16_t speed)
{
    uint8_t payload[] = {
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00
    };

    // Limit speed
    if (speed > SPEED_LIMIT)
    {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT)
    {
        speed = -SPEED_LIMIT;
    }

    // Split speed into low and high bytes
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}


void Kobuki_Rotate(int16_t speed)
{
    uint8_t payload[] = {
        0x01, 0x04, 0x00, 0x00, 0x01, 0x00
    };

    // Limit speed
    if (speed > SPEED_LIMIT)
    {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT)
    {
        speed = -SPEED_LIMIT;
    }

    // Split speed into low and high bytes
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}


// ============================================================
// Rotation by angle
// ============================================================

void Kobuki_Rotate_Angle_Setpoint(
    int16_t current_angle_tick,
    int16_t rot_angle_deg)
{
    // Convert requested degrees to Kobuki angle ticks.
    // DEG_PER_TICK is defined in kobuki.h.
    setpoint_angle_deg = (int16_t)(
        current_angle_tick +
        (rot_angle_deg / DEG_PER_TICK) * GAIN_ROTATE
    );

    rotation_complete = 0;
}


void Kobuki_Rotate_Angle(int16_t current_angle_tick)
{
    int16_t error;

    error = setpoint_angle_deg - current_angle_tick;

    if (abs(error) <= 1)
    {
        Kobuki_Rotate(STOP);
        rotation_complete = 1;
    }
    else
    {
        rotation_complete = 0;

        if (error > 0)
        {
            Kobuki_Rotate(MIN_SPEED);
        }
        else
        {
            Kobuki_Rotate(-MIN_SPEED);
        }
    }
}


// ============================================================
// Driving by distance
// ============================================================

void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance)
{
    // Convert requested distance in mm to distance ticks.
    setpoint_distance_tick = (int32_t)current_distance_tick
                           + (int32_t)(
                               distance / MM_PER_TICK
                               * GAIN_DRIVE
                             );

    distance_complete = 0;
}


void Kobuki_Drive_Distance(uint16_t current_distance_tick)
{
    int32_t error;

    error = setpoint_distance_tick - (int32_t)current_distance_tick;

    if (abs(error) <= 1)
    {
        Kobuki_Drive(STOP);
        distance_complete = 1;
    }
    else
    {
        distance_complete = 0;

        if (error > 0)
        {
            Kobuki_Drive(MIN_SPEED);
        }
        else
        {
            Kobuki_Drive(-MIN_SPEED);
        }
    }
}


// ============================================================
// Kobuki LED control
// ============================================================

void Kobuki_Update_LEDs(
    Kobuki_LED_Enum led_1_colour,
    Kobuki_LED_Enum led_2_colour)
{
    uint8_t payload[] = {
        0x0B, 0x03, 0x00
    };

    // LED values must match the enum definitions in kobuki.h.
    payload[2] = (uint8_t)(
        ((uint8_t)led_1_colour & 0x03U) |
        (((uint8_t)led_2_colour & 0x03U) << 2)
    );

    Kobuki_Tx(payload, sizeof(payload));
}