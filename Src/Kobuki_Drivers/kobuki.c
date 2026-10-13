#include <stdint.h>

#define STM32G474xx

#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"


void Kobuki_Tx(uint8_t *payload, uint8_t size_payload)
{
    uint8_t checksum = 0;
    uint8_t i;

    // Send header 0
    UART_Tx(0xAA);

    // Send header 1
    UART_Tx(0x55);

    // Send payload size
    UART_Tx(size_payload);

    // Calculate running checksum
    checksum ^= size_payload;

    // Send payload bytes one by one
    for (i = 0; i < size_payload; i++)
    {
        UART_Tx(payload[i]);

        checksum ^= payload[i];
    }

    // Send checksum
    UART_Tx(checksum);
}


uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback)
{
    uint8_t feedback_header = 0;
    uint8_t size_payload = 0;
    uint8_t checksum = 0;
    uint8_t rx_checksum = 0;

    uint32_t i;

    // 1. Find valid 0xAA 0x55 sequence
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

    // 2. Read payload length
    size_payload = UART_Rx();

    // Include payload length in checksum
    checksum ^= size_payload;

    // 3. Determine safe boundary
    uint32_t limit = (size_payload < size_feedback)
                   ? size_payload
                   : size_feedback;

    // 4. Read payload into feedback buffer
    for (i = 0; i < limit; i++)
    {
        feedback[i] = UART_Rx();

        checksum ^= feedback[i];
    }

    // 5. Read remaining payload bytes
    for (i = limit; i < size_payload; i++)
    {
        checksum ^= UART_Rx();
    }

    // 6. Receive checksum
    rx_checksum = UART_Rx();

    // 0 = success
    // 1 = error
    return (checksum != rx_checksum);
}


void Kobuki_Read(Kobuki_Typedef *kobuki)
{
    // Initialise buffer
    uint8_t kobuki_rx_buffer[SIZE_FEEDBACK];

    uint8_t checksum_result;

    // Read Kobuki Tx data stream
    checksum_result = Kobuki_Rx(
        kobuki_rx_buffer,
        SIZE_FEEDBACK
    );

    // If checksum checks out
    if (checksum_result == 0)
    {
        kobuki->bumper =
            kobuki_rx_buffer[BUMPER];

        kobuki->wheeldrop =
            kobuki_rx_buffer[WHEEL_DROP];

        kobuki->cliff =
            kobuki_rx_buffer[CLIFF];

        kobuki->button =
            kobuki_rx_buffer[BUTTON];
    }

    // Otherwise don't update values
    else
    {
    }
}

void Kobuki_Drive ( int16_t speed ) {
    uint8_t speed_lsb = 0;
    uint8_t speed_msb = 0;
     // Construct payload
    uint8_t payload [] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00};

    // saturation to avoid going over speed limit
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    // Split LSB & MSB for payload
    payload[2] = (uint8_t) speed;
    payload[3] = (uint8_t) (speed >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate (int16_t speed) {
    uint8_t speed_lsb = 0;
    uint8_t speed_msb = 0;
    // Construct payload
    uint8_t payload[] = {0x01, 0x04, 0x00, 0x00, 0x01, 0x00};

    // saturation to avoid going over speed limit
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    // split lsb & msb for payload
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}