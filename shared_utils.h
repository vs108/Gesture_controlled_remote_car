/*
 * Shared Utilities for Gesture-Controlled 4WD RC Car
 * Common definitions and functions for both controller and car
 */

#ifndef SHARED_UTILS_H
#define SHARED_UTILS_H

#include <avr/io.h>
#include <stdint.h>

// Common Command Protocol
#define CMD_FORWARD     'F'
#define CMD_BACKWARD    'B'
#define CMD_LEFT        'L'
#define CMD_RIGHT       'R'
#define CMD_STOP        'S'

// Speed Levels
#define SPEED_LOW       '1'
#define SPEED_MEDIUM    '2'
#define SPEED_HIGH      '3'

// UART Configuration
#define UART_BAUD_RATE  9600
#define UART_UBRR_VALUE 0x67  // For 16MHz crystal

// Common Pin Definitions
#define UART_TX_PIN     PD1
#define UART_RX_PIN     PD0

// Status Codes
#define STATUS_OK       0x00
#define STATUS_ERROR    0x01
#define STATUS_TIMEOUT  0x02

// Timeout values
#define UART_TIMEOUT_MS 100
#define I2C_TIMEOUT_MS  50

// Function Prototypes
void initUART_common(void);
void UART_send_common(uint8_t data);
void UART_sendString_common(const char* str);
uint8_t UART_receive_common(void);
uint8_t UART_receiveWithTimeout(uint16_t timeout_ms);
void delay_ms(uint16_t ms);
uint8_t isValidCommand(uint8_t direction, uint8_t speed);
const char* getCommandString(uint8_t direction, uint8_t speed);

/*
 * Initialize UART with common settings
 */
inline void initUART_common(void) {
    // Set baud rate
    UBRRH = 0x00;
    UBRRL = UART_UBRR_VALUE;
    
    // Enable transmitter and receiver
    UCSRB = (1 << TXEN) | (1 << RXEN);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

/*
 * Send single byte via UART
 */
inline void UART_send_common(uint8_t data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

/*
 * Send string via UART
 */
inline void UART_sendString_common(const char* str) {
    while (*str) {
        UART_send_common(*str++);
    }
}

/*
 * Receive single byte via UART
 */
inline uint8_t UART_receive_common(void) {
    while (!(UCSRA & (1 << RXC)));
    return UDR;
}

/*
 * Receive byte with timeout
 */
uint8_t UART_receiveWithTimeout(uint16_t timeout_ms) {
    uint16_t count = 0;
    while (!(UCSRA & (1 << RXC)) && count < timeout_ms) {
        delay_ms(1);
        count++;
    }
    
    if (count >= timeout_ms) {
        return STATUS_TIMEOUT;
    }
    
    return UDR;
}

/*
 * Simple delay function
 */
inline void delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        for (volatile uint16_t j = 0; j < 1600; j++); // Approximate for 16MHz
    }
}

/*
 * Validate command parameters
 */
inline uint8_t isValidCommand(uint8_t direction, uint8_t speed) {
    // Check direction
    if (direction != CMD_FORWARD && direction != CMD_BACKWARD && 
        direction != CMD_LEFT && direction != CMD_RIGHT && 
        direction != CMD_STOP) {
        return 0;
    }
    
    // Check speed
    if (speed != SPEED_LOW && speed != SPEED_MEDIUM && speed != SPEED_HIGH) {
        return 0;
    }
    
    return 1;
}

/*
 * Get string representation of command
 */
const char* getCommandString(uint8_t direction, uint8_t speed) {
    static char cmd_str[10];
    
    switch(direction) {
        case CMD_FORWARD:
            sprintf(cmd_str, "FORWARD_%c", speed);
            break;
        case CMD_BACKWARD:
            sprintf(cmd_str, "BACKWARD_%c", speed);
            break;
        case CMD_LEFT:
            sprintf(cmd_str, "LEFT_%c", speed);
            break;
        case CMD_RIGHT:
            sprintf(cmd_str, "RIGHT_%c", speed);
            break;
        case CMD_STOP:
            sprintf(cmd_str, "STOP");
            break;
        default:
            sprintf(cmd_str, "UNKNOWN");
            break;
    }
    
    return cmd_str;
}

#endif // SHARED_UTILS_H 
