/*
 * Gesture-Controlled 4WD RC Car
 * ATmega16 microcontroller with L293D motor driver and HC-05 Bluetooth
 * 
 * Hardware Connections:
 * - HC-05: TX -> PD0, RX -> PD1 (UART)
 * - L293D Motor Driver:
 *   - Enable 1: PB0 (PWM for left motors)
 *   - Enable 2: PB1 (PWM for right motors)
 *   - Input 1: PC0 (Left motor 1 direction)
 *   - Input 2: PC1 (Left motor 1 direction)
 *   - Input 3: PC2 (Left motor 2 direction)
 *   - Input 4: PC3 (Left motor 2 direction)
 *   - Input 5: PC4 (Right motor 1 direction)
 *   - Input 6: PC5 (Right motor 1 direction)
 *   - Input 7: PC6 (Right motor 2 direction)
 *   - Input 8: PC7 (Right motor 2 direction)
 * - Status LED: PB2
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Command Protocol
#define CMD_FORWARD     'F'
#define CMD_BACKWARD    'B'
#define CMD_LEFT        'L'
#define CMD_RIGHT       'R'
#define CMD_STOP        'S'

// Speed Levels
#define SPEED_LOW       '1'
#define SPEED_MEDIUM    '2'
#define SPEED_HIGH      '3'

// PWM Values for different speeds
#define PWM_LOW         102  // ~40% duty cycle
#define PWM_MEDIUM      179  // ~70% duty cycle
#define PWM_HIGH        255  // ~100% duty cycle

// Motor Control Pins
#define LEFT_ENABLE     PB0   // Enable 1 for left motors
#define RIGHT_ENABLE    PB1   // Enable 2 for right motors
#define STATUS_LED      PB2   // Status indicator

// Left Motors (Motor 1 & 2)
#define LEFT_MOTOR1_IN1 PC0   // Input 1
#define LEFT_MOTOR1_IN2 PC1   // Input 2
#define LEFT_MOTOR2_IN1 PC2   // Input 3
#define LEFT_MOTOR2_IN2 PC3   // Input 4

// Right Motors (Motor 3 & 4)
#define RIGHT_MOTOR1_IN1 PC4  // Input 5
#define RIGHT_MOTOR1_IN2 PC5  // Input 6
#define RIGHT_MOTOR2_IN1 PC6  // Input 7
#define RIGHT_MOTOR2_IN2 PC7  // Input 8

// Global Variables
volatile uint8_t current_direction = CMD_STOP;
volatile uint8_t current_speed = SPEED_LOW;
volatile uint8_t command_received = 0;

// Function Prototypes
void initSystem(void);
void initUART(void);
void initPWM(void);
void UART_send(uint8_t data);
void UART_sendString(const char* str);
uint8_t UART_receive(void);
void processCommand(uint8_t direction, uint8_t speed);
void setMotorSpeed(uint8_t left_speed, uint8_t right_speed);
void setPWM(uint8_t channel, uint8_t value);
void forward(uint8_t speed);
void backward(uint8_t speed);
void left(uint8_t speed);
void right(uint8_t speed);
void stop(void);
uint8_t speedToPWM(uint8_t speed_level);
void updateStatusLED(uint8_t direction);

int main(void) {
    // Initialize all systems
    initSystem();
    initUART();
    initPWM();
    
    // Enable global interrupts
    sei();
    
    // Send startup message
    UART_sendString("Car Ready\r\n");
    
    // Stop motors initially
    stop();
    
    // Main control loop
    while(1) {
        // Check if command received
        if (command_received) {
            processCommand(current_direction, current_speed);
            command_received = 0;
        }
        
        // Small delay
        _delay_ms(10);
    }
    
    return 0;
}

/*
 * Initialize system ports and basic settings
 */
void initSystem(void) {
    // Set motor control pins as output
    DDRB |= (1 << LEFT_ENABLE) | (1 << RIGHT_ENABLE) | (1 << STATUS_LED);
    DDRC |= (1 << LEFT_MOTOR1_IN1) | (1 << LEFT_MOTOR1_IN2) | 
            (1 << LEFT_MOTOR2_IN1) | (1 << LEFT_MOTOR2_IN2) |
            (1 << RIGHT_MOTOR1_IN1) | (1 << RIGHT_MOTOR1_IN2) |
            (1 << RIGHT_MOTOR2_IN1) | (1 << RIGHT_MOTOR2_IN2);
    
    // Initialize all pins to low
    PORTB = 0x00;
    PORTC = 0x00;
    
    // Set UART pins (PD0-RX, PD1-TX)
    DDRD |= (1 << PD1);  // TX as output
    DDRD &= ~(1 << PD0); // RX as input
}

/*
 * Initialize UART for HC-05 Bluetooth communication
 * Baud rate: 9600, 8 data bits, 1 stop bit, no parity
 */
void initUART(void) {
    // Set baud rate (9600 @ 16MHz)
    UBRRH = 0x00;
    UBRRL = 0x67;
    
    // Enable transmitter and receiver with interrupt
    UCSRB = (1 << TXEN) | (1 << RXEN) | (1 << RXCIE);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

/*
 * Initialize PWM for motor speed control
 * Using Timer1 for PWM generation
 */
void initPWM(void) {
    // Set Timer1 for Fast PWM mode with ICR1 as TOP
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    
    // Set PWM frequency to ~1kHz (16MHz / 16000)
    ICR1 = 15999;
    
    // Initialize PWM outputs to 0
    OCR1A = 0; // Left motors
    OCR1B = 0; // Right motors
}

/*
 * Send single byte via UART
 */
void UART_send(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSRA & (1 << UDRE)));
    
    // Put data into buffer
    UDR = data;
}

/*
 * Send string via UART
 */
void UART_sendString(const char* str) {
    while (*str) {
        UART_send(*str++);
    }
}

/*
 * Receive single byte via UART
 */
uint8_t UART_receive(void) {
    // Wait for data to be received
    while (!(UCSRA & (1 << RXC)));
    
    // Return received data
    return UDR;
}

/*
 * UART Receive Complete Interrupt Service Routine
 * Handles incoming commands from controller
 */
ISR(USART_RXC_vect) {
    static uint8_t command_buffer[3];
    static uint8_t buffer_index = 0;
    
    uint8_t received_byte = UDR;
    
    // Store received byte
    command_buffer[buffer_index] = received_byte;
    buffer_index++;
    
    // Check for end of command (carriage return)
    if (received_byte == '\r' || buffer_index >= 3) {
        if (buffer_index >= 2) {
            // Valid command received
            current_direction = command_buffer[0];
            current_speed = command_buffer[1];
            command_received = 1;
        }
        
        // Reset buffer
        buffer_index = 0;
    }
}

/*
 * Process received command and execute corresponding action
 */
void processCommand(uint8_t direction, uint8_t speed) {
    switch(direction) {
        case CMD_FORWARD:
            forward(speed);
            break;
        case CMD_BACKWARD:
            backward(speed);
            break;
        case CMD_LEFT:
            left(speed);
            break;
        case CMD_RIGHT:
            right(speed);
            break;
        case CMD_STOP:
        default:
            stop();
            break;
    }
    
    // Update status LED
    updateStatusLED(direction);
    
    // Send acknowledgment
    UART_sendString("OK\r\n");
}

/*
 * Set PWM value for specified channel
 * channel: 0 for left motors, 1 for right motors
 */
void setPWM(uint8_t channel, uint8_t value) {
    if (channel == 0) {
        OCR1A = value; // Left motors
    } else {
        OCR1B = value; // Right motors
    }
}

/*
 * Convert speed level to PWM value
 */
uint8_t speedToPWM(uint8_t speed_level) {
    switch(speed_level) {
        case SPEED_LOW:
            return PWM_LOW;
        case SPEED_MEDIUM:
            return PWM_MEDIUM;
        case SPEED_HIGH:
            return PWM_HIGH;
        default:
            return PWM_LOW;
    }
}

/*
 * Set motor speeds for left and right sides
 */
void setMotorSpeed(uint8_t left_speed, uint8_t right_speed) {
    setPWM(0, left_speed);   // Left motors
    setPWM(1, right_speed);  // Right motors
}

/*
 * Move car forward with specified speed
 */
void forward(uint8_t speed) {
    uint8_t pwm_value = speedToPWM(speed);
    
    // Set left motors forward
    PORTC |= (1 << LEFT_MOTOR1_IN1) | (1 << LEFT_MOTOR2_IN1);
    PORTC &= ~((1 << LEFT_MOTOR1_IN2) | (1 << LEFT_MOTOR2_IN2));
    
    // Set right motors forward
    PORTC |= (1 << RIGHT_MOTOR1_IN1) | (1 << RIGHT_MOTOR2_IN1);
    PORTC &= ~((1 << RIGHT_MOTOR1_IN2) | (1 << RIGHT_MOTOR2_IN2));
    
    // Set speeds
    setMotorSpeed(pwm_value, pwm_value);
}

/*
 * Move car backward with specified speed
 */
void backward(uint8_t speed) {
    uint8_t pwm_value = speedToPWM(speed);
    
    // Set left motors backward
    PORTC |= (1 << LEFT_MOTOR1_IN2) | (1 << LEFT_MOTOR2_IN2);
    PORTC &= ~((1 << LEFT_MOTOR1_IN1) | (1 << LEFT_MOTOR2_IN1));
    
    // Set right motors backward
    PORTC |= (1 << RIGHT_MOTOR1_IN2) | (1 << RIGHT_MOTOR2_IN2);
    PORTC &= ~((1 << RIGHT_MOTOR1_IN1) | (1 << RIGHT_MOTOR2_IN1));
    
    // Set speeds
    setMotorSpeed(pwm_value, pwm_value);
}

/*
 * Turn car left with specified speed
 */
void left(uint8_t speed) {
    uint8_t pwm_value = speedToPWM(speed);
    
    // Set left motors backward (or stop for differential steering)
    PORTC |= (1 << LEFT_MOTOR1_IN2) | (1 << LEFT_MOTOR2_IN2);
    PORTC &= ~((1 << LEFT_MOTOR1_IN1) | (1 << LEFT_MOTOR2_IN1));
    
    // Set right motors forward
    PORTC |= (1 << RIGHT_MOTOR1_IN1) | (1 << RIGHT_MOTOR2_IN1);
    PORTC &= ~((1 << RIGHT_MOTOR1_IN2) | (1 << RIGHT_MOTOR2_IN2));
    
    // Set speeds (right motors at full speed, left motors at reduced speed)
    setMotorSpeed(pwm_value/2, pwm_value);
}

/*
 * Turn car right with specified speed
 */
void right(uint8_t speed) {
    uint8_t pwm_value = speedToPWM(speed);
    
    // Set left motors forward
    PORTC |= (1 << LEFT_MOTOR1_IN1) | (1 << LEFT_MOTOR2_IN1);
    PORTC &= ~((1 << LEFT_MOTOR1_IN2) | (1 << LEFT_MOTOR2_IN2));
    
    // Set right motors backward (or stop for differential steering)
    PORTC |= (1 << RIGHT_MOTOR1_IN2) | (1 << RIGHT_MOTOR2_IN2);
    PORTC &= ~((1 << RIGHT_MOTOR1_IN1) | (1 << RIGHT_MOTOR2_IN1));
    
    // Set speeds (left motors at full speed, right motors at reduced speed)
    setMotorSpeed(pwm_value, pwm_value/2);
}

/*
 * Stop all motors
 */
void stop(void) {
    // Stop all motors by setting PWM to 0
    setMotorSpeed(0, 0);
    
    // Clear all motor control pins
    PORTC = 0x00;
}

/*
 * Update status LED based on current direction
 * Blinking pattern indicates different states
 */
void updateStatusLED(uint8_t direction) {
    static uint8_t led_state = 0;
    
    switch(direction) {
        case CMD_FORWARD:
            // Solid ON for forward
            PORTB |= (1 << STATUS_LED);
            break;
        case CMD_BACKWARD:
            // Blinking for backward
            led_state = !led_state;
            if (led_state) {
                PORTB |= (1 << STATUS_LED);
            } else {
                PORTB &= ~(1 << STATUS_LED);
            }
            break;
        case CMD_LEFT:
        case CMD_RIGHT:
            // Fast blinking for turning
            led_state = !led_state;
            if (led_state) {
                PORTB |= (1 << STATUS_LED);
            } else {
                PORTB &= ~(1 << STATUS_LED);
            }
            break;
        case CMD_STOP:
        default:
            // LED off when stopped
            PORTB &= ~(1 << STATUS_LED);
            break;
    }
} 
