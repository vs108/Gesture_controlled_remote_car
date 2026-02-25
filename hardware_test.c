/*
 * Hardware Test Program for Gesture-Controlled 4WD RC Car
 * Tests all components and connections before running main application
 * 
 * This program can be used on both controller and car boards
 * with different test modes selected via compile-time defines
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Test mode selection
// Uncomment one of these for specific board testing
// #define TEST_CONTROLLER
// #define TEST_CAR

// If neither is defined, run all tests
#ifndef TEST_CONTROLLER
#ifndef TEST_CAR
#define TEST_ALL
#endif
#endif

// Pin definitions
#define LED1 PB0
#define LED2 PB1
#define LED3 PB2
#define LED4 PB3

// UART functions
void initUART(void);
void UART_send(uint8_t data);
void UART_sendString(const char* str);
uint8_t UART_receive(void);

// I2C functions
void initI2C(void);
void I2C_start(void);
void I2C_stop(void);
void I2C_write(uint8_t data);
uint8_t I2C_read(uint8_t ack);

// Test functions
void testLEDs(void);
void testUART(void);
void testI2C(void);
void testPWM(void);
void testMotors(void);
void testMPU6050(void);

int main(void) {
    // Initialize UART for test output
    initUART();
    UART_sendString("\r\n=== Hardware Test Started ===\r\n");
    
    // Enable interrupts
    sei();
    
    // Run tests based on mode
    #ifdef TEST_CONTROLLER
        UART_sendString("Running Controller Tests...\r\n");
        testLEDs();
        testUART();
        testI2C();
        testMPU6050();
    #elif defined(TEST_CAR)
        UART_sendString("Running Car Tests...\r\n");
        testLEDs();
        testUART();
        testPWM();
        testMotors();
    #else
        UART_sendString("Running All Tests...\r\n");
        testLEDs();
        testUART();
        testI2C();
        testPWM();
        testMotors();
        testMPU6050();
    #endif
    
    UART_sendString("=== Hardware Test Complete ===\r\n");
    
    // Continuous status indication
    while(1) {
        // Blink all LEDs to show system is running
        PORTB = 0x0F; // All LEDs on
        _delay_ms(500);
        PORTB = 0x00; // All LEDs off
        _delay_ms(500);
    }
    
    return 0;
}

/*
 * Initialize UART for test communication
 */
void initUART(void) {
    // Set baud rate (9600 @ 16MHz)
    UBRRH = 0x00;
    UBRRL = 0x67;
    
    // Enable transmitter and receiver
    UCSRB = (1 << TXEN) | (1 << RXEN);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

/*
 * Send single byte via UART
 */
void UART_send(uint8_t data) {
    while (!(UCSRA & (1 << UDRE)));
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
    while (!(UCSRA & (1 << RXC)));
    return UDR;
}

/*
 * Initialize I2C communication
 */
void initI2C(void) {
    // Set I2C bit rate (100kHz @ 16MHz)
    TWBR = 72;
    
    // Enable TWI
    TWCR = (1 << TWEN);
}

/*
 * Send I2C start condition
 */
void I2C_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

/*
 * Send I2C stop condition
 */
void I2C_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

/*
 * Write byte via I2C
 */
void I2C_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

/*
 * Read byte via I2C
 */
uint8_t I2C_read(uint8_t ack) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

/*
 * Test LED functionality
 */
void testLEDs(void) {
    UART_sendString("Testing LEDs...\r\n");
    
    // Set LED pins as output
    DDRB |= (1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4);
    
    // Test each LED individually
    for (int i = 0; i < 4; i++) {
        PORTB = (1 << i); // Turn on one LED
        UART_sendString("LED ");
        UART_send('1' + i);
        UART_sendString(" ON\r\n");
        _delay_ms(500);
        PORTB = 0x00; // Turn off all LEDs
        _delay_ms(200);
    }
    
    // Test all LEDs together
    PORTB = 0x0F; // All LEDs on
    UART_sendString("All LEDs ON\r\n");
    _delay_ms(1000);
    PORTB = 0x00; // All LEDs off
    UART_sendString("All LEDs OFF\r\n");
    
    UART_sendString("LED test complete\r\n\r\n");
}

/*
 * Test UART communication
 */
void testUART(void) {
    UART_sendString("Testing UART...\r\n");
    UART_sendString("Send any character to test echo...\r\n");
    
    // Echo test
    for (int i = 0; i < 5; i++) {
        uint8_t received = UART_receive();
        UART_sendString("Received: ");
        UART_send(received);
        UART_sendString("\r\n");
    }
    
    UART_sendString("UART test complete\r\n\r\n");
}

/*
 * Test I2C communication
 */
void testI2C(void) {
    UART_sendString("Testing I2C...\r\n");
    
    initI2C();
    
    // Test I2C bus by scanning for devices
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        I2C_start();
        I2C_write((addr << 1) | 0); // Write address
        
        if ((TWCR & (1 << TWINT)) && !(TWSR & 0xF8)) {
            UART_sendString("I2C device found at address: 0x");
            // Convert address to hex string
            uint8_t high = (addr >> 4) & 0x0F;
            uint8_t low = addr & 0x0F;
            UART_send(high < 10 ? '0' + high : 'A' + high - 10);
            UART_send(low < 10 ? '0' + low : 'A' + low - 10);
            UART_sendString("\r\n");
        }
        
        I2C_stop();
        _delay_ms(10);
    }
    
    UART_sendString("I2C test complete\r\n\r\n");
}

/*
 * Test PWM functionality
 */
void testPWM(void) {
    UART_sendString("Testing PWM...\r\n");
    
    // Set PWM pins as output
    DDRB |= (1 << PB0) | (1 << PB1); // PWM outputs
    
    // Configure Timer1 for PWM
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1 = 15999; // 1kHz PWM frequency
    
    // Test different PWM values
    uint16_t pwm_values[] = {0, 4000, 8000, 12000, 16000};
    
    for (int i = 0; i < 5; i++) {
        OCR1A = pwm_values[i]; // Left motors
        OCR1B = pwm_values[i]; // Right motors
        
        UART_sendString("PWM set to: ");
        // Convert to percentage
        uint8_t percent = (pwm_values[i] * 100) / 16000;
        UART_send('0' + (percent / 10));
        UART_send('0' + (percent % 10));
        UART_sendString("%\r\n");
        
        _delay_ms(1000);
    }
    
    // Turn off PWM
    OCR1A = 0;
    OCR1B = 0;
    
    UART_sendString("PWM test complete\r\n\r\n");
}

/*
 * Test motor connections
 */
void testMotors(void) {
    UART_sendString("Testing Motors...\r\n");
    
    // Set motor control pins as output
    DDRC |= 0xFF; // All PC pins as output
    DDRB |= (1 << PB0) | (1 << PB1); // Enable pins
    
    // Test each motor individually
    uint8_t motor_tests[][8] = {
        {1,0,0,0,0,0,0,0}, // Motor 1 forward
        {0,1,0,0,0,0,0,0}, // Motor 1 backward
        {0,0,1,0,0,0,0,0}, // Motor 2 forward
        {0,0,0,1,0,0,0,0}, // Motor 2 backward
        {0,0,0,0,1,0,0,0}, // Motor 3 forward
        {0,0,0,0,0,1,0,0}, // Motor 3 backward
        {0,0,0,0,0,0,1,0}, // Motor 4 forward
        {0,0,0,0,0,0,0,1}  // Motor 4 backward
    };
    
    for (int i = 0; i < 8; i++) {
        PORTC = motor_tests[i][0] | (motor_tests[i][1] << 1) | 
                (motor_tests[i][2] << 2) | (motor_tests[i][3] << 3) |
                (motor_tests[i][4] << 4) | (motor_tests[i][5] << 5) |
                (motor_tests[i][6] << 6) | (motor_tests[i][7] << 7);
        
        PORTB |= (1 << PB0) | (1 << PB1); // Enable motors
        
        UART_sendString("Testing motor ");
        UART_send('1' + (i / 2));
        UART_sendString(i % 2 == 0 ? " forward" : " backward");
        UART_sendString("\r\n");
        
        _delay_ms(500);
        
        PORTC = 0x00; // Stop all motors
        PORTB &= ~((1 << PB0) | (1 << PB1)); // Disable motors
        _delay_ms(200);
    }
    
    UART_sendString("Motor test complete\r\n\r\n");
}

/*
 * Test MPU6050 sensor
 */
void testMPU6050(void) {
    UART_sendString("Testing MPU6050...\r\n");
    
    initI2C();
    
    // Try to read MPU6050 WHO_AM_I register (should return 0x68)
    I2C_start();
    I2C_write(0xD0); // MPU6050 address + write
    I2C_write(0x75); // WHO_AM_I register
    I2C_start();
    I2C_write(0xD1); // MPU6050 address + read
    uint8_t who_am_i = I2C_read(0); // No ACK
    I2C_stop();
    
    if (who_am_i == 0x68) {
        UART_sendString("MPU6050 found! WHO_AM_I = 0x68\r\n");
        
        // Read accelerometer data
        I2C_start();
        I2C_write(0xD0);
        I2C_write(0x3B); // ACCEL_XOUT_H
        I2C_start();
        I2C_write(0xD1);
        
        uint8_t axh = I2C_read(1);
        uint8_t axl = I2C_read(1);
        uint8_t ayh = I2C_read(1);
        uint8_t ayl = I2C_read(1);
        uint8_t azh = I2C_read(1);
        uint8_t azl = I2C_read(0);
        I2C_stop();
        
        int16_t ax = (axh << 8) | axl;
        int16_t ay = (ayh << 8) | ayl;
        int16_t az = (azh << 8) | azl;
        
        UART_sendString("Accelerometer: X=");
        // Simple number output (basic implementation)
        if (ax < 0) {
            UART_send('-');
            ax = -ax;
        }
        UART_sendString("XXXX, Y=");
        if (ay < 0) {
            UART_send('-');
            ay = -ay;
        }
        UART_sendString("XXXX, Z=");
        if (az < 0) {
            UART_send('-');
            az = -az;
        }
        UART_sendString("XXXX\r\n");
        
    } else {
        UART_sendString("MPU6050 not found! WHO_AM_I = 0x");
        // Convert to hex
        uint8_t high = (who_am_i >> 4) & 0x0F;
        uint8_t low = who_am_i & 0x0F;
        UART_send(high < 10 ? '0' + high : 'A' + high - 10);
        UART_send(low < 10 ? '0' + low : 'A' + low - 10);
        UART_sendString("\r\n");
    }
    
    UART_sendString("MPU6050 test complete\r\n\r\n");
} 
