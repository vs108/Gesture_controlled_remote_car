/*
 * Gesture-Controlled 4WD RC Car - Controller Code
 * ATmega16 microcontroller with MPU6050 and HC-05 Bluetooth
 * 
 * Hardware Connections:
 * - MPU6050: SCL -> PC0, SDA -> PC1 (I2C)
 * - HC-05: TX -> PD0, RX -> PD1 (UART)
 * - LED indicators: PB0-PB3 for status
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

// MPU6050 Register Addresses
#define MPU6050_ADDR        0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

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

// Tilt Thresholds (in degrees)
#define TILT_THRESHOLD_LOW    10
#define TILT_THRESHOLD_MEDIUM 25
#define TILT_THRESHOLD_HIGH   40

// Global Variables
volatile float pitch = 0.0;
volatile float roll = 0.0;
volatile uint8_t last_command = 0;

// Function Prototypes
void initSystem(void);
void initUART(void);
void initI2C(void);
void initMPU6050(void);
void UART_send(uint8_t data);
void UART_sendString(const char* str);
uint8_t UART_receive(void);
void I2C_start(void);
void I2C_stop(void);
void I2C_write(uint8_t data);
uint8_t I2C_read(uint8_t ack);
void MPU6050_write(uint8_t reg, uint8_t data);
uint8_t MPU6050_read(uint8_t reg);
void MPU6050_readAccel(int16_t* ax, int16_t* ay, int16_t* az);
void MPU6050_readGyro(int16_t* gx, int16_t* gy, int16_t* gz);
void calculateTilt(void);
void sendCommand(uint8_t direction, uint8_t speed);
uint8_t mapTiltToSpeed(float tilt);
void updateStatusLEDs(uint8_t direction, uint8_t speed);

int main(void) {
    // Initialize all systems
    initSystem();
    initUART();
    initI2C();
    initMPU6050();
    
    // Enable global interrupts
    sei();
    
    // Send startup message
    UART_sendString("Controller Ready\r\n");
    
    // Main control loop
    while(1) {
        // Read sensor data and calculate tilt
        calculateTilt();
        
        // Determine direction based on tilt
        uint8_t direction = CMD_STOP;
        uint8_t speed = SPEED_LOW;
        
        // Check pitch (forward/backward)
        if (pitch > TILT_THRESHOLD_LOW) {
            direction = CMD_FORWARD;
            speed = mapTiltToSpeed(pitch);
        } else if (pitch < -TILT_THRESHOLD_LOW) {
            direction = CMD_BACKWARD;
            speed = mapTiltToSpeed(-pitch);
        }
        // Check roll (left/right) - takes precedence if both are tilted
        else if (roll > TILT_THRESHOLD_LOW) {
            direction = CMD_RIGHT;
            speed = mapTiltToSpeed(roll);
        } else if (roll < -TILT_THRESHOLD_LOW) {
            direction = CMD_LEFT;
            speed = mapTiltToSpeed(-roll);
        }
        
        // Send command if changed or if stopping
        if (direction != last_command || direction == CMD_STOP) {
            sendCommand(direction, speed);
            last_command = direction;
        }
        
        // Update status LEDs
        updateStatusLEDs(direction, speed);
        
        // Small delay for stability
        _delay_ms(50);
    }
    
    return 0;
}

/*
 * Initialize system ports and basic settings
 */
void initSystem(void) {
    // Set status LED pins as output (PB0-PB3)
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
    PORTB = 0x00; // Turn off all LEDs initially
    
    // Set I2C pins as output (PC0-SCL, PC1-SDA)
    DDRC |= (1 << PC0) | (1 << PC1);
    
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
    
    // Enable transmitter and receiver
    UCSRB = (1 << TXEN) | (1 << RXEN);
    
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
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
 * Initialize I2C communication
 * SCL: PC0, SDA: PC1
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
 * Write to MPU6050 register
 */
void MPU6050_write(uint8_t reg, uint8_t data) {
    I2C_start();
    I2C_write((MPU6050_ADDR << 1) | 0); // Write address
    I2C_write(reg);
    I2C_write(data);
    I2C_stop();
}

/*
 * Read from MPU6050 register
 */
uint8_t MPU6050_read(uint8_t reg) {
    uint8_t data;
    
    I2C_start();
    I2C_write((MPU6050_ADDR << 1) | 0); // Write address
    I2C_write(reg);
    I2C_start();
    I2C_write((MPU6050_ADDR << 1) | 1); // Read address
    data = I2C_read(0); // No ACK for last byte
    I2C_stop();
    
    return data;
}

/*
 * Initialize MPU6050 sensor
 */
void initMPU6050(void) {
    // Wake up MPU6050 (clear sleep bit)
    MPU6050_write(MPU6050_PWR_MGMT_1, 0x00);
    _delay_ms(100);
    
    // Configure gyroscope (±2000°/s)
    MPU6050_write(MPU6050_GYRO_CONFIG, 0x18);
    
    // Configure accelerometer (±2g)
    MPU6050_write(MPU6050_ACCEL_CONFIG, 0x00);
    
    // Configure digital low pass filter
    MPU6050_write(MPU6050_CONFIG, 0x06);
    
    _delay_ms(100);
}

/*
 * Read accelerometer data
 */
void MPU6050_readAccel(int16_t* ax, int16_t* ay, int16_t* az) {
    *ax = (MPU6050_read(MPU6050_ACCEL_XOUT_H) << 8) | MPU6050_read(MPU6050_ACCEL_XOUT_H + 1);
    *ay = (MPU6050_read(MPU6050_ACCEL_XOUT_H + 2) << 8) | MPU6050_read(MPU6050_ACCEL_XOUT_H + 3);
    *az = (MPU6050_read(MPU6050_ACCEL_XOUT_H + 4) << 8) | MPU6050_read(MPU6050_ACCEL_XOUT_H + 5);
}

/*
 * Read gyroscope data
 */
void MPU6050_readGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    *gx = (MPU6050_read(MPU6050_GYRO_XOUT_H) << 8) | MPU6050_read(MPU6050_GYRO_XOUT_H + 1);
    *gy = (MPU6050_read(MPU6050_GYRO_XOUT_H + 2) << 8) | MPU6050_read(MPU6050_GYRO_XOUT_H + 3);
    *gz = (MPU6050_read(MPU6050_GYRO_XOUT_H + 4) << 8) | MPU6050_read(MPU6050_GYRO_XOUT_H + 5);
}

/*
 * Calculate tilt angles using complementary filter
 * Combines accelerometer and gyroscope data for smooth readings
 */
void calculateTilt(void) {
    static float pitch_gyro = 0.0;
    static float roll_gyro = 0.0;
    static uint32_t last_time = 0;
    
    int16_t ax, ay, az, gx, gy, gz;
    
    // Read sensor data
    MPU6050_readAccel(&ax, &ay, &az);
    MPU6050_readGyro(&gx, &gy, &gz);
    
    // Calculate time delta (assuming 50ms loop)
    float dt = 0.05;
    
    // Convert accelerometer readings to angles
    float pitch_accel = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
    float roll_accel = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    
    // Integrate gyroscope readings
    pitch_gyro += (gx / 16.4) * dt; // Convert to degrees/s
    roll_gyro += (gy / 16.4) * dt;
    
    // Complementary filter (0.96 gyro + 0.04 accel)
    pitch = 0.96 * pitch_gyro + 0.04 * pitch_accel;
    roll = 0.96 * roll_gyro + 0.04 * roll_accel;
    
    // Update gyro angles for next iteration
    pitch_gyro = pitch;
    roll_gyro = roll;
}

/*
 * Map tilt angle to speed level
 */
uint8_t mapTiltToSpeed(float tilt) {
    if (tilt < TILT_THRESHOLD_MEDIUM) {
        return SPEED_LOW;
    } else if (tilt < TILT_THRESHOLD_HIGH) {
        return SPEED_MEDIUM;
    } else {
        return SPEED_HIGH;
    }
}

/*
 * Send command via UART to car
 * Format: [Direction][Speed] (e.g., "F2" = forward medium speed)
 */
void sendCommand(uint8_t direction, uint8_t speed) {
    UART_send(direction);
    UART_send(speed);
    UART_send('\r'); // End of command
}

/*
 * Update status LEDs based on direction and speed
 * PB0: Forward indicator
 * PB1: Backward indicator  
 * PB2: Left indicator
 * PB3: Right indicator
 */
void updateStatusLEDs(uint8_t direction, uint8_t speed) {
    // Clear all LEDs first
    PORTB &= 0xF0; // Clear lower 4 bits
    
    // Set appropriate LED based on direction
    switch(direction) {
        case CMD_FORWARD:
            PORTB |= (1 << PB0);
            break;
        case CMD_BACKWARD:
            PORTB |= (1 << PB1);
            break;
        case CMD_LEFT:
            PORTB |= (1 << PB2);
            break;
        case CMD_RIGHT:
            PORTB |= (1 << PB3);
            break;
        case CMD_STOP:
        default:
            // All LEDs off
            break;
    }
} 
