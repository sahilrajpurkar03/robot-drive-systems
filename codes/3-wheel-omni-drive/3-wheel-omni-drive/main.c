/*
 * 3-wheel-drive.c
 *
 * Created: 10-01-2018 17:16:01
 * Author : Sahil
 *
 * Description:
 *   Control system for a 3-wheel omni-directional drive robot
 *   Controlled via serial communication (Bluetooth or similar)
 *   Uses PWM and joystick vector mapping for motion control
 */

#define F_CPU 14745600UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// Serial communication setup
#define BAUD 9600
#define BAUDRATE ((F_CPU / (BAUD * 16UL)) - 1)

// Motor direction and PWM output
#define drive_dir PORTB
#define drive_pwm1 OCR3A    // Front wheel
#define drive_pwm2 OCR3B    // Right wheel
#define drive_pwm3 OCR3C    // Left wheel

// LEDs or indicators
#define indicator_port1 PORTD
#define indicator_port2 PORTH

// Communication buffers
uint8_t RX[16] = {100};
uint8_t TX[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

// Communication flags and buffers
int RX_range = 200;
int RX_raw = 255;
int RX_ad = 255;
int RX_count = 0;
int flag[16] = {0};
uint8_t TX_raw = 200;
uint8_t TX_ad = 201;
uint8_t TX_flag = 0;

// PS2 button mappings
#define PS_L1 10
#define PS_R1 11
#define PS_L2 8
#define PS_R2 9
#define PS_L3 6
#define PS_R3 7
#define PS_SQUARE 14
#define PS_TRIANGLE 12
#define PS_CIRCLE 13
#define PS_CROSS 15
#define PS_UP 2
#define PS_DOWN 5
#define PS_LEFT 4
#define PS_RIGHT 3
#define PS_START 0
#define PS_SELECT 1

// Motor speed vectors
int w1 = 0, w2 = 0, w3 = 0;
int js_error = 20;
int pwm_range = 120;

// Analog joystick inputs
int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0;
int pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;

// Button states
int butt[16] = {0};

// Function declarations
void manual();
void drive_3W(int x_vect, int y_vect, int m_vect);
void drivewheel_1(int sp_vect);
void drivewheel_2(int sp_vect);
void drivewheel_3(int sp_vect);
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max);
long limit_var(long in_var, long l_limit, long h_limit);
void pwm_init();
void serialstart_3();
void receive();

int main(void)
{
    // Set Data Direction Registers
    DDRB = 0xFF;    // PORTB - Motor directions
    DDRD = 0xFF;    // Indicator port 1
    DDRE = 0xFF;    // PWM outputs (OC3A, OC3B, OC3C)
    PORTE = 0xFF;   // Pull-up
    DDRH = 0xFF;    // Indicator port 2
    DDRJ = 0xFF;    // Serial TX on port J
    DDRL = 0xF0;    // Flags input/output
    PORTL |= 0x0F;  // Enable pull-ups on input flags

    sei();              // Enable global interrupts
    serialstart_3();    // Start serial port 3
    pwm_init();         // Initialize PWM

    drive_3W(0, 0, 0);  // Stop wheels on start

    while (1)
    {
        manual();       // Run manual control
    }
}

void manual()
{
    drive_3W(xj1, yj1, (xj2 + yj2) / 2);  // Drive via joystick input
}

// Omni-wheel inverse kinematics
void drive_3W(int x_vect, int y_vect, int m_vect)
{
    w1 = m_vect - x_vect - y_vect;
    w2 = m_vect + x_vect - y_vect;
    w3 = (m_vect + y_vect) * 2;

    drivewheel_1(w1);
    drivewheel_2(w2);
    drivewheel_3(w3);
}

// Front wheel
void drivewheel_1(int sp_vect)
{
    sp_vect = limit_var(sp_vect, -255, 255);
    if (sp_vect < -js_error)
    {
        drive_dir |= (1 << PD0);
        drive_dir &= ~(1 << PD1);
        sp_vect = -sp_vect;
    }
    else if (sp_vect > js_error)
    {
        drive_dir |= (1 << PD1);
        drive_dir &= ~(1 << PD0);
    }
    else
    {
        sp_vect = 0;
    }
    drive_pwm1 = sp_vect;
}

// Right wheel
void drivewheel_2(int sp_vect)
{
    sp_vect = limit_var(sp_vect, -255, 255);
    if (sp_vect < -js_error)
    {
        drive_dir |= (1 << PD2);
        drive_dir &= ~(1 << PD3);
        sp_vect = -sp_vect;
    }
    else if (sp_vect > js_error)
    {
        drive_dir |= (1 << PD3);
        drive_dir &= ~(1 << PD2);
    }
    else
    {
        sp_vect = 0;
    }
    drive_pwm2 = sp_vect;
}

// Left wheel
void drivewheel_3(int sp_vect)
{
    sp_vect = limit_var(sp_vect, -255, 255);
    if (sp_vect < -js_error)
    {
        drive_dir |= (1 << PD4);
        drive_dir &= ~(1 << PD5);
        sp_vect = -sp_vect;
    }
    else if (sp_vect > js_error)
    {
        drive_dir |= (1 << PD5);
        drive_dir &= ~(1 << PD4);
    }
    else
    {
        sp_vect = 0;
    }
    drive_pwm3 = sp_vect;
}

// Map input from one range to another
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max)
{
    return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Constrain variable within a range
long limit_var(long in_var, long l_limit, long h_limit)
{
    if (in_var > h_limit) return h_limit;
    else if (in_var < l_limit) return l_limit;
    return in_var;
}

// Initialize PWM for Timer3 (OC3A, OC3B, OC3C)
void pwm_init()
{
    TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM30);
    TCCR3B |= (1 << CS32) | (1 << CS30) | (1 << WGM32);  // Fast PWM with prescaler 1024
}

// Initialize USART3
void serialstart_3()
{
    UBRR3H = BAUDRATE >> 8;
    UBRR3L = BAUDRATE;
    UCSR3B = (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3);  // Enable RX, TX, and RX interrupt
    UCSR3C = (1 << UCSZ31) | (1 << UCSZ30);                // 8-bit data
}

// USART RX Interrupt Handler
ISR(USART3_RX_vect)
{
    RX_count = 1;
    RX_raw = UDR3;

    if ((RX_raw > 200) && (RX_raw < 255)) // Address bytes
    {
        RX_ad = RX_raw;

        if ((RX_raw > 230) && (RX_raw < 247)) // Button values
        {
            uint8_t r_temp0 = RX_raw - 231;
            butt[r_temp0] = 1;
        }
    }
    else if ((RX_raw >= 0) && (RX_raw < 201)) // Actual data
    {
        uint8_t r_temp1 = RX_ad - 201;
        if (r_temp1 < 16)
        {
            RX[r_temp1] = RX_raw;
        }
    }
}

// Process received data
void receive()
{
    yj1 = map_value(RX[0], 0, RX_range, -pwm_range, pwm_range);
    xj1 = map_value(RX[1], 0, RX_range, pwm_range, -pwm_range);
    yj2 = map_value(RX[2], 0, RX_range, -pwm_range, pwm_range);
    xj2 = map_value(RX[3], 0, RX_range, pwm_range, -pwm_range);

    // Clear each button manually (explicitly preserved)
    if (butt[PS_START] == 1) butt[PS_START] = 0;
    if (butt[PS_SELECT] == 1) butt[PS_SELECT] = 0;
    if (butt[PS_UP] == 1) butt[PS_UP] = 0;
    if (butt[PS_DOWN] == 1) butt[PS_DOWN] = 0;
    if (butt[PS_LEFT] == 1) butt[PS_LEFT] = 0;
    if (butt[PS_RIGHT] == 1) butt[PS_RIGHT] = 0;
    if (butt[PS_SQUARE] == 1) butt[PS_SQUARE] = 0;
    if (butt[PS_CIRCLE] == 1) butt[PS_CIRCLE] = 0;
    if (butt[PS_TRIANGLE] == 1) butt[PS_TRIANGLE] = 0;
    if (butt[PS_CROSS] == 1) butt[PS_CROSS] = 0;
    if (butt[PS_L1] == 1) butt[PS_L1] = 0;
    if (butt[PS_R1] == 1) butt[PS_R1] = 0;
    if (butt[PS_L2] == 1) butt[PS_L2] = 0;
    if (butt[PS_R2] == 1) butt[PS_R2] = 0;
    if (butt[PS_L3] == 1) butt[PS_L3] = 0;
    if (butt[PS_R3] == 1) butt[PS_R3] = 0;
}
