#define F_CPU 16000000UL // Needs to be defined for the delay functions to work.
#define BAUD 9600
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include <math.h>

// Constants
#define WHEEL_CIRCUMFERENCE 19.635  // Updated for 62.5 mm diameter wheels
#define ENCODER_HOLES 15       // Number of filled holes in the encoder
#define MIN_SPEED 30          // Minimum motor speed
#define MAX_SPEED 100         // Maximum motor speed
#define DEBOUNCE_TIME 10       // Debounce time in ms

// Global variable to count encoder triggers
volatile uint16_t trigger_count = 0; // How many holes the car actually went through
int time = 0;                        // Time for the car to move
float distance = 0;                  // Distance the car should go
volatile uint8_t last_state = 0;
volatile uint32_t last_debounce_time = 0;

// Function prototypes
void setup_encoder(void);
void timer1_init(void);
void setup_motor(void);
int calc_holes(float target_distance);
void motor(float target_distance, float target_time);

ISR(PCINT0_vect) {
    uint8_t current_state = (PINB & (1 << PB0)) ? 1 : 0;
    uint32_t current_time = (TCNT1 * (1024.0 / F_CPU) * 1000.0);

    if (current_time - last_debounce_time > DEBOUNCE_TIME) {
        if (current_state != last_state) {
            last_state = current_state;
            if (current_state) {
                trigger_count++;
            }
            last_debounce_time = current_time;
        }
    }
}

void timer1_init(void) {
    TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
    TCNT1 = 0;                          // Start counting from 0
}

void setup_motor(void) {
    DDRD |= (1 << PD6); // Set PD6 (OC0A) as output
    DDRD |= (1 << PD5); // Set PD5 (OC0B) as output

    // Configure Timer0 for Fast PWM, non-inverting mode
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler set to 1024
}


void setup_encoder(void) {
    DDRB &= ~(1 << PB0); // Set PB0 as input
    PORTB |= (1 << PB0); // Enable pull-up resistor on PB0
    PCICR |= (1 << PCIE0); // Enable Pin Change Interrupt Control for PCINT0-7
    PCMSK0 |= (1 << PCINT0); // Enable interrupt for PCINT0 (PB0)
}

int calc_holes(float target_distance) {
    return (int)((target_distance / WHEEL_CIRCUMFERENCE) * ENCODER_HOLES);
}

void motor(float target_distance, float target_time) {
    cli(); // Disable interrupts temporarily
    trigger_count = 0;
    sei(); // Re-enable interrupts

    float elapsed_time = 0.0;
    float time_step = 0.1; // Smaller time step for smoother updates (in seconds)
    float distance_per_tick = WHEEL_CIRCUMFERENCE / ENCODER_HOLES;

    while (elapsed_time < target_time) {
        // Calculate current progress
        float current_distance = trigger_count * distance_per_tick;
        float remaining_distance = target_distance - current_distance;
        float remaining_time = target_time - elapsed_time;

        // Stop if the target distance is reached
        if (remaining_distance <= 0 || remaining_time <= 0) {
            break;
        }

        // Calculate expected progress
        float expected_distance = (elapsed_time / target_time) * target_distance;
        float progress_error = expected_distance - current_distance; // Positive if behind, negative if ahead

        // Proportional control for speed adjustment
        int base_speed = (remaining_distance / remaining_time) / (WHEEL_CIRCUMFERENCE / (ENCODER_HOLES * 255));

        // Aggressive deceleration logic
        float deceleration_factor = (remaining_distance < (target_distance * 0.3)) ? 2.0 : 1.0; // Steeper deceleration near the end
        int correction = (int)(progress_error * 5.0 * deceleration_factor); // Adjust proportional constant as needed
        int motor_speed = base_speed + correction;

        // Ensure motor speed stays within bounds
        if (motor_speed > MAX_SPEED) motor_speed = MAX_SPEED;
        if (motor_speed < MIN_SPEED) motor_speed = MIN_SPEED;

        // Update motor PWM
        OCR0A = motor_speed;
        OCR0B = motor_speed;

        // Print debug info
        printf("Time: %.1f s, Current Distance: %.2f cm, Remaining Distance: %.2f cm, Speed: %d PWM\n",
               elapsed_time, current_distance, remaining_distance, motor_speed);

        // Delay and update elapsed time
        _delay_ms(time_step * 100);
        elapsed_time += time_step;
    }

    // Stop the motor after loop completion
    OCR0A = 0;
    OCR0B = 0;
    printf("Target reached. Total Time: %.1f s, Total Distance: %.2f cm\n", elapsed_time, target_distance);
}



int main(void) {
    uart_init();
    io_redirect();
    cli();

    printf("System Initialized\n");

    setup_encoder();
    timer1_init();
    setup_motor();

    while (1) {
        printf("Enter time in seconds: ");
        scanf("%d", &time);
        printf("Enter distance in cm: ");
        scanf("%f", &distance);

        sei(); // Enable global interrupts

        motor(distance, time);
    }

    return 0;
}