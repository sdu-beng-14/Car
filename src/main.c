#define F_CPU 16000000UL // Needs to be defined for the delay functions to work.
#define BAUD 9600
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include <math.h>

// Constants
#define WHEEL_CIRCUMFERENCE 18.85  // Wheel circumference in cm
#define ENCODER_HOLES 15       // Number of filled holes in the encoder
#define MIN_SPEED 50          // Minimum motor speed
#define MAX_SPEED 255         // Maximum motor speed
#define DEBOUNCE_TIME 50       // Debounce time in ms

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
    DDRD |= (1 << PD6); // PD6 (OC0A)
    DDRD |= (1 << PD5); // PD5 (OC0B)

    TCCR0A |= 0xA3; // Fast PWM, non-inverting mode
    TCCR0B |= 0x05; // Prescaler 1024 for Timer0
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

    int total_hits = calc_holes(target_distance);
    float elapsed_time = 0.0;
    float time_step = 0.1; // Smaller time step for smoother updates (in seconds)

    int motor_speed = MAX_SPEED; // Start at maximum speed
    OCR0A = motor_speed;         // Set motor PWM initially
    OCR0B = motor_speed;

    while (elapsed_time <= target_time) {
        int remaining_hits = total_hits - trigger_count;

        // Time-based cubic deceleration
        float progress = elapsed_time / target_time; // Progress from 0 to 1 based on time
        motor_speed = MIN_SPEED + (int)((1.0 - pow(progress, 3)) * (MAX_SPEED - MIN_SPEED));
        if (motor_speed > MAX_SPEED) motor_speed = MAX_SPEED; // Cap at max speed

        motor_speed = motor_speed < MIN_SPEED ? MIN_SPEED : motor_speed;

        OCR0A = motor_speed;
        OCR0B = motor_speed;

        // Print debug info: elapsed time, triggered hits, remaining hits, motor speed
        printf("Time: %.1f s, Triggered Hits: %d, Remaining Hits: %d, PWM: %d\n",
               elapsed_time, trigger_count, remaining_hits, motor_speed);

        // Stop if the car reaches the target distance before time is up
        if (trigger_count >= total_hits) {
            OCR0A = 0;
            OCR0B = 0;
            printf("Target distance reached at %.1f s.\n", elapsed_time);
            break;
        }

        _delay_ms((int)(time_step * 1000)); // Delay in milliseconds
        elapsed_time += time_step;         // Increment elapsed time
    }

    // Ensure the motor stops after the loop
    OCR0A = 0;
    OCR0B = 0;

    printf("Drive complete. Total Time: %.1f s\n", elapsed_time);
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
