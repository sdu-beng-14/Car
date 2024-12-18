#define F_CPU 16000000UL // Needs to be defined for the delay functions to work.
#define BAUD 9600
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include <math.h>

// Constants
#define TARGET_DISTANCE 300.0  // Target distance in cm (3 meters)
#define TARGET_TIME 50.0       // Target time in seconds
#define WHEEL_CIRCUMFERENCE 18.85  // Wheel circumference in cm
#define ENCODER_HOLES 15       // Number of filled holes in the encoder
#define MIN_SPEED 245          // Minimum motor speed
#define MAX_SPEED 255         // Maximum motor speed
#define DEBOUNCE_TIME 1       // Debounce time in ms

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
    float hits_per_second = total_hits / target_time;

    int motor_speed = MAX_SPEED; // Start at maximum speed
    OCR0A = motor_speed;         // Set motor PWM initially
    OCR0B = motor_speed;

    for (float t = 0; t <= target_time; t += 1) {
        int expected_hits = (int)(hits_per_second * t);
        int remaining_hits = total_hits - trigger_count;

        if (trigger_count < expected_hits) {
            // Car is behind, increase speed
            int speed_boost = (expected_hits - trigger_count) * (MAX_SPEED - MIN_SPEED) / total_hits;
            motor_speed = MIN_SPEED + speed_boost;
            if (motor_speed > MAX_SPEED) motor_speed = MAX_SPEED;
        } else if (remaining_hits > 0) {
            // Adjust speed dynamically as we approach the target
            motor_speed = (int)((float)remaining_hits / total_hits * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
        } else {
            motor_speed = 0; // Stop motor
        }

        motor_speed = motor_speed < MIN_SPEED ? MIN_SPEED : motor_speed;

        OCR0A = motor_speed;
        OCR0B = motor_speed;

        printf("Time: %.1f s, Target Hits: %d, Triggered Hits: %d, Remaining Hits: %d, PWM: %d\n",
               t, expected_hits, trigger_count, remaining_hits, motor_speed);

        if (trigger_count >= total_hits) {
            OCR0A = 0;
            OCR0B = 0; // Stop the motor
            printf("Target distance reached.\n");
            break;
        }

        _delay_ms(1000);
    }

    OCR0A = 0;
    OCR0B = 0;

    printf("Drive complete.\n");
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
