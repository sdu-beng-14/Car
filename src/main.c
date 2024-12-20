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

// Function to initialize ADC
void ADC_Init() {
    ADMUX = (1 << REFS0); // Set reference voltage to AVcc (5V)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler 64
}

// Function to read ADC value
uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

// Function to initialize UART
void UART_Init(uint16_t baudrate) {
    uint16_t ubrr_value = (F_CPU / (16UL * baudrate)) - 1;
    UBRR0H = (ubrr_value >> 8); // Set high byte of UBRR
    UBRR0L = ubrr_value;        // Set low byte of UBRR
    UCSR0B = (1 << TXEN0);      // Enable transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Set frame format: 8 data bits, 1 stop bit
}

// Function to send a string via UART
void UART_SendString(const char *str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0))); // Wait until buffer is empty
        UDR0 = *str++;
    }
}

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
        _delay_ms((int)(time_step * 150)); 
        elapsed_time += time_step;
    }

    // Stop the motor after loop completion
    OCR0A = 0;
    OCR0B = 0;
    printf("Target reached. Total Time: %.1f s, Total Distance: %.2f cm\n", elapsed_time, target_distance);
}



int main(void) {
    cli();

    ADC_Init(); // Initialize ADC
    uart_init();
    io_redirect();
    setup_encoder();
    timer1_init();
    setup_motor();

    printf("System Initialized\n");

    char buffer[32];
    const float referenceVoltage = 5.0; // Reference voltage (5v)

    uint16_t adcValue = ADC_Read(0); // Read ADC value from channel 0
    float Vadcpin = (adcValue * referenceVoltage) / 1023.0; // Calculate ADC pin voltage
    float batteryVoltage = Vadcpin + 5.0; // Add 5V to Vadcpin and save as a float

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