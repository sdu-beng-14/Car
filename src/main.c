#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

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

int main(void) {
    ADC_Init(); // Initialize ADC
    UART_Init(9600); // Initialize UART with baud rate 9600

    char buffer[32];
    const float referenceVoltage = 5.0; // Reference voltage (AVcc)

    while (1) {
        uint16_t adcValue = ADC_Read(0); // Read ADC value from channel 0
        float Vadcpin = (adcValue * referenceVoltage) / 1023.0; // Calculate ADC pin voltage
        float batteryVoltage = Vadcpin + 5.0; // Add 5V to Vadcpin and save as a float

        // Print the calculated battery voltage
        snprintf(buffer, sizeof(buffer), "Battery Voltage: %.2f V\r\n", batteryVoltage);
        UART_SendString(buffer);

        _delay_ms(5000); // Wait 5 second
    }

    return 0;
}
