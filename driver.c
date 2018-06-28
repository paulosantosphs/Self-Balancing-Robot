#include "driver.h"

void driver_init(void) {
    //Fast PWM on PORTD pins 5 and 6
    DDRD |= (1 << DDD5) | (1 << DDD6);
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B |= (1 << CS00);
    //Pins for motor rotation direction PORTB pins 0,1,2,3
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3);
}

void send_pwm_motorA(uint8_t pwm, char rotation) {
    if (rotation) {
        PORTB |= (1 << DDB0);
        PORTB &= ~(1 << DDB1);
    } else {
        PORTB |= (1 << DDB1);
        PORTB &= ~(1 << DDB0);
    }

    OCR0A = pwm;
}

void send_pwm_motorB(uint8_t pwm, char rotation) {
    if (rotation) {
        PORTB |= (1 << DDB2);
        PORTB &= ~(1 << DDB3);
    } else {
        PORTB |= (1 << DDB3);
        PORTB &= ~(1 << DDB2);
    }

    OCR0B = pwm;
}