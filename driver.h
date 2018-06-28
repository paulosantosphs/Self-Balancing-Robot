#include <avr/io.h>

void driver_init(void);

void send_pwm_motorA(uint8_t pwm, char rotation);

void send_pwm_motorB(uint8_t pwm, char rotation);
