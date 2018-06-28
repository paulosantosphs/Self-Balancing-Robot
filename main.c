#define F_CPU 16000000UL
#define UART_BAUD_RATE 9600

#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>
#include <math.h>
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "i2c.h"
#include "uart.h"
#include "driver.h"

void timer_setup();

void get_time(double *dt);

volatile double count;
const double unit_t = 8 / 16000000;

int main(void) {

    DDRC = 0xFF;   // Port C contains the pins for i2c
    DDRB = 1 << 5;

    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    sei();

    double num = 1510034.385;

    i2c_init();
    driver_init();

    char rotation;
    uint8_t pwm;

    uint8_t in;

    int16_t accel_buff[3], gyro_buff[3];
    double accelX, accelY, accelZ;
    double gyroX, gyroY, gyroZ;
    double biasX, biasY;
    double phi_accel, theta_accel;
    double phi_innov, theta_innov;
    double phi_est, theta_est;
    double phi_prev, theta_prev;

    double dt;
    char buffer[7];

    _delay_ms(1000);

    if (mpu6050_start() == 0x68) {
        uart_puts("Found MPU!\n");
    } else {
        uart_puts("init error, got value:");
        return 0;
    }

    _delay_ms(1000);

    // initialize & test MPU5060 availability
    mpu6050_init();
    timer_setup();

    //find gyro bias
    biasX = 0;
    biasY = 0;
    uint8_t i;
    for (i = 0; i < 20; i++) {
        mpu6050_read_gyro_ALL(gyro_buff);
        biasX += gyro_buff[0];
        biasY += gyro_buff[1];
    }
    biasX = biasX / 20 * (3.14159 / 180) / 1000 / 32768;
    biasY = biasY / 20 * (3.14159 / 180) / 1000 / 32768;

    // initialization for Kalman filter
    double P = 0.0;
    double Q = 0.001;
    double R = 0.03;
    double Pp, K;
    mpu6050_read_accel_ALL(accel_buff);
    phi_prev = atan2(accelY, accelZ); // row
    theta_prev = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)); // pitch

    for (;;) {
        get_time(&dt);
        mpu6050_read_accel_ALL(accel_buff);
        mpu6050_read_gyro_ALL(gyro_buff);

        // acceleration (m/s^2)
        accelX = accel_buff[0] * 9.8 * 2 / 32768;
        accelY = accel_buff[1] * 9.8 * 2 / 32768;
        accelZ = accel_buff[2] * 9.8 * 2 / 32768;
        // gyro rate (rad/s)
        gyroX = gyro_buff[0] * (3.14159 / 180) / 1000 / 32768;
        gyroY = gyro_buff[1] * (3.14159 / 180) / 1000 / 32768;
        gyroZ = gyro_buff[2] * (3.14159 / 180) / 1000 / 32768;

        // estimation
        phi_est = phi_prev + dt * (gyroX - biasX);
        theta_est = theta_prev + dt * (gyroY - biasY);
        Pp = P + Q;

        // innovation
        phi_accel = atan2(accelY, accelZ); // row
        phi_innov = phi_accel - phi_est;
        theta_accel = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)); // pitch
        theta_innov = theta_accel - theta_est;

        // Kalman gain
        K = Pp / (Pp + R);

        // correction
        phi_prev = phi_prev + K * phi_innov;
        theta_prev = theta_prev + K * theta_innov;
        P = (1 - K) * Pp;

        if(theta_prev < 0){
            rotation = 0;
            pwm = (theta_prev * 150) * -1;
        }else{
            rotation = 1;
            pwm = (theta_prev * 150);
        }

        send_pwm_motorA(pwm, rotation);
        send_pwm_motorB(pwm, rotation);
        
        uart_putc('\n');
        dtostrf(pwm, 5, 4, buffer);   // convert interger into string (decimal format)
        uart_puts(buffer);

        /*uart_putc('\n');
        uart_puts("  Phi: ");
        dtostrf(phi_prev, 5, 4, buffer);   // convert interger into string (decimal format)
        uart_puts(buffer);
        uart_puts("  Theta: ");
        dtostrf(theta_prev, 5, 4, buffer);   // convert interger into string (decimal format)
        uart_puts(buffer);
        uart_puts("  Dt: ");
        dtostrf(dt, 5, 4, buffer);   // convert interger into string (decimal format)
        uart_puts(buffer);*/

    }
}//end of main



void timer_setup() {
    TCCR1A = 0x00;
    TIMSK1 |= _BV(TOIE1);
    TCCR1B |= _BV(CS11);
    TCCR1B &= ~(_BV(CS12) | _BV(CS10)); // prescaler=8

}


void get_time(double *dt) {
    cli();
    uint8_t l = TCNT1L;
    uint8_t h = TCNT1H;
    uint16_t step = h << 8 | l;
    *dt = (double) step * 5e-7 + count * 0.032768;
    count = 0;
    sei();
}


// timer 1 overflow interrupt handler
SIGNAL(TIMER1_OVF_vect){
        count += 1;
        //TCNT1H = 0x00;
        //TCNT1L = 0x00;

}




