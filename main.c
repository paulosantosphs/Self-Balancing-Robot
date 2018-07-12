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

int16_t accel_buff[3], gyro_buff[3];
double accelX, accelY, accelZ;
double gyroX, gyroY, gyroZ;
double biasX, biasY;
double phi_accel;
double phi_innov;
double phi_est;
double phi_prev;

const double dt = 0.020032;
char buffer[7];

// initialization for Kalman filter
double P = 0.0;
double Q = 0.001;
double R = 0.03;
double Pp, K;

//Control gains 
const double kangle = -0.2484;
const double kdangle = -256.8558;
const double kposition = -1.3595;
const double kspeed = -7.7696;

double phi;
double phip = 0;
double dphi;
double accelYp = 0;
double speedY = 0;
double speedYp = 0;
double positionY = 0;
double positionYp = 0;

double controlResult;
char rotation;
uint8_t pwm;

int main(void) {

    DDRC = 0xFF;   // Port C contains the pins for i2c
    DDRB = 1 << 5;

    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));

    i2c_init();
    driver_init();
    sei();

    _delay_ms(1000);

    // if (mpu6050_start(MPU6050_ADDRESS) == 0x68) {
    //      uart_puts("Found MPU 0!\n");
    // } else {
    //      uart_puts("MPU 0 ERROR");
    // }


    // if (mpu6050_start(MPU6050_ADDRESS) == 0x69) {
    //      uart_puts("Found MPU 1!\n");
    // } else {
    //      uart_puts("MPU 1 ERROR");
    // }

    _delay_ms(1000);

    // initialize & test MPU5060 availability
    mpu6050_init(MPU6050_ADDRESS);
    timer_setup();

    //find gyro bias
    biasX = 0;
    biasY = 0;
    uint8_t i;
    for (i = 0; i < 20; i++) {
        mpu6050_read_gyro_ALL(MPU6050_ADDRESS, gyro_buff);
        biasX += gyro_buff[0];
        biasY += gyro_buff[1];
    }
    biasX = biasX / 20 * (3.14159 / 180) / 1000 / 32768;
    biasY = biasY / 20 * (3.14159 / 180) / 1000 / 32768;

    mpu6050_read_accel_ALL(MPU6050_ADDRESS, accel_buff);
    phi_prev = atan2(accelY, accelZ); // row

    while(1){}
    	
}//end of main

void timer_setup(){
    cli();
    DDRB |= (1 << DDB5);
    TCCR1B |= (1 << CS10) | (1 << CS12) | (1 << WGM12);
    OCR1A = dt / (0.000064);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}


ISR(TIMER1_COMPA_vect){
    mpu6050_read_accel_ALL(MPU6050_ADDRESS, accel_buff);
    mpu6050_read_gyro_ALL(MPU6050_ADDRESS, gyro_buff);

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
    Pp = P + Q;

    // innovation
    phi_accel = atan2(accelY, accelZ); // row
    phi_innov = phi_accel - phi_est;

    // Kalman gain
    K = Pp / (Pp + R);

    // correction
    phi_prev = phi_prev + K * phi_innov;
    P = (1 - K) * Pp;

    //State variables
    phi = phi_prev;
    dphi = (phi - phip)/dt;
    speedY = speedY + (accelY - accelYp) * dt;
    positionY = positionY + (speedY - speedYp) * dt;


    if (speedY > 0.4){
        speedY = 0;
    }
    if (speedY < -0.4){
        speedY = 0;
    }
    if (positionY > 0.02){
        positionY = 0;
    }
    if (positionY < -0.02){
        positionY = 0;
    }

    controlResult = (phi * kangle + dphi * kdangle + positionY * kposition + speedY * kspeed);

    //Conversion of control result to PWM(6v == 255)
    pwm = (controlResult*255)/6;

    if (phi < 0){
        rotation = 1;
    }else{
        rotation = 0;
    }

    if (pwm < 0){
        pwm = pwm * -1;
    }
    if (pwm > 255){
        pwm = 255;
    }

    send_pwm_motorA(pwm, rotation);
    send_pwm_motorB(pwm, rotation);

    uart_putc(',');
    //uart_puts("dt: ");
    //dtostrf(dt, 5, 4, buffer);   // convert interger into string (decimal format)
    //uart_puts(buffer);
    //uart_puts(" Control: ");
    //dtostrf(pwm, 5, 4, buffer);   // convert interger into string (decimal format)
    //uart_puts(buffer);
    //uart_puts("Angle: ");
    dtostrf(phi_prev, 5, 4, buffer);   // convert interger into string (decimal format)
    uart_puts(buffer);
    // uart_puts(" AngleDt: ");
    //dtostrf(dphi, 5, 10, buffer);   // convert interger into string (decimal format)
    //uart_puts(buffer);
    // uart_puts(" speedY: ");
    // dtostrf(speedY, 5, 4, buffer);   // convert interger into string (decimal format)
    // uart_puts(buffer);
    // uart_puts(" positionY: ");
    // dtostrf(positionY, 5, 4, buffer);   // convert interger into string (decimal format)
    // uart_puts(buffer);

    phip = phi;
    accelYp = accelY;
    speedYp = speedY;
}



