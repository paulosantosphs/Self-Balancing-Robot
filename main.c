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
//#include "uart.h"
#include "driver.h"

void timer_setup();

void get_time(double *dt);

volatile double count;
const double unit_t = 8 / 16000000;

int main(void) {

    DDRC = 0xFF;   // Port C contains the pins for i2c
    DDRB = 1 << 5;

    //uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    sei();

    i2c_init();
    driver_init();

    char rotation;
    uint8_t pwm;

    int16_t accel_buff[3], gyro_buff[3];
    double accelX, accelY, accelZ;
    double gyroX, gyroY, gyroZ;
    double biasX, biasY;
    double phi_accel;
    double phi_innov;
    double phi_est;
    double phi_prev;

    double dt;
    char buffer[7];

    _delay_ms(1000);

    if (mpu6050_start() == 0x68) {
        //uart_puts("Found MPU!\n");
    } else {
        //uart_puts("init error, got value:");
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

    //Control gains 
    double kangle = -10.1139;
    double kdangle = -279.0673;
    double kposition = -6.3112;
    double kspeed = -40.6784;

    double phi;
    double phip = 0;
    double dphi;
    double accelYp = 0;
    double speedY = 0;
    double speedYp = 0;
    double positionY = 0;
    double positionYp = 0;

    double sampleTime = 0.001;
    double timerCounter = 0;

    double controlResult;
    char controlFlag;

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


        if(speedY > 0.4){
        	speedY = 0;
        }
        if(speedY < -0.4){
        	speedY = 0;
        }
        if(positionY > 0.02){
        	positionY = 0;
        }
        if(positionY < -0.02){
        	positionY = 0;
        }


        timerCounter = timerCounter + dt;
        if(timerCounter > sampleTime){
        	controlFlag = 1;

        	//uart_putc('\n');
        	// uart_puts("SampleTime: ");
        	// dtostrf(timerCounter, 5, 4, buffer);   // convert interger into string (decimal format)
        	// uart_puts(buffer);

        	timerCounter = 0;
        }

        if(controlFlag){

        	controlResult = (phi * kangle + dphi * kdangle + positionY * kposition + speedY * kspeed);
        	
        	//Conversion of control result to PWM(6v == 255)
			pwm = (controlResult*255)/6;
			 
        	if(phi < 0){
        		rotation = 1;
        	}else{
        		rotation = 0;
        	}

        	if(pwm < 0){
        		pwm = pwm * -1;
        	}
        	if(pwm > 255){
        		pwm = 255;
        	}


        	send_pwm_motorA(pwm, rotation);
        	send_pwm_motorB(pwm, rotation);

     		controlFlag = 0;

        	//uart_puts("Control: ");
        	//dtostrf(controlResult, 5, 4, buffer);   // convert interger into string (decimal format)
        	//uart_puts(buffer);
        	// uart_puts("Angle: ");
        	// dtostrf(phi_prev, 5, 4, buffer);   // convert interger into string (decimal format)
        	// uart_puts(buffer);
        	// uart_puts(" AngleDt: ");
        	// dtostrf(dphi, 5, 4, buffer);   // convert interger into string (decimal format)
        	// uart_puts(buffer);
        	// uart_puts(" speedY: ");
        	// dtostrf(speedY, 5, 4, buffer);   // convert interger into string (decimal format)
        	// uart_puts(buffer);
        	// uart_puts(" positionY: ");
        	// dtostrf(positionY, 5, 4, buffer);   // convert interger into string (decimal format)
        	// uart_puts(buffer);

        }

        // uart_putc('\n');
        // uart_puts("dt: ");
        // dtostrf(dt, 5, 4, buffer);   // convert interger into string (decimal format)
        // uart_puts(buffer);

        phip = phi;
        accelYp = accelY;
        speedYp = speedY;
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




