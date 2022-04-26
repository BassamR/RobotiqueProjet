#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>		//is it normal to include main here ?? we did that in the tp
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>

static float ERROR_THRESHOLD = 0.25; //minimum error we want to detect is 0.2cm

static float kp = 1.2;
static float ki = 0.0045;
static float ts = 1/100; // sampling period in s, either multiply with ts or with the variable time
static int reference = 10; // cm

static float ui = 0; // initial condition for integrator term
static float up = 0;

static float uMax = 13;

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    float error = 0; // = ref - distance, here the declaration = initial error

    while(1) {
        time = chVTGetSystemTime();

        error = VL53L0X_get_dist_mm()/10 - reference;

        if(fabs(error) < ERROR_THRESHOLD) {
        	speed = 0;
        } else {
            up = kp*error;
            if(!(fabs(up + ui + ki*error) > uMax)) ui = ui + ki*error; // corresponds to current ui + ARW

            speed = up + ui; // loop
        }

        //chprintf((BaseSequentialStream *)&SDU1, "%nspeed(cm)=%.2f \r\n", speed);

        speed = speed * 1000/13; //convert from cm/s to steps/s

        //applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);

        //100Hz
		//chprintf((BaseSequentialStream *)&SD3, "%ntime is %d \r\n",chVTGetSystemTime()-time);
		//chThdSleepMilliseconds(10);
		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pid_regulator_start(void){
	//inits the motors
	motors_init();
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

