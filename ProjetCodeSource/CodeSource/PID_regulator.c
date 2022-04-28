#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>		//is it normal to include main here ?? we did that in the tp
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <audio_processing.h>

#define ROTATION_COEFF 				0.5
#define ROTATION_ERROR_THRESHOLD	5 // deg

#define ERROR_THRESHOLD 			0.25 //minimum error we want to detect is 0.2cm

// Static PID variables for distance regulation
static float kp = 1.2;
static float ki = 0.0045;
static float ts = 1/100; // sampling period in s, either multiply with ts or with the variable time
static int reference = 10; // cm

static float ui = 0; // initial condition for integrator term
static float up = 0;

static float uMax = 13;

// Static PID variables for angle regulation
static float angleKp = 2;
static float angleKi = 0;
static float angleKd = 2;
static int angleReference = 90; //degrees

static float angleUi = 0;
static float angleUp = 0;
static float angleUd = 0;

static float angleUmax = 13;
static float prevAngleError = 0;

static int count = 0;


// Thread declaration
//static THD_WORKING_AREA(waPiRegulator, 256);
//static THD_FUNCTION(PiRegulator, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    systime_t time;
//
//    int16_t speed = 0;
//
//    float error = 0; // = ref - distance, here the declaration = initial error
//
//    while(1) {
//        time = chVTGetSystemTime();
//
//        error = VL53L0X_get_dist_mm()/10 - reference;
//
//        if(fabs(error) < ERROR_THRESHOLD) {
//        	speed = 0;
//        } else {
//            up = kp*error;
//            if(!(fabs(up + ui + ki*error) > uMax)) ui = ui + ki*error; // corresponds to current ui + ARW
//
//            speed = up + ui; // loop
//        }
//
//        //chprintf((BaseSequentialStream *)&SDU1, "%nspeed(cm)=%.2f \r\n", speed);
//
//        speed = speed * 1000/13; //convert from cm/s to steps/s
//
//        //applies the speed from the PI regulator
//		right_motor_set_speed(speed);
//		left_motor_set_speed(speed);
//
//        //100Hz
//		//chprintf((BaseSequentialStream *)&SD3, "%ntime is %d \r\n",chVTGetSystemTime()-time);
//		//chThdSleepMilliseconds(10);
//		chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
//}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    float angleSpeed = 0; //what the rotational PID corrects

    float angleError = 0; // = ref - distance, here the declaration = initial error

    while(1) {
        time = chVTGetSystemTime();

        float angle = getAngleFromSource();
        angleError = angle - angleReference;
        //chprintf((BaseSequentialStream *)&SDU1, "%nAngle=%.2f \r\n", angle);

        if(fabs(angleError) < ROTATION_ERROR_THRESHOLD) {
        	angleSpeed = 0;
        } else {
        	// Calculate angleUp
            angleUp = angleKp*angleError;
            // Calculate angleUd
            angleUd = angleKd * (angleError - prevAngleError);
            // Calculate angleUi with an ARW
            if(!(fabs(angleUp + angleUi + angleKi*angleError) > angleUmax)) angleUi = angleUi + angleKi*angleError;

            angleSpeed = angleUp + angleUd + angleUi; // loop
            prevAngleError = angleError;
        }

        //chprintf((BaseSequentialStream *)&SDU1, "%nspeed(cm)=%.2f \r\n", speed);

        angleSpeed = angleSpeed * 1000/13; //convert from cm/s to steps/s

//        if(count == 10) {
//        	chprintf((BaseSequentialStream *)&SDU1, "%nAngleSpeed=%.2f \r\n", angleSpeed);
//        	count = 0;
//        } else {
//        	++count;
//        }


        //applies the speed from the PI regulator
		right_motor_set_speed(ROTATION_COEFF*angleSpeed);
		left_motor_set_speed(-ROTATION_COEFF*angleSpeed);

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

