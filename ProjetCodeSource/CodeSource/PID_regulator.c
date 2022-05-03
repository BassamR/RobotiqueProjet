#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>		//is it normal to include main here ?? we did that in the tp
#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <audio_processing.h>

// General constants
#define ROTATION_COEFF 				0.4
#define ROTATION_ERROR_THRESHOLD	0 // degrees
#define ERROR_THRESHOLD 			0.25 //minimum error we want to detect is 0.2cm

#define TS 0.015		// sampling period in s, either multiply with ts or with the variable time
#define TF 0.075		// filter frequency, in the range 2*ts - 10*ts

#define U_MAX  		1000 	// steps/s
#define U_MAX_CM	13		// cm/s

//PID variables for distance regulation
#define DISTANCE_KP 	1.2f
#define DISTANCE_KI 	0.0045f
#define DISTANCE_REF 	10 // cm

static int16_t distanceSpeed = 0;
static float distanceError = 0; // = ref - distance, here the declaration = initial error

static float distanceUi = 0; // initial condition for integrator term
static float distanceUp = 0;

// PID variables for angle regulation
#define ANGLE_KP 		13.0f
#define ANGLE_KI 		10.0f
#define ANGLE_KD 		5.0f
#define ANGLE_REF 		90 //deg
#define ANGLE_UMAX 		13

static int16_t angleSpeed = 0; //what the rotational PID corrects
static int16_t angleError = 0; // = ref - distance, here the declaration = initial error
static float prevAngleError = 0;

static float angleUi = 0;
static float angleUp = 0;
static float angleUd = 0;

//static int count = 0;

// Thread declaration

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1) {
        time = chVTGetSystemTime();

        // Rotation PID
        //float angle = getAngleFromSource();
        angleError = getAngleFromSource() - ANGLE_REF;
//		if(count == 10) {
//			chprintf((BaseSequentialStream *)&SDU1, "%nAngleError=%.2f \r\n", angle);
//			count = 0;
//		} else {
//			++count;
//		}

        if(fabs(angleError) < ROTATION_ERROR_THRESHOLD) {
        	angleSpeed = 0;
        } else {
        	// Calculate angleUp
            angleUp = ANGLE_KP*angleError;
            // Calculate angleUd
            angleUd = (TF*angleUd + ANGLE_KD * (prevAngleError - angleError))/(TF + TS); //instead of ts, use real system time maybe
            // Calculate angleUi with an ARW
            if(!(fabs(angleUp + angleUi + ANGLE_KI*angleError) > ANGLE_UMAX)) angleUi = angleUi + ANGLE_KI*angleError;

            angleSpeed = angleUp + angleUd + angleUi; // loop
            //angleSpeed = angleUp + angleUi; // loop
            prevAngleError = angleError;
            //chprintf((BaseSequentialStream *)&SDU1, "%nspeed(cm)=%.2f \r\n", angleSpeed);

            //avoid command saturation
            if(angleSpeed > U_MAX) angleSpeed = U_MAX;
            if(angleSpeed < -U_MAX) angleSpeed = -U_MAX;
        }
        //angleSpeed = angleSpeed * 1000/13; //convert from cm/s to steps/s

        // Distance PID
        distanceError = VL53L0X_get_dist_mm()/10 - DISTANCE_REF;
		if(fabs(distanceError) < ERROR_THRESHOLD) {
			distanceSpeed = 0;
		} else {
			distanceUp = DISTANCE_KP*distanceError;
			if(!(fabs(distanceUp + distanceUi + DISTANCE_KI*distanceError) > U_MAX_CM)) distanceUi = distanceUi + DISTANCE_KI*distanceError; // corresponds to current ui + ARW

			distanceSpeed = distanceUp + distanceUi; // loop
			//chprintf((BaseSequentialStream *)&SDU1, "%nspeed(cm)=%.2f \r\n", distanceSpeed);

			if(distanceSpeed > 7) distanceSpeed = 7;
			if(distanceSpeed < -7) distanceSpeed = -7;
		}
		distanceSpeed = distanceSpeed * 1000/13; //convert from cm/s to steps/s

//        if(count == 10) {
//        	chprintf((BaseSequentialStream *)&SDU1, "%nAngleSpeed=%.2f \r\n", angleSpeed);
//        	count = 0;
//        } else {
//        	++count;
//        }

        //applies the speed from the PI regulator
		right_motor_set_speed(distanceSpeed + angleSpeed);
		left_motor_set_speed(distanceSpeed - angleSpeed);

        //100Hz
		//chprintf((BaseSequentialStream *)&SD3, "%ntime is %d \r\n",chVTGetSystemTime()-time);
		//chThdSleepMilliseconds(10);
		chThdSleepUntilWindowed(time, time + MS2ST(15));
    }
}

void pid_regulator_start(void){
	//inits the motors
	motors_init();
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+2, PiRegulator, NULL);
}

