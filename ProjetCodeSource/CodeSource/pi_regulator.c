/*
* @name		pi_regulator.c
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#include "pi_regulator.h"
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <math.h>

#include <sensors/VL53L0X/VL53L0X.h>
#include <motors.h>
#include <audio_processing.h>

// General constants
#define ERROR_THRESHOLD 			0.0 	// cm

#define TS 							0.01	// sampling period in s, either multiply with ts or with the variable time
#define TF 							0.00	// filter frequency, in the range 2*ts - 10*ts

#define U_MAX  						1000 	// steps/s
#define U_MAX_CM					13		// cm/s

#define DISTANCE_SPEED_MAX 			8 		// cm/s

// PID variables for distance regulation
#define DISTANCE_KP 				1.65f
#define DISTANCE_KI 				0.0045f
#define DISTANCE_REF 				10 		// cm

static int16_t distanceSpeed = 0;
static int16_t distanceError = 0;

static float distanceUi = 0;
static float distanceUp = 0;

// PID variables for angle regulation
#define ANGLE_KP 					12.0f
#define ANGLE_KI 					3.0f
#define ANGLE_KD 					0.0f
#define ANGLE_REF 					90 //deg
#define ANGLE_UMAX 					13

static int16_t angleSpeed = 0;
static int16_t angleError = 0;
static float prevAngleError = 0;

static float angleUi = 0;
static float angleUp = 0;
static float angleUd = 0;

// PI thread declaration
static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {
    chRegSetThreadName("pi thread");
    (void)arg;

    systime_t time;

    while(1) {
        time = chVTGetSystemTime();

        // Rotation PID
        angleError = getAngleFromSource() - ANGLE_REF;

		// Calculate angleUp
		angleUp = ANGLE_KP*angleError;
		// Calculate angleUd
		angleUd = (TF*angleUd + ANGLE_KD * (prevAngleError - angleError))/(TF + TS);
		// Calculate angleUi with an ARW
		if(!(fabs(angleUp + angleUi + ANGLE_KI*angleError) > ANGLE_UMAX)) angleUi = angleUi + ANGLE_KI*angleError;

		angleSpeed = angleUp + angleUd + angleUi; // loop
		prevAngleError = angleError;

		// Avoid command saturation
		if(angleSpeed > U_MAX) angleSpeed = U_MAX;
		if(angleSpeed < -U_MAX) angleSpeed = -U_MAX;

        // Distance PID
        distanceError = VL53L0X_get_dist_mm()/10 - DISTANCE_REF;
		if(fabs(distanceError) < ERROR_THRESHOLD) {
			distanceSpeed = 0;
		} else {
			// Calculate distanceUp
			distanceUp = DISTANCE_KP*distanceError;
			// Calculate distanceUi with an ARW
			if(!(fabs(distanceUp + distanceUi + DISTANCE_KI*distanceError) > U_MAX_CM)) distanceUi = distanceUi + DISTANCE_KI*distanceError; // corresponds to current ui + ARW

			distanceSpeed = distanceUp + distanceUi; // loop

			// Cap the speed to avoid saturation
			if(distanceSpeed > DISTANCE_SPEED_MAX) distanceSpeed = DISTANCE_SPEED_MAX;
			if(distanceSpeed < -DISTANCE_SPEED_MAX) distanceSpeed = -DISTANCE_SPEED_MAX;
		}
		distanceSpeed = distanceSpeed * 1000/13; //convert from cm/s to steps/s

        //applies the speed from the PI regulator
		right_motor_set_speed(distanceSpeed + angleSpeed);
		left_motor_set_speed(distanceSpeed - angleSpeed);

        //100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/*
*	Starts the PI thread
*
*	@params: none
*	@return: none
*/
void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+2, PiRegulator, NULL);
}
