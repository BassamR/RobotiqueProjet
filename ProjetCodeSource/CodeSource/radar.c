/*
* @name		radar.c
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <radar.h>

// General constants
#define INIT_COUNTS		10000000
#define MAX_COUNT 		2811162 	//2 second count
#define SENSITIVITY		20			//mm
#define SECOND_COUNT	1405581 	//1 seconds in counts equivalent
#define OBJECT_LENGTH	7.3f 		//epuck diameter
#define MAX_SPEED		7 			//cm/s, also decides whether to use max speed in cm/s or in counts/s

// Static variables
static enum State current_state = Detect;
static uint32_t count = 0;
static uint16_t reference = 0;
static uint16_t dist_to_perp = 0;


/* Uncomment to send measured speed to computer*/
//#define SEND_SPEED_TO_COMPUTER

/*
*	Sets distance to the reference target, radar then captures objects passing between the reference and the robot
*
*	@params: none
*	@return: none
*/
static void set_reference(void) {
	for (uint32_t i = 0; i <= INIT_COUNTS; ++i) {
		reference = VL53L0X_get_dist_mm();
	}

	return;
}

/*
*	Initializes radar with distance reference and init state
*
*	@params: none
*	@return: none
*/
void radar_start(void) {
	current_state = Detect;
    set_reference();
	dist_to_perp = reference;
	count = 0;
}

/*
*	Getter for current state (Chase or Detect) of the robot
*
*	@params: none
*	@return: enum State current state
*/
enum State get_radar_state(void) {
	return current_state;
}

/*
*	Changes state (Detect -> Chase) when ToF detects a sufficiently fast object
*
*	@params: none
*	@return: none
*/
void radar_measure_speed(void) {
	uint16_t distance = VL53L0X_get_dist_mm();
	if(distance <= reference - SENSITIVITY) {
		++count;
		dist_to_perp = distance;
	}
#ifdef MAX_SPEED
	else if(distance >= dist_to_perp+SENSITIVITY) {
		float speed = OBJECT_LENGTH/((float)count/(float)SECOND_COUNT); //en cm/s
		if (speed >= (float)MAX_SPEED) {
			current_state = Chase;
		}
#ifdef SEND_SPEED_TO_COMPUTER
		chprintf((BaseSequentialStream *)&SDU1, "Epuck measured speed is: %f \r\n", speed);
#endif
		dist_to_perp = distance;
		count = 0;
	}
#else

	else if(distance >= dist_to_perp + SENSITIVITY) {
		if(count <= MAX_COUNT) {
			current_state = Chase;
		}
		float time_pass = (float)count/(float)SECOND_COUNT;
#ifdef SEND_SPEED_TO_COMPUTER
		chprintf((BaseSequentialStream *)&SDU1, "Epuck measured counts are: %f \r\n", time_pass);
#endif
		dist_to_perp = distance;
		count = 0;
	}

#endif

}
