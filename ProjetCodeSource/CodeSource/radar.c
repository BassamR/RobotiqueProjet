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
#include <radar.h>

// General constants
#define INIT_TIME		10000000
#define SENSITIVITY 	20
#define MAX_COUNT 		250000

// Static variables
static enum State current_state = Detect;
static uint16_t count = 0;
static uint16_t reference = 0;
static uint16_t dist_to_perp = 0;

/*
*	aly will type it
*
*	@params: none
*	@return: none
*/
static void set_reference(void) {  //you need to skip a bunch of measurments at startup because theyre not correct
	reference = 0;
	for (uint32_t i = 0; i <= INIT_TIME; ++i) { //distance is only refreshed every 100ms if we add the print the correct values can be obtained after a bit under 10 cycles, without the print need to wait for at least 10000000
		reference = VL53L0X_get_dist_mm();
		//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", reference); //this needs to be kept here otherwise the tof has no time to get new measurments since it takes 100 ms
	} //run 10buffer cycles
	//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", reference);

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
	if (distance <= reference - SENSITIVITY) {
		++count;
		dist_to_perp = distance;
		//reference=distance; this should only be used if the object stays for a while infront of the tof to set a new reference
		//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", count);
	} else if (distance >= dist_to_perp + SENSITIVITY) {
		if (count <= MAX_COUNT) { //case object was fast
			//call function to estimate speed then activate the lights
			current_state = Chase;
		}
		//reference=distance;
		dist_to_perp = distance;
		count = 0;
	}
}
