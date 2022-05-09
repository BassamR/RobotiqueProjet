#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
//#include "memory_protection.h"
//#include <usbcfg.h>
//#include <main.h>
//#include "msgbus/messagebus.h"
//#include "parameter/parameter.h"
#include <chprintf.h>
//#include <motors.h>
//#include <audio/microphone.h>

//#include <audio_processing.h>
//#include <fft.h>
//#include <communications.h>
//#include <arm_math.h>


//#include <spi_comm.h>
//#include <siren.h>
//#include <PID_regulator.h>
//#include <audio/audio_thread.h>
#include <radar.h>


#define SENSITIVITY 	20
#define MAX_COUNT 		250



static enum state current_state=0;
static uint16_t count = 0;
static uint16_t reference =0;
static uint16_t dist_to_perp=0;

static void set_reference(void) {  //you need to skip a bunch of measurments at startup because theyre not correct
	reference = 0;
	for (unsigned long int i=0; i<=10000000;++i){ //distance is only refreshed every 100ms if we add the print the correct values can be obtained after a bit under 10 cycles, without the print need to wait for at least 10000000
		reference= VL53L0X_get_dist_mm();
		//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", reference); //this needs to be kept here otherwise the tof has no time to get new measurments since it takes 100 ms
	} //run 10buffer cycles
	//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", reference);

	return;
}

enum state get_radar_state(void){
	return current_state;
}

void radar_start(void){
	current_state = Detect;
    set_reference(); //make a sensor stabilization loop
	dist_to_perp = reference; //needs to be initialized non zero
	count = 0;
}

void radar_measure_speed(void){
	uint16_t distance = VL53L0X_get_dist_mm();
	if (distance <= reference - SENSITIVITY) {
		++count;
		dist_to_perp=distance;
		//reference=distance; this should only be used if the object stays for a while infront of the tof to set a new reference
		//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", count);
	} else if (distance >= dist_to_perp + SENSITIVITY) {
		if (count <= MAX_COUNT) { //case object was fast \\pbl is this will acivate if the reference moves can make it more robust
			//call function to estimate speed then activate the lights
			// can activate all the chase threads here too still works, maybe we wont need the chase state stuff after all
			current_state = Chase;
		}
		//reference=distance;
		dist_to_perp = distance;
		count = 0;
	}
}





