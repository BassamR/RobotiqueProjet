#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>


#include <sensors/VL53L0X/VL53L0X.h>
#include <audio/audio_thread.h>
#include <PID_regulator.h>
#include <siren.h>

#define sensitivity 20
#define max_count 250

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

enum state {Detect,Chase};

int main(void){

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts spi communication (for RGB leds)
    spi_comm_start();
    //start the ToF
    VL53L0X_start();
    //start the mic
    mic_start(&processAudioData);


    while (true){

    	enum state current_state=Detect;
    	uint16_t reference = 0;
    	uint16_t dist_to_perp = 0;
    	reference = set_ref(); //make a sensor stabilization loop
    	dist_to_perp = reference; //needs to be initialized non zero
    	uint16_t count = 0;

    	while (current_state==Detect){
    		uint16_t distance = VL53L0X_get_dist_mm();
    		//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", distance);
    		if (distance<=reference-20){
    			++count;
    			dist_to_perp=distance;
    			//reference=distance; this should only be used if the object stays for a while infront of the tof to set a new reference
    			chprintf((BaseSequentialStream *)&SD3, "%u \r\n", count);
    		}else if (distance>=dist_to_perp+20){
    			if (count<=250) { //case object was fast \\pbl is this will acivate if the reference moves can make it more robust
    				//call function to estimate speed then activate the lights
    				// can activate all the chase threads here too still works, maybe we wont need the chase state stuff after all
    				current_state=Chase;
    			}
    			//reference=distance;
    			dist_to_perp=distance;
    			count=0;
    		}
    	}

    	//start the siren thread
    	//siren_start();
    	//start the pid thread
    	pid_regulator_start(); //careful where you place this, it should be called only once otherwise panics

    	while (current_state==Chase){
    		chprintf((BaseSequentialStream *)&SD3, "%nChase mode is active \r\n");

    	}
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}

uint16_t set_ref(void) {  //you need to skip a bunch of measurments at startup because theyre not correct
	uint16_t ref = 0;
	for (int i=0; i<=1000;++i){
		ref= VL53L0X_get_dist_mm();
		chprintf((BaseSequentialStream *)&SD3, "%u \r\n", ref); //need to uncomment this line for ref to keep that final value for some weird reason
	} //run 10buffer cycles

	return ref;
}



