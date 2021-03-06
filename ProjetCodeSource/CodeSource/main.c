/*
* @name		main.c
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

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
#include <pi_regulator.h>
#include <communications.h>

#include <audio/audio_thread.h>
#include <spi_comm.h>
#include <siren.h>
#include <pi_regulator.h>
#include <radar.h>

static void serial_start(void) {
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3
}


int main(void) {
	//initialize main systems
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts spi communication (for RGB leds)
    spi_comm_start();
    //start the ToF Thread
    VL53L0X_start();
    //start the mic thread
    mic_start(&processAudioData);
	//start the motor thread
	motors_init();
	//start the speakers
	dac_start();
    //start the radar (not a thread)
	radar_start();

    while (true) {
    	while (get_radar_state() == Detect) {
    		radar_measure_speed();
    	}

    	//enables microphone functionalities
    	enableMicrophone();
    	//start the pid thread
    	pi_regulator_start();
    	//start the siren thread
    	siren_start();

    	while(get_radar_state() == Chase) {
    		chThdSleepMilliseconds(1000);
    	}
    }
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
