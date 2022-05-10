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

#include <leds.h> ///////////////////////////////////////////

//#define PERP_EPUCK
#define PERP_MAX_SPEED 615 // steps/s

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
#ifndef PERP_EPUCK
    while (true) {
    	set_led(LED7,1); //////////////////////////////////////////
    	while (get_radar_state() == Detect) {
    		radar_measure_speed();
    	}
    	set_led(LED7,0); ///////////////////////////////////////////
    	enableMicrophone();

    	//start the pid thread
    	pi_regulator_start();
    	//start the siren thread
    	siren_start();

    	while (get_radar_state() == Chase) {
    		//chprintf((BaseSequentialStream *)&SD3, "%nChase mode is active \r\n"); do stuff
    	}
    }
#else
    right_motor_set_speed(PERP_MAX_SPEED);
    left_motor_set_speed(PERP_MAX_SPEED);
#endif
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}

