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
#include <audio/audio_thread.h>
#include <selector.h>

#include <spi_comm.h>
#include <leds.h>

#define EPUCK_SPEED_FAST 	8 // cm/s
#define EPUCK_SPEED_SLOW 	5 // cm/s

int main(void) {

    halInit();
    chSysInit();
    mpu_init();

    //init leds, to make sure they stay turned off
    spi_comm_start();
    //inits the motors
    motors_init();
	//start the speakers
	dac_start();

    /* Infinite loop. */
    while (1) {
    	switch(get_selector()) {
    	case 1: //fast with noise
    		dac_play(440);
    		right_motor_set_speed(EPUCK_SPEED_FAST*1000/13);
    		left_motor_set_speed(EPUCK_SPEED_FAST*1000/13);
    		break;
    	case 2: //slow with noise
    		dac_play(440);
    		right_motor_set_speed(EPUCK_SPEED_SLOW*1000/13);
    		left_motor_set_speed(EPUCK_SPEED_SLOW*1000/13);
    		break;
    	case 3: //fast without noise
    		dac_stop();
    		right_motor_set_speed(EPUCK_SPEED_FAST*1000/13);
    		left_motor_set_speed(EPUCK_SPEED_FAST*1000/13);
    		break;
    	case 4: //slow without noise
    		dac_stop();
    		right_motor_set_speed(EPUCK_SPEED_SLOW*1000/13);
    		left_motor_set_speed(EPUCK_SPEED_SLOW*1000/13);
    		break;
    	default: //do nothing by default
    		dac_stop();
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    		break;
    	}

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
