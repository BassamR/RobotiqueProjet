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

#include <communications.h>
#include <arm_math.h>

#include <selector.h>

#define EPUCK_SPEED_FAST 	8 // cm/s
#define EPUCK_SPEED_SLOW 	5 // cm/s

static void serial_start(void) {
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void) {
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void) {

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();

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
