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

#include <spi_comm.h> //works even without that
#include <leds.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <audio/audio_thread.h>

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


static bool siren_freq=1; //start at 1 with the low freq
static THD_WORKING_AREA(waRGBThd, 128);
static THD_FUNCTION(RGBThd, arg) {
	chRegSetThreadName("RGB leds thread");
	(void)arg;
	set_rgb_led(LED4,RGB_MAX_INTENSITY,0,0);  //open the red leds first to create a delay
	set_rgb_led(LED8,RGB_MAX_INTENSITY,0,0);
	while(1){
		siren_freq=!siren_freq;
		dac_stop();
		dac_start();
		toggle_rgb_led(LED2, BLUE_LED,RGB_MAX_INTENSITY);
		toggle_rgb_led(LED4, RED_LED,RGB_MAX_INTENSITY);
		toggle_rgb_led(LED6, BLUE_LED,RGB_MAX_INTENSITY);
		toggle_rgb_led(LED8, RED_LED,RGB_MAX_INTENSITY);

		if (siren_freq==false){
			dac_play(705);
		}
		else{
			dac_play(933);
		}

		//uint16_t freq_buffer[2]={705,933};
		//dac_play_buffer(freq_buffer,2,1,NULL);
		chThdSleepMilliseconds(500); //make it yield?
	}
}


int main(void){

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //start spi communication for the rgb leds
    spi_comm_start();
    //start the ToF
    VL53L0X_start();
    //start the speakers
    dac_start();

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
    				chprintf((BaseSequentialStream *)&SD3, "%u \r\n", count);
    				chThdCreateStatic(waRGBThd, sizeof(waRGBThd), NORMALPRIO, RGBThd, NULL);
    				// here we should break from the while loop because the detection state will be over start chase state
    				current_state=Chase;
    			}
    			//reference=distance;
    			dist_to_perp=distance;
    			count=0;
    		}
    	}

    	while (current_state==Chase){
    		chprintf((BaseSequentialStream *)&SD3, "%nChase mode is active \r\n");
    	}
    }
}
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

uint16_t set_ref(void){  //you need to skip a bunch of measurments at startup because theyre not correct
	uint16_t ref = 0;
	for (int i=0; i<=1000;++i){
		ref= VL53L0X_get_dist_mm();
		// how to use the dsp instructions??
		chprintf((BaseSequentialStream *)&SD3, "%u \r\n", ref); //need to uncomment this line for ref to keep that final value for some weird reason
	} //run 10buffer cycles
	//uint16_t error= 10;
	//while(abs(ref-VL53L0X_get_dist_mm())<= error) {
		//ref=VL53L0X_get_dist_mm();

	//}
	//chprintf((BaseSequentialStream *)&SD3, "%u \r\n", ref);
	return ref;
}



