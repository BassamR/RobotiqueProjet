#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/audio_thread.h>
#include <spi_comm.h> //Leds work even withou that
#include <leds.h>
#include <siren.h>

static bool siren_freq=1; //start at 1 with the low freq
static THD_WORKING_AREA(waSirenThd, 128);
static THD_FUNCTION(SirenThd, arg) {
	chRegSetThreadName("siren thread");
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
		chThdSleepMilliseconds(500); //make it yield?
	}
}

void siren_start(void){
	//start spi communication for the rgb leds
	//spi_comm_start();		// removed it from here beacause needs to be called at startup to turn off the leds after each external reset, if we make our own reset add it back here!
	//start the speakers
	dac_start();
	chThdCreateStatic(waSirenThd, sizeof(waSirenThd), NORMALPRIO, SirenThd, NULL);

}

