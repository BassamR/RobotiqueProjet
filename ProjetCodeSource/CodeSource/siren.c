/*
* @name		siren.c
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/audio_thread.h>
#include <spi_comm.h> //Leds work even without that
#include <leds.h>
#include <siren.h>

// Static variables
static bool siren_freq = 1; //start at 1 with the low freq

// Siren thread declaration
static THD_WORKING_AREA(waSirenThd, 128);
static THD_FUNCTION(SirenThd, arg) {
	chRegSetThreadName("siren thread");
	(void)arg;

	set_rgb_led(LED4, RGB_MAX_INTENSITY, 0, 0); //open the red leds first to create a delay
	set_rgb_led(LED8, RGB_MAX_INTENSITY, 0, 0);

	while(1) {
		siren_freq =! siren_freq;
		dac_stop();
		dac_start(); //stop first frequency then play next one
		toggle_rgb_led(LED2, BLUE_LED, RGB_MAX_INTENSITY);
		toggle_rgb_led(LED4, RED_LED, RGB_MAX_INTENSITY);
		toggle_rgb_led(LED6, BLUE_LED, RGB_MAX_INTENSITY);
		toggle_rgb_led(LED8, RED_LED, RGB_MAX_INTENSITY);

		if(siren_freq == false) {
			dac_play(705);
		} else {
			dac_play(933);
		}

		chThdSleepMilliseconds(500);
	}
}

/*
*	Creates siren thread
*
*	@params: none
*	@return: none
*/
void siren_start(void) {
	chThdCreateStatic(waSirenThd, sizeof(waSirenThd), NORMALPRIO, SirenThd, NULL);
}
