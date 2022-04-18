#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

#include <math.h>

#define SOUND_SPEED 	34300 	// cm/s
#define EPUCK_RADIUS    2.675f  // cm

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

// Extra static variables and functions
static int positionInBuffer = 0;
static int sendToComputer = 0;

static float getAngleFromSource(void);
static void alignRobot(float angle);

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	@params:
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*
*	@return: none
*/
void processAudioData(int16_t *data, uint16_t num_samples) {
	//Fill the samples buffers to reach 1024 samples, then compute FFTs

	for(int i = 0; i < (int)num_samples/4; ++i) {
		micRight_cmplx_input[positionInBuffer] = data[4*i];
		micLeft_cmplx_input[positionInBuffer] = data[4*i + 1];
		micBack_cmplx_input[positionInBuffer] = data[4*i + 2];
		micFront_cmplx_input[positionInBuffer] = data[4*i + 3];

		micRight_cmplx_input[positionInBuffer + 1] = 0; //imaginary part = 0
		micLeft_cmplx_input[positionInBuffer + 1] = 0;
		micBack_cmplx_input[positionInBuffer + 1] = 0;
		micFront_cmplx_input[positionInBuffer + 1] = 0;

		positionInBuffer += 2;

		if(positionInBuffer >= 2*FFT_SIZE) break; //finished filling up the 2*1024 buffer
	}

	if(positionInBuffer >= 2*FFT_SIZE) {
		//reset positionInBuffer
		positionInBuffer = 0;

		//compute FFT
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		//compute magnitude of FFT
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		//call functions that need audio data
		float robotAngle = getAngleFromSource();
		//alignRobot(angle);
		chprintf((BaseSequentialStream *)&SDU1, "%nAngle=%.2f \r\n", robotAngle);

		//send to computer
		if(sendToComputer == 5) {
			chBSemSignal(&sendToComputer_sem);
			sendToComputer = 0;
		} else {
			++sendToComputer;
		}

	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

/*
*	Returns buffer pointer of microphone
*
*	@params: BUFFER_NAME_t name		microphone name
*	@return: float*					microphone buffer
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

/*
*	Calculates angle of robot (in deg) relative to a noise source using 2 microphones (left and right)
*	(might need to use back and front mics, not sure)
*
*	@params: none
*	@return: none
*/
static float getAngleFromSource(void) {
	//get frequency of sound (ie frequency with the highest FFT amplitude)
	float maxLeftOutput = 0;
	int maxFreq = 0;
	//run through half the array because FFT gives positive part on [0,512], and negative part on [512, 1024]
	for(int i = 0; i < FFT_SIZE/2; ++i) {
		if(micLeft_output[i] > maxLeftOutput) {
			maxLeftOutput = micLeft_output[i]; //contains the biggest amplitude of the FFT
			maxFreq = i; //contains the index at which we have the biggest amplitude
		}
	}

	// Calculate time shift at max amplitude frequency using FFT argument
	float micLeftArg = atan2f(micLeft_cmplx_input[2*maxFreq + 1], micLeft_cmplx_input[2*maxFreq]);
	float micRightArg = atan2f(micRight_cmplx_input[2*maxFreq + 1], micRight_cmplx_input[2*maxFreq]);
	float timeShift = FFT_SIZE * abs(micLeftArg - micRightArg) / (2 * M_PI * maxFreq); //time difference of arrival;

	// Calculate angle in deg
	float cosineArgument = SOUND_SPEED * timeShift/(2*EPUCK_RADIUS);
	if(cosineArgument > 1) cosineArgument = 1;
	if(cosineArgument < -1) cosineArgument = -1; //safety to avoid taking arccos of undefined values
	float angle = acosf(cosineArgument) * 180/M_PI;

	return angle;
}

/*
*	Rotates the robot to align it with noise source
*
*	@params: angle from noise source (in deg)
*	@return: none
*/
static void alignRobot(float angle) {

}
