/*
* @name		audio_processing.c
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>

/*
 * Uncomment to return the real angle in [-pi, pi]
 * Else, will return an angle in [0, pi] but won't detect the side the sound is coming from
 */
//#define COMPUTE_SIGNED_ANGLE

// General Constants
#define SOUND_SPEED 			34300 	// cm/s
#define MIC_DISTANCE    		6.0f  	// cm
#define AMPLITUDE_THRESHOLD		13000
#define FREQ_THRESHOLD 			70
#define FREQ_MIN				5
#define FREQ_MAX				40
#define FREQ_RESOLUTION			15.625f

// 2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
// Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

// Double Buffering arrays
static float micLeftInputDB[2 * FFT_SIZE];
static float micRightInputDB[2 * FFT_SIZE];

static float micLeftOutputDB[FFT_SIZE];
static float micRightOutputDB[FFT_SIZE];

#ifdef COMPUTE_SIGNED_ANGLE
static float micFrontInputDB[2 * FFT_SIZE];
static float micBackInputDB[2 * FFT_SIZE];

static float micFrontOutputDB[FFT_SIZE];
static float micBackOutputDB[FFT_SIZE];
#endif

// Extra static variables and functions
static int16_t positionInBuffer = 0;
static bool micEnable = false;

//#define SEND_TO_COMPUTER

#ifdef SEND_TO_COMPUTER
static int sendToComputer = 0;
#endif

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz).
*	Fills the samples buffers to reach 1024 samples, then computes their FFTs
*
*	@params:
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*
*	@return: none
*/
void processAudioData(int16_t *data, uint16_t num_samples) {
	if(!micEnable) return;

	// Fill input arrays with mic samples
	for(int16_t i = 0; i < (int16_t)num_samples/4; ++i) {
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

		//compute FFT of left and right mic then compute its magnitude
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		// Fill out double buffering arrays
		for(int i = 0; i < 2*FFT_SIZE; ++i) {
			micLeftInputDB[i] = micLeft_cmplx_input[i];
			micRightInputDB[i] = micRight_cmplx_input[i];
		}

		for(int i = 0; i < FFT_SIZE; ++i) {
			micLeftOutputDB[i] = micLeft_output[i];
			micRightOutputDB[i] = micRight_output[i];
		}

#ifdef COMPUTE_SIGNED_ANGLE // No need to waste resources on back and front mics when unnecessary
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		for(int i = 0; i < 2*FFT_SIZE; ++i) {
			micFrontInputDB[i] = micFront_cmplx_input[i];
			micBackInputDB[i] = micBack_cmplx_input[i];
		}

		for(int i = 0; i < FFT_SIZE; ++i) {
			micFrontOutputDB[i] = micFront_output[i];
			micBackOutputDB[i] = micBack_output[i];
		}
#endif

		//send to computer
#ifdef SEND_TO_COMPUTER
		if(sendToComputer == 5) {
			//chBSemSignal(&sendToComputer_sem);
			sendToComputer = 0;

			//call functions that need audio data
			int16_t robotAngle = getAngleFromSource();
			//chprintf((BaseSequentialStream *)&SDU1, "%nAngle=%.7d \r\n", robotAngle);
			chprintf((BaseSequentialStream *)&SDU1, "%nMaxFreqAmplitude=%f \r\n", maxAmplitudeTest);

//			int maxFreqLeft = 0;
//			float maxLeftOutput = 0;
//			for(int i = 5; i < FFT_SIZE/2; ++i) {
//				//start from i=5 as to not consider low freq
//				if(micLeft_output[i] > maxLeftOutput) {
//					maxLeftOutput = micLeft_output[i]; //contains the biggest amplitude of the FFT
//					maxFreqLeft = i; //contains the index at which we have the biggest amplitude
//				}
//			}
//
//			chprintf((BaseSequentialStream *)&SDU1, "%nFreqIndex=%.7d \r\n", maxFreqLeft);

		} else {
			++sendToComputer;
		}
#endif

	}
}

/*
*	Enables microphone processing calculations (FFT, buffer filling)
*
*	@params: none
*	@return: none
*/
void enableMicrophone(void) {
	micEnable = true;
	return;
}

/*
*	Returns buffer pointer of microphone
*
*	@params: BUFFER_NAME_t name		microphone name
*	@return: float*					microphone buffer
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name) {
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
*	Optional: use front and back mics to return a signed angle
*
*	@params: none
*	@return: int16_t angle from source in deg
*/
int16_t getAngleFromSource(void) {
	static float prevAngle = 90;

	float maxLeftOutput = 0;
	float maxRightOutput = 0;
	int16_t maxFreqLeft = 0;
	int16_t maxFreqRight = 0;
	int16_t maxFreq = 0;

#ifdef COMPUTE_SIGNED_ANGLE
	float maxFrontOutput = 0;
	float maxBackOutput = 0;
#endif

	// Get frequency of sound (ie frequency with the highest FFT amplitude)

	// Run through half the array because FFT gives positive part on [0,512], and negative part on [512, 1024]
	for(int16_t i = FREQ_MIN; i < FREQ_MAX; ++i) {
		if(micLeftOutputDB[i] > maxLeftOutput) {
			maxLeftOutput = micLeftOutputDB[i]; //contains the biggest amplitude of the FFT
			maxFreqLeft = i; //contains the index at which we have the biggest amplitude
		}

		if(micRightOutputDB[i] > maxRightOutput) {
			maxRightOutput = micRightOutputDB[i];
			maxFreqRight = i;
		}

#ifdef COMPUTE_SIGNED_ANGLE
		if(micFrontOutputDB[i] > maxFrontOutput) maxFrontOutput = micFrontOutputDB[i];
		if(micBackOutputDB[i] > maxBackOutput) maxBackOutput = micBackOutputDB[i];
#endif
	}

	// If the biggest amplitude is smaller than a certain threshold (ie a clear sound isnt being played), do nothing
	if((maxLeftOutput < AMPLITUDE_THRESHOLD) || (maxRightOutput < AMPLITUDE_THRESHOLD)) {
		return prevAngle; //this makes the robot memorize current noise angle once noise shuts off
	}

	// Choose the frequency coming from the mic closest to the sound source
	if(maxFreqLeft >= maxFreqRight) maxFreq = maxFreqLeft;
	else maxFreq = maxFreqRight;

	// Calculate time shift at max amplitude frequency using FFT argument
	float micLeftArg = atan2f(micLeftInputDB[2*maxFreq + 1], micLeftInputDB[2*maxFreq]);
	float micRightArg = atan2f(micRightInputDB[2*maxFreq + 1], micRightInputDB[2*maxFreq]);
	float timeShift = (micRightArg - micLeftArg) / (2 * M_PI * maxFreq * FREQ_RESOLUTION); //time difference of arrival;
	//chprintf((BaseSequentialStream *)&SD3, "%ntimeShift=%f \r\n", timeShift);

	// Calculate angle in deg
	float cosineArgument = SOUND_SPEED * timeShift/MIC_DISTANCE;
	//skip next angle calculation if cosineArgument is too big (to avoid undefined arccos)
	if((cosineArgument > 1) || (cosineArgument < -1)) {
		return prevAngle;
	}

	float angle = acosf(cosineArgument) * 180/M_PI;

#ifdef COMPUTE_SIGNED_ANGLE
	//front-back, if negative then sound comes from back, else front
	if(maxFrontOutput < maxBackOutput) angle = -angle;
#endif

	prevAngle = angle;

	//chprintf((BaseSequentialStream *)&SD3, "%nAngle=%.2f \r\n", angle);

	return angle;
}
