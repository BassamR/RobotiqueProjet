/*
* @name		audio_processing.h
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

//Convert microphone data into readable arrays
void processAudioData(int16_t *data, uint16_t num_samples);

//Returns the pointer to the BUFFER_NAME_t buffer asked
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

//Returns the angle (in deg) from the center of the robot and a noise source
int16_t getAngleFromSource(void);

//Enables microphone computations
void enableMicrophone(void);

#endif /* AUDIO_PROCESSING_H */
