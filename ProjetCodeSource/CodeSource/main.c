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

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

//time measurement
//#define MEASURE_TIME

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

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();
#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE) {

        	static complex_float convertedCmplxArray[FFT_SIZE]; //already a pointer
        	//equivalent to static complex_float* convertedCmplxArray; except above we reserve FFT_SIZE slots and here we don't
        	//converts bufferCmplxInput to an array of half the size, each cell containing re(z)+i*im(z)
        	// bufferCmplxInput is of the form [re1, im1, re2, im2, ...]

        	// convert array to correct form
//        	for(int i = 0; i < FFT_SIZE; ++i) {
//        		convertedCmplxArray[i].real = bufferCmplxInput[2*i];
//        		convertedCmplxArray[i].imag = bufferCmplxInput[2*i + 1];
//        	}

#ifdef MEASURE_TIME
        	volatile uint16_t time = 0;
        	chSysLock();
        	//reset the timer counter
        	GPTD12.tim->CNT = 0;

        	//-> Functions to measure <-//
        	//call unoptimized FFT
        	//doFFT_c(FFT_SIZE, convertedCmplxArray);
            doFFT_optimized(FFT_SIZE, bufferCmplxInput);

        	time = GPTD12.tim->CNT;
        	chSysUnlock();
        	chprintf((BaseSequentialStream *)&SDU1, "time=%dus\n", time);
#else
        	//call unoptimized FFT
        	//doFFT_c(FFT_SIZE, convertedCmplxArray);
            doFFT_optimized(FFT_SIZE, bufferCmplxInput);
#endif

        	//reconvert convertedCmplxArray to an array of size 2*FFT_SIZE for the rest of the code
//        	for(int i = 0; i < FFT_SIZE; ++i) {
//        		bufferCmplxInput[2*i] = convertedCmplxArray[i].real;
//        		bufferCmplxInput[2*i + 1] = convertedCmplxArray[i].imag;
//        	}

            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);

        }
#endif  /* SEND_FROM_MIC */
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
