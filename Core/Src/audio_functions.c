/*
 * audio_functions.c
 *
 *  Created on: 12 mai 2021
 *      Author: Thomas
 */


/* Includes */

#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_audio.h"
#include "stdio.h"
#include "math.h"
#include "file_functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*--------------------------------*/

void CreateSamples(Slot* slots){
	for (int i = 0; i < 8; ++i) {
		sprintf(slots[i].sample.nom,"SAMPLE%u",i+1);
		slots[i].sample.numchannels = 2;
		slots[i].sample.numsamples = 0;
		slots[i].sample.samplelength = 2;
		slots[i].sample.samplerate = 44100;
		CreateWaveFile(&(slots[i].sample));
		SetHeader(&(slots[i].sample));
	}
}

void CloseSamples(Slot* slots){
	for (int i = 0; i < 8; ++i) {
		SetHeader(&(slots[i].sample));
	}
}
