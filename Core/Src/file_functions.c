/*
 * file_functions.c
 *
 *  Created on: Apr 16, 2021
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*--------------------------------*/
char data_char[5];
uint8_t data_uint[4];

void uint32toArray(uint32_t v, uint8_t *data) {
	//little endian
	data[0] = (v & 0xff);
	data[1] = (v & 0xff00) >> 8;
	data[3] = (v & 0xff0000) >> 16;
	data[4] = (v & 0xff000000) >> 24;
}

void uint16toArray(uint32_t v, uint8_t *data) {
	//little endian
	data[0] = (v & 0xff);
	data[1] = (v & 0xff00) >> 8;
}

void CreateWaveFile(Sample *sample) {
	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */
	//Create file
	if (f_open(&(sample->fichier), strcat(sample->nom, ".WAV"),
	FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		Error_Handler();
	} else {
		uint8_t wtext[44] = { 0 };
		res = f_write(&(sample->fichier), wtext, 44, (void*) &byteswritten);
		if ((byteswritten == 0) || (res != FR_OK)) {
			Error_Handler();
		} else {
			f_sync(&(sample->fichier));
		}
	}
}

void AddData(Sample *sample, uint8_t *data, uint32_t nbytes) {

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */
	res = f_write(&(sample->fichier), data, nbytes, (void*) &byteswritten);
	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {
		sample->numsamples += nbytes
				/ (sample->numchannels * sample->samplelength);
		f_sync(&(sample->fichier));
	}
}

void SetHeader(Sample *sample) {

	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten; /* File write/read counts */

	//Write to the text file
	uint8_t wtext[44];

	sprintf(data_char, "RIFF");
	memcpy(wtext, (uint8_t*) data_char, 4);

	//filesize
	uint32toArray(
			(uint32_t) sample->numchannels * sample->numsamples
					* sample->samplelength + 36, data_uint);
	memcpy(wtext + 4, data_uint, 4);

	//file format
	sprintf(data_char, "WAVE");
	memcpy(wtext + 8, (uint8_t*) data_char, 4);

	//Bloc format audio
	sprintf(data_char, "fmt ");
	memcpy(wtext + 12, (uint8_t*) data_char, 4);
	//nombre d'octets du bloc
	uint32toArray((uint32_t) 16, data_uint);
	memcpy(wtext + 16, data_uint, 4);
	//audio format
	uint16toArray((uint16_t) 1, data_uint);
	memcpy(wtext + 20, data_uint, 2);
	//nbr canaux
	uint16toArray((uint16_t) sample->numchannels, data_uint);
	memcpy(wtext + 22, data_uint, 2);
	//freq
	uint32toArray((uint32_t) sample->samplerate, data_uint);
	memcpy(wtext + 24, data_uint, 4);
	//bytepersec
	uint32toArray(
			(uint32_t) sample->samplerate * sample->samplelength
					* sample->numchannels, data_uint);
	memcpy(wtext + 28, data_uint, 4);
	//byteperbloc
	uint16toArray((uint16_t) sample->samplelength * sample->numchannels,
			data_uint);
	memcpy(wtext + 32, data_uint, 2);
	//bitspersample
	uint16toArray((uint16_t) 8 * sample->samplelength, data_uint);
	memcpy(wtext + 34, data_uint, 2);

	//Bloc donnees
	sprintf(data_char, "data");
	memcpy(wtext + 36, (uint8_t*) data_char, 4);
	//databytes
	uint32toArray(
			(uint32_t) sample->numchannels * sample->numsamples
					* sample->samplelength, data_uint);
	memcpy(wtext + 40, data_uint, 4);

	sample->fichier.fptr = 0;

	res = f_write(&(sample->fichier), wtext, 44, (void*) &byteswritten);
	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {
		f_close(&(sample->fichier));
	}
}

void MountSD(void) {
	FRESULT res; /* FatFs function common result code */
	uint8_t rtext[_MAX_SS];/* File read buffer */
	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		Error_Handler();
	} else {
		res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext));
		if (res != FR_OK) {
			Error_Handler();
		}
	}
}
