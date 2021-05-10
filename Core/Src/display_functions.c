/*
 * display_functions.c
 *
 *  Created on: 18 avr. 2021
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

void HomeScreen(void) {
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1,
	LCD_FB_START_ADDRESS + BSP_LCD_GetXSize() * BSP_LCD_GetYSize() * 4);
	BSP_LCD_DisplayOn();
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(LCD_COLOR_BG);
	BSP_LCD_SetFont(&Font24);

	uint8_t status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	if (status != TS_OK) {
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t*) "ERROR",
				CENTER_MODE);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80,
				(uint8_t*) "Touchscreen cannot be initialized", CENTER_MODE);
	} else {
		BSP_LCD_SetBackColor(LCD_COLOR_YELLOW);
		uint8_t pasX = BSP_LCD_GetXSize() / 4;
		uint8_t pasY = BSP_LCD_GetYSize() / 2;
		uint8_t pad = (pasX > pasY) ? pasY * 0.8 : pasX * 0.8;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 2; ++j) {
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
				BSP_LCD_FillRect((i+0.5)*pasX-pad/2, (j+0.5)*pasY-pad/2, pad, pad);
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_DrawRect((i+0.5)*pasX-pad/2, (j+0.5)*pasY-pad/2, pad, pad);
				BSP_LCD_DrawRect((i+0.5)*pasX-pad/2-1, (j+0.5)*pasY-pad/2-1, pad+2, pad+2);
				BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
				BSP_LCD_DisplayChar((i+0.5)*pasX-9,(j+0.5)*pasY-9,(uint8_t)49+i+4*j);
			}
		}
	}
}
