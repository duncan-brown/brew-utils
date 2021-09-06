#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arrow_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "st7789.h"
#include "fontx.h"

extern uint16_t segment_0;

TickType_t DisplaySegment0(TFT_t * dev, FontxFile *fx, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	//ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	const uint8_t ascii_len = 28;
	uint8_t ascii[ascii_len];
	uint16_t color;

	lcdFillScreen(dev, BLACK);
	lcdSetFontDirection(dev, DIRECTION90);
	color = WHITE;
	ypos = 0;
    xpos = width - 1;

    snprintf((char *)ascii, ascii_len, "Display updated: %d", segment_0);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

TickType_t DisplayTLCPins(TFT_t * dev, FontxFile *fx, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	//ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	const uint8_t ascii_len = 28;
	uint8_t ascii[ascii_len];
	uint16_t color;

	lcdFillScreen(dev, BLACK);
	lcdSetFontDirection(dev, DIRECTION90);
	color = WHITE;
	ypos = 0;
    xpos = width - 1;

    snprintf((char *)ascii, ascii_len, "    SIN: %d   VPRG: %d", CONFIG_TLC_SIN_PIN, CONFIG_TLC_VPRG_PIN);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, ascii_len, "   SOUT: %d   XLAT: %d", CONFIG_TLC_SOUT_PIN, CONFIG_TLC_XLAT_PIN);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, ascii_len, "   SCLK: %d  BLANK: %d", CONFIG_TLC_SCLK_PIN, CONFIG_TLC_BLANK_PIN);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, ascii_len, "  DCPRG: %d  GSCLK: %d", CONFIG_TLC_DCPRG_PIN, CONFIG_TLC_GSCLK_PIN);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, ascii_len, "COUNTER: %d  LINE 1: %d", CONFIG_TLC_COUNTER_PIN, CONFIG_TLC_LINE_1_PIN);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, ascii_len, "  LINE2: %d  LINE 3: %d", CONFIG_TLC_LINE_2_PIN, CONFIG_TLC_LINE_3_PIN);
	xpos -= fontHeight + 1;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

TickType_t ArrowTest(TFT_t * dev, FontxFile *fx, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	//ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	int	stlen;
	uint8_t ascii[24];
	uint16_t color;

	lcdFillScreen(dev, BLACK);

	strcpy((char *)ascii, "KARR");
	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 1;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 1;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	lcdSetFontDirection(dev, 0);
	color = RED;
	lcdDrawFillArrow(dev, 10, 10, 0, 0, 5, color);
	strcpy((char *)ascii, "0,0");
	lcdDrawString(dev, fx, 0, 30, ascii, color);

	color = GREEN;
	lcdDrawFillArrow(dev, width-11, 10, width-1, 0, 5, color);
	//strcpy((char *)ascii, "79,0");
	sprintf((char *)ascii, "%d,0",width-1);
	stlen = strlen((char *)ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(dev, fx, xpos, 30, ascii, color);

	color = GRAY;
	lcdDrawFillArrow(dev, 10, height-11, 0, height-1, 5, color);
	//strcpy((char *)ascii, "0,159");
	sprintf((char *)ascii, "0,%d",height-1);
	ypos = (height-11) - (fontHeight) + 5;
	lcdDrawString(dev, fx, 0, ypos, ascii, color);

	color = CYAN;
	lcdDrawFillArrow(dev, width-11, height-11, width-1, height-1, 5, color);
	//strcpy((char *)ascii, "79,159");
	sprintf((char *)ascii, "%d,%d",width-1, height-1);
	stlen = strlen((char *)ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}