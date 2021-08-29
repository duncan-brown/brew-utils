#ifndef MAIN_ARROW_TEST_H_
#define MAIN_ARROW_TEST_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "st7789.h"
#include "fontx.h"

TickType_t ArrowTest(TFT_t * dev, FontxFile *fx, int width, int height);

#endif