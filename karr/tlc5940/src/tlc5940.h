#ifndef MAIN_TLC5940_H_
#define MAIN_TLC5940_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <driver/pcnt.h>
#include <driver/timer.h>

void tlc5940_init(void);

void gs_input_data_cycle_task(void *parameters);

void set_pin_value(uint8_t pin, uint16_t value);

void compute_leds_values();

#endif
