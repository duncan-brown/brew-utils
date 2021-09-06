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

void tlc5940_init(pcnt_isr_handle_t user_isr_handle, spi_device_handle_t spi);

#endif