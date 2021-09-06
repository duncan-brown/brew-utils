#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tlc5940.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <driver/pcnt.h>
#include <driver/timer.h>

// SPI, GSCLK and counter frequencies
#define GS_CLOCK_FREQ 5 * 1000 * 1000
#define SCLK_FREQ 5 * 1000 * 1000                           // SPI frequency
#define COUNTER_FREQ_MS 1000                                // Timer frequency in milliseconds
#define TIMER_DIVIDER 16                                    //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER / 1000) // convert counter value to milliseconds

// Ledc control unit
#define APP_PCNT_UNIT PCNT_UNIT_0

static void IRAM_ATTR gs_clk_handler(void *args);

void tlc5940_init(pcnt_isr_handle_t user_isr_handle, spi_device_handle_t spi)
{
     // Used to read the value of the pin (not from the device)
    gpio_set_direction(CONFIG_TLC_VPRG_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(CONFIG_TLC_XLAT_PIN, GPIO_MODE_DEF_INPUT);
    gpio_set_direction(CONFIG_TLC_BLANK_PIN, GPIO_MODE_DEF_OUTPUT);

    gpio_set_direction(CONFIG_TLC_LINE_1_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(CONFIG_TLC_LINE_2_PIN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(CONFIG_TLC_LINE_3_PIN, GPIO_MODE_DEF_OUTPUT);

    // Set initial output value
    gpio_set_level(CONFIG_TLC_SCLK_PIN, 0);
    gpio_set_level(CONFIG_TLC_DCPRG_PIN, 0);
    gpio_set_level(CONFIG_TLC_VPRG_PIN, 0);
    gpio_set_level(CONFIG_TLC_BLANK_PIN, 0);
    gpio_set_level(CONFIG_TLC_LINE_1_PIN, 0);
    gpio_set_level(CONFIG_TLC_LINE_2_PIN, 0);
    gpio_set_level(CONFIG_TLC_LINE_3_PIN, 0);

    // Set the gray scale clock
    ledc_timer_config_t ledc_timer = {
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = GS_CLOCK_FREQ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = CONFIG_TLC_GSCLK_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 1,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);

        // Setup the pulse counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = CONFIG_TLC_COUNTER_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 4095,
        .counter_l_lim = 0,
        .unit = APP_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0};
    pcnt_unit_config(&pcnt_config);
    pcnt_event_enable(APP_PCNT_UNIT, PCNT_EVT_H_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(APP_PCNT_UNIT);
    pcnt_counter_clear(APP_PCNT_UNIT);

    // Set interrupt
    pcnt_isr_register(gs_clk_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(APP_PCNT_UNIT);

    // Start the counter
    pcnt_counter_resume(APP_PCNT_UNIT);

    // Set spi master driver
    spi_bus_config_t bus_config = {
        .sclk_io_num = CONFIG_TLC_SCLK_PIN,
        .mosi_io_num = CONFIG_TLC_SIN_PIN,
        .miso_io_num = CONFIG_TLC_SOUT_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 24 * 8};

    //1: Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 0));

    // Spi defice configuration
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = SCLK_FREQ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .input_delay_ns = 10,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &device_config, &spi));
}

// Handler for counter overflow at 4096 pulses
static void IRAM_ATTR gs_clk_handler(void *args)
{
    // Clear the interrupt, this is need to be done immediately otherwise the counter will be increased again to MAX+1 an the interrupt will be triggered another time
    PCNT.int_clr.val = BIT(APP_PCNT_UNIT);

    // Start the task that will handle the gs data input cycle
    // We'll work on the APP_CPU while the main default loop will work on the PRO_CPU
    //xTaskCreatePinnedToCore(gs_input_data_cycle_task, GS_CYCLE_TASK_NAME, 4096, NULL, GS_CYCLE_TASK_PRIORITY, NULL, APP_CPU_NUM); 
}
