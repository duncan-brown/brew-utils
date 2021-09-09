#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/pcnt.h"
#include "driver/timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "st7789.h"
#include "fontx.h"
#include "arrow_test.h"
#include "tlc5940.h"

// Timer task
#define TIMER_TASK_NAME "TIMER"
#define TIMER_TASK_PRIORITY 2
#define COUNTER_FREQ_MS 500                                 // Timer frequency in milliseconds
#define TIMER_DIVIDER 16                                    //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER / 1000) // convert counter value to milliseconds

static const char *TAG = "KARR";

TFT_t dev;
FontxFile fx16G[2];
FontxFile fx24G[2];

static void SPIFFS_Directory(char * path);
void st7789_init(void);
void display_timer_init();
void timer_task(void *parameters);
static void IRAM_ATTR timer_handler(void *args);

// Display values
uint16_t segment_0 = 0;
uint16_t bar = 0;

void app_main(void)
{
	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 10,
		.format_if_mount_failed =true
	};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total,&used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	SPIFFS_Directory("/spiffs/");

	ESP_LOGI(TAG, "Initializing ST7789");
	st7789_init();

	DisplayTLCPins(&dev, fx16G, CONFIG_WIDTH, CONFIG_HEIGHT);

	// Initialize the TLC5940
	tlc5940_init();

	// Initialize the timer
	display_timer_init();
	
}


static void SPIFFS_Directory(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}


void st7789_init(void)
{
	// load fonts
	InitFontx(fx16G,"/spiffs/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic

	// initialize display
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
	return;
}


// Setup the timer to update the value on the display
void display_timer_init()
{
    // Init the counter timer
    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);

    // Set to 0 the counter start value
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x0LLU);

    // Configure the alarm and the interrupt
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, COUNTER_FREQ_MS * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // Start the timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}


static void IRAM_ATTR timer_handler(void *args)
{
    // Clear the interrupt
    TIMERG0.int_clr_timers.t0 = 1;

    // Re-enable the timer for the next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    xTaskCreatePinnedToCore(timer_task, TIMER_TASK_NAME, 4096, NULL, TIMER_TASK_PRIORITY, NULL, APP_CPU_NUM);
}

void timer_task(void *parameters)
{
	segment_0 += 1;
	if (segment_0 > 91) segment_0 = 0;

    bar = segment_0 % 13;
	DisplaySegment0(&dev, fx16G, CONFIG_WIDTH, CONFIG_HEIGHT);

	ESP_LOGI(TAG, "%d, %d", segment_0, bar);

    // Finally delete the task
    vTaskDelete(NULL);
}
