// espnow_client/main.c
// Updated client per user's modifications
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_random.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "espnow_data.h"
#include "nvs_helper.h"

static const char *TAG = "main";

// #define REGISTER_INTERVAL_MS   (30 * 1000)    // when gateway unknown, send register every 30s
#define GPIO_DOUBLE_PRESS      GPIO_NUM_9
#define DOUBLE_PRESS_MS        500

#define LED_GPIO               GPIO_NUM_15
// #define CONNECTED_TIMEOUT_MS   (2 * 60 * 1000) // consider connected if we saw gateway in last 2 mins

// static TimerHandle_t register_timer;
static TaskHandle_t led_task_handle = NULL;
// static TaskHandle_t hb_task_handle = NULL;
static volatile int64_t last_press_ts = 0;

/* ------------- GPIO double-press handler ------------- */
/* On GPIO9 double press: send register payload. If gateway known, erase.
   If gateway unknown, broadcast register (so gateway can be discovered). */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int level = gpio_get_level(GPIO_DOUBLE_PRESS);
    if (level == 0) {
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_press_ts <= DOUBLE_PRESS_MS) {
            if (gateway_known) {
                // Erase gateway to rediscover
                nvs_erase_gateway_mac();
            }
        }
        last_press_ts = now;
    }
}

/* ------------- LED task ------------- */
static void led_task(void *arg) {
    // gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    const TickType_t blink_delay = pdMS_TO_TICKS(500);
    while (1) {
        if (gateway_known) {
            // solid
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            // blink
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(blink_delay);
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(blink_delay);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Client booting...");    
    uint8_t gw[6];
    char macs[18]; 

    esp_err_t r = nvs_init();
    ESP_ERROR_CHECK(r);

    // read or create gateway_mac first (so it is "first object" in NVS)
    esp_err_t rr = nvs_load_gateway_mac(gw); // sets gateway_known if found
    if (rr == ESP_OK && gateway_known) {
        memcpy(s_gateway_mac, gw, 6);
        ESP_LOGI(TAG, "Loaded gateway MAC from NVS");
    } else {
        ESP_LOGI(TAG, "No gateway MAC in NVS (will use discovery)");
        gateway_known = false;
    }

    // ensure config keys exist (create after gateway_mac, so gateway_mac is first)
    for (int i=0;i<CFG_COUNT;i++) {
        int32_t dummy;
        nvs_get_cfg(i, &dummy);
    }

    // print device mac
    // esp_efuse_mac_get_default(s_my_mac);
    esp_read_mac(s_my_mac, ESP_MAC_WIFI_STA);
    // mac_to_str(s_my_mac, macs, sizeof(macs));

    // setup GPIO ISR for double press
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1ULL<<GPIO_DOUBLE_PRESS;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_DOUBLE_PRESS, gpio_isr_handler, NULL);

    wifi_init();
    espnow_init();
    // led task
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);
    ESP_LOGI(TAG, "Client ready. Gateway_known=%s", gateway_known?"yes":"no");
}
