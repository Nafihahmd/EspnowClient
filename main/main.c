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

static const char *TAG = "espnow_client";

/* Configs */
#define NVS_NAMESPACE      "espnow_ns"
#define NVS_KEY_GATEWAY    "gateway_mac"     // stored first (string "AA:BB:...")
#define NVS_KEY_CFG_PREFIX "cfg"             // cfg0..cfg4 (five random variables)
#define CFG_COUNT          5

#define HEARTBEAT_INTERVAL_MS (5 * 60 * 1000) // unused as sensor-event based; kept if needed
#define REGISTER_INTERVAL_MS   (30 * 1000)    // when gateway unknown, send register every 30s
#define GPIO_DOUBLE_PRESS      GPIO_NUM_9
#define DOUBLE_PRESS_MS        500

#define LED_GPIO               GPIO_NUM_15
#define CONNECTED_TIMEOUT_MS   (2 * 60 * 1000) // consider connected if we saw gateway in last 2 mins

static uint8_t s_my_mac[6];
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t s_gateway_mac[6];
static bool gateway_known = false;
static int64_t last_seen_from_gateway = 0;

static TimerHandle_t register_timer;
static TaskHandle_t led_task_handle = NULL;
static TaskHandle_t hb_task_handle = NULL;
static volatile int64_t last_press_ts = 0;

/* ------------ helpers ------------- */
static void mac_to_str(const uint8_t *mac, char *str, size_t len) {
    snprintf(str, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
static bool is_broadcast_mac(const uint8_t *mac) {
    return memcmp(mac, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0;
}

/* NVS: store gateway MAC (string) and cfg0..cfg4 ints.
   We ensure gateway_mac key is created first during init. */
static esp_err_t nvs_store_gateway_mac(const uint8_t *mac) {
    char macs[18];
    mac_to_str(mac, macs, sizeof(macs));
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_str(h, NVS_KEY_GATEWAY, macs);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    if (r == ESP_OK) {
        memcpy(s_gateway_mac, mac, 6);
        gateway_known = true;
        last_seen_from_gateway = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG, "Saved gateway MAC %s to NVS", macs);
    }
    return r;
}
static esp_err_t nvs_load_gateway_mac(uint8_t *mac_out) {
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    size_t required = 0;
    r = nvs_get_str(h, NVS_KEY_GATEWAY, NULL, &required);
    if (r == ESP_OK && required > 0) {
        char *tmp = malloc(required);
        if (!tmp) { nvs_close(h); return ESP_ERR_NO_MEM; }
        r = nvs_get_str(h, NVS_KEY_GATEWAY, tmp, &required);
        if (r == ESP_OK) {
            unsigned int b[6];
            if (sscanf(tmp, "%02x:%02x:%02x:%02x:%02x:%02x",
                       &b[0],&b[1],&b[2],&b[3],&b[4],&b[5])==6) {
                for (int i=0;i<6;i++) mac_out[i]=(uint8_t)b[i];
                gateway_known = true;
                last_seen_from_gateway = esp_timer_get_time() / 1000;
            } else {
                r = ESP_ERR_INVALID_ARG;
            }
        }
        free(tmp);
    }
    nvs_close(h);
    return r;
}

static esp_err_t nvs_set_cfg(int idx, int32_t val) {
    if (idx < 0 || idx >= CFG_COUNT) return ESP_ERR_INVALID_ARG;
    char key[16]; snprintf(key, sizeof(key), "%s%d", NVS_KEY_CFG_PREFIX, idx);
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_i32(h, key, val);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t nvs_get_cfg(int idx, int32_t *val) {
    if (idx < 0 || idx >= CFG_COUNT) return ESP_ERR_INVALID_ARG;
    char key[16]; snprintf(key, sizeof(key), "%s%d", NVS_KEY_CFG_PREFIX, idx);
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
    int32_t tmp; r = nvs_get_i32(h, key, &tmp);
    if (r == ESP_ERR_NVS_NOT_FOUND) { *val = 0; r = ESP_OK; }
    else if (r == ESP_OK) *val = tmp;
    nvs_close(h);
    return r;
}

/* ------------- ESP-NOW JSON helpers ------------- */
static int prepare_espnow_json(uint8_t *buf, size_t buf_len, const char *json, const uint8_t *dest_mac) {
    size_t jlen = strlen(json);
    size_t total_len = sizeof(espnow_data_t) + jlen;
    if (total_len > buf_len) return -1;
    espnow_data_t *hdr = (espnow_data_t *)buf;
    hdr->type = is_broadcast_mac(dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    hdr->state = 0;
    hdr->seq_num = 0;
    hdr->crc = 0;
    hdr->magic = esp_random();
    memcpy(hdr->payload, json, jlen);
    hdr->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)hdr, total_len);
    return (int)total_len;
}

static esp_err_t ensure_peer_and_send(const uint8_t *dest_mac, const char *json) {
    // If dest mac is broadcast, just send (add broadcast peer was done in init).
    if (!is_broadcast_mac(dest_mac)) {
        if (!esp_now_is_peer_exist(dest_mac)) {
            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, dest_mac, ESP_NOW_ETH_ALEN);
            peer.channel = CONFIG_ESPNOW_CHANNEL;
            peer.ifidx = ESPNOW_WIFI_IF;
            peer.encrypt = false;
            esp_err_t r = esp_now_add_peer(&peer);
            if (r != ESP_OK /*(&& r != ESP_ERR_ESPNOW_ID */) {
                ESP_LOGW(TAG, "esp_now_add_peer failed %d", r);
            }
        }
    }
    size_t buf_len = 512;
    uint8_t *buf = malloc(buf_len);
    if (!buf) return ESP_ERR_NO_MEM;
    int total = prepare_espnow_json(buf, buf_len, json, dest_mac);
    if (total < 0) { free(buf); return ESP_ERR_INVALID_ARG; }
    esp_err_t r = esp_now_send(dest_mac, buf, total);
    free(buf);
    return r;
}

/* ------------- Application API hook ------------- */
/* Call this function to send an event-based sensor reading to gateway.
   Example payload: payload JSON object should contain "mac" field automatically filled in.
   Returns ESP_OK on send attempt. */
esp_err_t espnow_client_send_sensor_event(cJSON *payload_obj) {
    if (!gateway_known) {
        ESP_LOGW(TAG, "Gateway unknown - cannot send sensor event");
        return ESP_ERR_INVALID_STATE;
    }
    // attach mac and wrap
    char macstr[18]; mac_to_str(s_my_mac, macstr, sizeof(macstr));
    cJSON_AddStringToObject(payload_obj, "mac", macstr);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "sensor");
    cJSON_AddItemToObject(o, "payload", cJSON_Duplicate(payload_obj, 1));
    cJSON_AddTrueToObject(o, "status");
    char *s = cJSON_PrintUnformatted(o);
    esp_err_t r = ensure_peer_and_send(s_gateway_mac, s);
    cJSON_free(s);
    cJSON_Delete(o);
    return r;
}

/* ------------- ESPNOW receive callback (from gateway or other) ------------- */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (!data || len <= (int)sizeof(espnow_data_t)) return;
    espnow_data_t *hdr = (espnow_data_t *)data;
    int json_len = len - sizeof(espnow_data_t);
    char *json = malloc(json_len + 1);
    if (!json) return;
    memcpy(json, hdr->payload, json_len);
    json[json_len] = 0;

    ESP_LOGI(TAG, "ESPNOW recv from "MACSTR": %s", MAC2STR(recv_info->src_addr), json);

    // If received from gateway, update last_seen and save gateway mac if unknown
    if (gateway_known == false) {
        // If we get a unicast reply from a device and we don't have a gateway, treat it as gateway reply
        // but we check: if the message contains "type":"gateway_info" or any JSON, accept it.
        // Save sender as gateway.
        ESP_LOGI(TAG, "Treating %02x:%02x:%02x:%02x:%02x:%02x as gateway",
                 recv_info->src_addr[0],recv_info->src_addr[1],recv_info->src_addr[2],
                 recv_info->src_addr[3],recv_info->src_addr[4],recv_info->src_addr[5]);
        nvs_store_gateway_mac(recv_info->src_addr);
    }
    // Update last_seen only if sender matches saved gateway
    if (gateway_known && memcmp(recv_info->src_addr, s_gateway_mac, 6) == 0) {
        last_seen_from_gateway = esp_timer_get_time() / 1000;
    }

    // Parse JSON for config_request replies or set_config messages
    cJSON *root = cJSON_Parse(json);
    if (root) {
        cJSON *type = cJSON_GetObjectItem(root, "type");
        if (cJSON_IsString(type)) {
            if (strcmp(type->valuestring, "config_response") == 0) {
                cJSON *pl = cJSON_GetObjectItem(root, "payload");
                if (pl) {
                    for (int i=0;i<CFG_COUNT;i++) {
                        char k[8]; snprintf(k, sizeof(k), "cfg%d", i);
                        cJSON *it = cJSON_GetObjectItem(pl, k);
                        if (cJSON_IsNumber(it)) {
                            nvs_set_cfg(i, it->valueint);
                            ESP_LOGI(TAG, "Saved %s=%d", k, it->valueint);
                        }
                    }
                }
            } else if (strcmp(type->valuestring, "set_config")==0) {
                cJSON *pl = cJSON_GetObjectItem(root, "payload");
                if (pl) {
                    cJSON *it = pl->child;
                    while (it) {
                        if (cJSON_IsNumber(it)) {
                            // accept cfg0..cfg4
                            if (strncmp(it->string, "cfg", 3) == 0) {
                                // parse index
                                int idx = atoi(it->string + 3);
                                if (idx >= 0 && idx < CFG_COUNT) {
                                    nvs_set_cfg(idx, it->valueint);
                                    ESP_LOGI(TAG, "Updated %s = %d", it->string, it->valueint);
                                }
                            }
                        }
                        it = it->next;
                    }
                }
            }
        }
        cJSON_Delete(root);
    }

    free(json);
}

/* ------------- GPIO double-press handler ------------- */
/* On GPIO9 double press: send register payload. If gateway known, send unicast register to gateway (so gateway can reply again).
   If gateway unknown, broadcast register (so gateway can be discovered). */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int level = gpio_get_level(GPIO_DOUBLE_PRESS);
    if (level == 0) {
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_press_ts <= DOUBLE_PRESS_MS) {
            // double press detected -> send register
            char macstr[18]; mac_to_str(s_my_mac, macstr, sizeof(macstr));
            cJSON *o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "type", "register");
            cJSON *pl = cJSON_CreateObject();
            cJSON_AddStringToObject(pl, "mac", macstr);
            cJSON_AddItemToObject(o, "payload", pl);
            char *s = cJSON_PrintUnformatted(o);
            if (gateway_known) {
                ESP_LOGI(TAG, "Double press -> sending register unicast to gateway");
                ensure_peer_and_send(s_gateway_mac, s);
            } else {
                ESP_LOGI(TAG, "Double press -> sending register broadcast (discover gateway)");
                ensure_peer_and_send(s_broadcast_mac, s);
            }
            cJSON_free(s);
            cJSON_Delete(o);
        }
        last_press_ts = now;
    }
}

/* ------------- Register timer (when gateway unknown) ------------- */
static void register_timer_cb(TimerHandle_t t) {
    if (gateway_known) return; // stop sending
    // send register as broadcast
    char macstr[18]; mac_to_str(s_my_mac, macstr, sizeof(macstr));
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "register");
    cJSON *pl = cJSON_CreateObject();
    cJSON_AddStringToObject(pl, "mac", macstr);
    cJSON_AddItemToObject(o, "payload", pl);
    char *s = cJSON_PrintUnformatted(o);
    ESP_LOGI(TAG, "Periodic register broadcast (gateway unknown)");
    ensure_peer_and_send(s_broadcast_mac, s);
    cJSON_free(s);
    cJSON_Delete(o);
}

/* ----------- Heartbeat task ----------*/
static void heartbeat_task(void *arg) {
    const TickType_t interval = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
    while (1) {
        if (gateway_known) {
            // send heartbeat to gateway
            char macstr[18]; mac_to_str(s_my_mac, macstr, sizeof(macstr));
            cJSON *o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "type", "heartbeat");
            cJSON_AddStringToObject(o, "mac", macstr);
            char *s = cJSON_PrintUnformatted(o);
            ensure_peer_and_send(s_gateway_mac, s);
            cJSON_free(s);
            cJSON_Delete(o);
        }
        vTaskDelay(interval);
    }
}
/* ------------- LED task ------------- */
static void led_task(void *arg) {
    // gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    const TickType_t blink_delay = pdMS_TO_TICKS(500);
    while (1) {
        int64_t now = esp_timer_get_time() / 1000;
        bool connected = gateway_known && (now - last_seen_from_gateway <= CONNECTED_TIMEOUT_MS);
        if (connected) {
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

/* ------------- WiFi & ESPNOW init ------------- */
static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
#if CONFIG_ESPNOW_WIFI_MODE_STATION
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
#else
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
#endif
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

void app_main(void) {
    ESP_LOGI(TAG, "Client booting...");

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(r);

    // read or create gateway_mac first (so it is "first object" in NVS)
    uint8_t gw[6];
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

    // get my mac
    esp_efuse_mac_get_default(s_my_mac);
    char macs[18]; mac_to_str(s_my_mac, macs, sizeof(macs));
    ESP_LOGI(TAG, "Client MAC: %s", macs);

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

    // led task
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle);

    wifi_init();
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    // add broadcast peer so we can send discovery broadcasts
    esp_now_peer_info_t peer = {0};
    peer.channel = CONFIG_ESPNOW_CHANNEL;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    esp_err_t e = esp_now_add_peer(&peer);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "add broadcast peer returned %d", e);
    }

    // If gateway known, add peer for gateway
    if (gateway_known) {
        esp_now_peer_info_t p2 = {0};
        p2.channel = CONFIG_ESPNOW_CHANNEL;
        p2.ifidx = ESPNOW_WIFI_IF;
        p2.encrypt = false;
        memcpy(p2.peer_addr, s_gateway_mac, ESP_NOW_ETH_ALEN);
        esp_now_add_peer(&p2);
    }

    // periodic register timer (only active if gateway unknown)
    register_timer = xTimerCreate("reg_t", pdMS_TO_TICKS(REGISTER_INTERVAL_MS), pdTRUE, NULL, register_timer_cb);
    if (register_timer) xTimerStart(register_timer, 0);
    
    // heartbeat task
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL,5, &hb_task_handle);
    ESP_LOGI(TAG, "Client ready. Gateway_known=%d", gateway_known);
}
