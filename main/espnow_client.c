/* ESPNOW Client

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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

static const char *TAG = "espnow_client";

/* Global Variables */
#define HEARTBEAT_INTERVAL_MS (20 * 1000) // unused as sensor-event based; kept if needed

uint8_t s_my_mac[6];
uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t s_gateway_mac[6];
bool gateway_known = false;
static QueueHandle_t s_espnow_queue = NULL;

/* ------------ helpers ------------- */
void mac_to_str(const uint8_t *mac, char *str, size_t len) {
    snprintf(str, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
// bool is_broadcast_mac(const uint8_t *mac) {
//     return memcmp(mac, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0;
// }


/* WiFi should start before using ESPNOW */
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR));
#endif
}

/* ESPNOW sending callback function */
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

/* ESPNOW receiving callback function */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (recv_info->src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *type)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *type = buf->type;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        buf->crc = crc;
        return 0; // Success
    }

    return -1; // CRC error
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param, uint8_t *payload, uint16_t payload_len)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->crc = 0;
    
    // Copy payload if provided
    if (payload != NULL && payload_len > 0) {
        memcpy(buf->payload, payload, payload_len);
    }
    
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

/* Deinitialize ESPNOW */
static void espnow_deinit(espnow_send_param_t *send_param)
{
    if (send_param) {
        free(send_param->buffer);
        free(send_param);
    }
    
    if (s_espnow_queue) {
        vQueueDelete(s_espnow_queue);
        s_espnow_queue = NULL;
    }
    
    esp_now_deinit();
}


/* ----------- Heartbeat task ----------*/
static void heartbeat_task(void *arg) {
    const TickType_t interval = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
    while (1) {
        if (gateway_known) {
            // send heartbeat to gateway
            char macstr[18];
            mac_to_str(s_my_mac, macstr, sizeof(macstr));
            cJSON *o = cJSON_CreateObject();
            cJSON_AddStringToObject(o, "mac", macstr);
            cJSON_AddStringToObject(o, "type", "heartbeat");
            cJSON_AddNumberToObject(o, "sensorValue", 2.2); // dummy value
            // char *s = cJSON_PrintUnformatted(o);
            ESP_LOGI(TAG, "Sending heartbeat to gateway");
            // ensure_peer_and_send(s_gateway_mac, s);
            espnow_send_json(s_gateway_mac, o);
            // cJSON_free(s);
            cJSON_Delete(o);
        }
        vTaskDelay(interval);
    }
}

/* ESPNOW task to handle events */
static void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t data_type;
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;

    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                ESP_LOGD(TAG, "Send data to "MACSTR", status: %d", 
                         MAC2STR(send_cb->mac_addr), send_cb->status);
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                
                if (espnow_data_parse(recv_cb->data, recv_cb->data_len, &data_type) == 0) {
                    if (data_type == ESPNOW_DATA_BROADCAST && !gateway_known) {
                        ESP_LOGI(TAG, "Receive broadcast data from: "MACSTR", len: %d", 
                                 MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                        // Add peer if not exists
                        if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                            esp_now_peer_info_t peer;
                            memset(&peer, 0, sizeof(esp_now_peer_info_t));
                            peer.channel = CONFIG_ESPNOW_CHANNEL;
                            peer.ifidx = ESPNOW_WIFI_IF;
                            peer.encrypt = true;
                            memcpy(peer.lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                            memcpy(peer.peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            ESP_ERROR_CHECK(esp_now_add_peer(&peer));
                        }
                        nvs_store_gateway_mac(recv_cb->mac_addr);
                    } else if (data_type == ESPNOW_DATA_UNICAST) {                                            
                        espnow_data_t *buf = (espnow_data_t *)recv_cb->data;
                        int payload_len = recv_cb->data_len - sizeof(espnow_data_t);
                        ESP_LOGI(TAG, "Receive unicast data from: "MACSTR", len: %d", 
                                 MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                        
                        if (payload_len > 0) {
                            // Add null terminator to make it a valid C string
                            char *json_str = malloc(payload_len + 1);
                            if (json_str) {
                                memcpy(json_str, buf->payload, payload_len);
                                json_str[payload_len] = '\0';
                                
                                // Parse and print the JSON
                                cJSON *root = cJSON_Parse(json_str);
                                if (root) {
                                    char *printed = cJSON_Print(root);
                                    ESP_LOGI(TAG, "Received JSON: %s", printed);
                                    free(printed);
                                    cJSON_Delete(root);
                                } else {
                                    ESP_LOGI(TAG, "Received data (not JSON): %s", json_str);
                                }
                            }
                            free(json_str);
                                
                        }
                    }
                } else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                
                free(recv_cb->data);
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

/* API to send JSON data */
esp_err_t espnow_send_json(const uint8_t *mac_addr, cJSON *json)
{
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON object");
        return ESP_FAIL;
    }
    
    // Convert JSON to string
    char *json_str = cJSON_PrintUnformatted(json);
    if (!json_str) {
        ESP_LOGE(TAG, "Failed to print JSON");
        return ESP_FAIL;
    }
    
    size_t json_len = strlen(json_str);
    size_t total_len = sizeof(espnow_data_t) + json_len;
    
    // Allocate buffer
    uint8_t *buffer = malloc(total_len);
    if (!buffer) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(json_str);
        return ESP_FAIL;
    }
    
    // Prepare send parameters
    espnow_send_param_t send_param;
    send_param.unicast = !IS_BROADCAST_ADDR(mac_addr);
    send_param.broadcast = IS_BROADCAST_ADDR(mac_addr);
    send_param.delay = 0;
    send_param.len = total_len;
    send_param.buffer = buffer;
    memcpy(send_param.dest_mac, mac_addr, ESP_NOW_ETH_ALEN);
    
    // Prepare the data
    espnow_data_prepare(&send_param, (uint8_t *)json_str, json_len);
    
    // Send the data
    esp_err_t err = esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);
    
    // Clean up
    free(buffer);
    free(json_str);
    
    return err;
}

/* Initialize ESPNOW */
esp_err_t espnow_init(void)
{
    espnow_send_param_t *send_param;

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
#endif
    
    /* Set primary master key. */
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = CONFIG_ESPNOW_CHANNEL;
    peer.ifidx = ESPNOW_WIFI_IF;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        espnow_deinit(send_param);
        return ESP_FAIL;
    }
    
    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->broadcast = true;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        espnow_deinit(send_param);
        return ESP_FAIL;
    }
    
    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);

    xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 5, NULL);

    return ESP_OK;
}

/* API to send data */
esp_err_t espnow_send_data(const uint8_t *mac_addr, const uint8_t *data, uint16_t len)
{
    espnow_send_param_t send_param;
    
    send_param.unicast = !IS_BROADCAST_ADDR(mac_addr);
    send_param.broadcast = IS_BROADCAST_ADDR(mac_addr);
    send_param.delay = 0; // No delay for event-based sending
    send_param.len = sizeof(espnow_data_t) + len;
    send_param.buffer = malloc(send_param.len);
    
    if (send_param.buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        return ESP_FAIL;
    }
    
    memcpy(send_param.dest_mac, mac_addr, ESP_NOW_ETH_ALEN);
    
    // Prepare the data
    espnow_data_prepare(&send_param, (uint8_t *)data, len);
    
    // Send the data
    esp_err_t err = esp_now_send(send_param.dest_mac, send_param.buffer, send_param.len);
    
    free(send_param.buffer);
    return err;
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
    char macstr[18];
    mac_to_str(s_my_mac, macstr, sizeof(macstr));
    cJSON_AddStringToObject(payload_obj, "mac", macstr);

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "sensor");
    cJSON_AddItemToObject(o, "payload", cJSON_Duplicate(payload_obj, 1));
    cJSON_AddTrueToObject(o, "status");
    // char *s = cJSON_PrintUnformatted(o);
    esp_err_t r = espnow_send_json(s_gateway_mac, o);
    // cJSON_free(s);
    cJSON_Delete(o);
    return r;
}

/* ------------- ESPNOW receive callback (from gateway or other) ------------- */
void espnow_json_cmd_handler(const char *json) {
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


/* ------------- WiFi & ESPNOW init ------------- */
// void wifi_espnow_init(void) {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
// #if CONFIG_ESPNOW_WIFI_MODE_STATION
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
// #else
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
// #endif
//     ESP_ERROR_CHECK(esp_wifi_start());
//     ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

//     /* Init ESPNOW and register recv cb */
//     ESP_ERROR_CHECK( esp_now_init() );
//     ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
//     ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

//     // add broadcast peer so we can send discovery broadcasts
//     esp_now_peer_info_t peer = {0};
//     peer.channel = CONFIG_ESPNOW_CHANNEL;
//     peer.ifidx = ESPNOW_WIFI_IF;
//     peer.encrypt = false;
//     memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
//     esp_err_t e = esp_now_add_peer(&peer);
//     if (e != ESP_OK) {
//         ESP_LOGW(TAG, "add broadcast peer returned %d", e);
//     }

//     // If gateway known, add peer for gateway
//     if (gateway_known) {
//         esp_now_peer_info_t p2 = {0};
//         p2.channel = CONFIG_ESPNOW_CHANNEL;
//         p2.ifidx = ESPNOW_WIFI_IF;
//         p2.encrypt = false;
//         memcpy(p2.peer_addr, s_gateway_mac, ESP_NOW_ETH_ALEN);
//         esp_now_add_peer(&p2);
//     }
// }


/* ------------- Register timer (when gateway unknown) ------------- */
// void register_timer_cb(TimerHandle_t t) {
//     ESP_LOGI(TAG, "Register timer callback");
//     if (gateway_known) {
//         xTimerDelete(t, 0);
//         return; // stop sending
//     }
//     // send register as broadcast
//     char macstr[18]; mac_to_str(s_my_mac, macstr, sizeof(macstr));
//     cJSON *o = cJSON_CreateObject();
//     cJSON_AddStringToObject(o, "type", "register");
//     cJSON *pl = cJSON_CreateObject();
//     cJSON_AddStringToObject(pl, "mac", macstr);
//     cJSON_AddItemToObject(o, "payload", pl);
//     char *s = cJSON_PrintUnformatted(o);
//     ESP_LOGI(TAG, "Periodic register broadcast (gateway unknown)");
//     ensure_peer_and_send(s_broadcast_mac, s);
//     cJSON_free(s);
//     cJSON_Delete(o);
// }
