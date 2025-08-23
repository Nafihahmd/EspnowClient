/* NVS_HELPER.C
   NVS Helper Implementation File

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "nvs_helper.h"
// #include "cJSON.h"
// #include "espnow_data.h"

static const char *TAG = "nvs_helper";

/* Configs */
#define NVS_NAMESPACE      "espnow_ns"
#define NVS_KEY_GATEWAY    "gateway_mac"     // stored first (string "AA:BB:...")
#define NVS_KEY_CFG_PREFIX "cfg"             // cfg0..cfg4 (five random variables)
extern bool gateway_known;
extern uint8_t s_gateway_mac[6];
/* ------------- Load gateway MAC from NVS ------------- */

/* NVS: store gateway MAC (string) and cfg0..cfg4 ints.
   We ensure gateway_mac key is created first during init. */
esp_err_t nvs_store_gateway_mac(const uint8_t *mac) {
    char macs[18];
    nvs_handle_t h;
    snprintf(macs, sizeof(macs), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_str(h, NVS_KEY_GATEWAY, macs);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    if (r == ESP_OK) {
        memcpy(s_gateway_mac, mac, 6);
        gateway_known = true;
        // last_seen_from_gateway = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG, "Saved gateway MAC %s to NVS", macs);
    }
    return r;
}

esp_err_t nvs_load_gateway_mac(uint8_t *mac_out) {
    nvs_handle_t h;
    size_t required = 0;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (r != ESP_OK) return r;
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
                // last_seen_from_gateway = esp_timer_get_time() / 1000;
            } else {
                r = ESP_ERR_INVALID_ARG;
            }
        }
        free(tmp);
    }
    nvs_close(h);
    return r;
}

esp_err_t nvs_erase_gateway_mac(void) {
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_erase_key(h, NVS_KEY_GATEWAY);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    if (r == ESP_OK) {
        gateway_known = false;
        memset(s_gateway_mac, 0, 6);
        ESP_LOGI(TAG, "Erased gateway MAC from NVS");
    }
    return r;
}

esp_err_t nvs_set_cfg(int idx, int32_t val) {
    char key[16];
    if (idx < 0 || idx >= CFG_COUNT) 
        return ESP_ERR_INVALID_ARG;
    snprintf(key, sizeof(key), "%s%d", NVS_KEY_CFG_PREFIX, idx);
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) 
        return r;
    r = nvs_set_i32(h, key, val);
    if (r == ESP_OK) 
        r = nvs_commit(h);
    nvs_close(h);
    return r;
}
esp_err_t nvs_get_cfg(int idx, int32_t *val) {
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

esp_err_t nvs_init(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    return r;
}
