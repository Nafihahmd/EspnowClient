/* NVS Helper Header File

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef NVS_HELPER_H
#define NVS_HELPER_H

/* Global Variables */
#define CFG_COUNT          5

/* Global Functions */
esp_err_t nvs_init(void);
esp_err_t nvs_store_gateway_mac(const uint8_t *mac);
esp_err_t nvs_load_gateway_mac(uint8_t *mac_out);
esp_err_t nvs_erase_gateway_mac(void);
esp_err_t nvs_get_cfg(int idx, int32_t *value);
esp_err_t nvs_set_cfg(int idx, int32_t value);
#endif // NVS_HELPER_H