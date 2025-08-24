/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_DATA_H
#define ESPNOW_DATA_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

//#MAC_SELF e4:b3:23:b6:a8:08
#define ESPNOW_QUEUE_SIZE           6
#define ESPNOW_MAXDELAY             512

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data. */
typedef struct {
    uint8_t type;                         // Broadcast or unicast ESPNOW data.
    uint16_t crc;                         // CRC16 value of ESPNOW data.
    uint8_t payload[0];                   // Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         // Send unicast ESPNOW data.
    bool broadcast;                       // Send broadcast ESPNOW data.
    uint16_t delay;                       // Delay between sending two ESPNOW data, unit: ms.
    int len;                              // Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device.
} espnow_send_param_t;

extern uint8_t s_gateway_mac[6];
extern bool gateway_known;
extern uint8_t s_my_mac[6];
extern uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN];

/* Function Declarations */
// void mac_to_str(const uint8_t *mac, char *str, size_t len);
// bool is_broadcast_mac(const uint8_t *mac);
// int prepare_espnow_json(uint8_t *buf, size_t buf_len, const char *json, const uint8_t *dest_mac);
// esp_err_t ensure_peer_and_send(const uint8_t *dest_mac, const char *json);
// esp_err_t espnow_send_sensor_event(cJSON *payload_obj);
// void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
// void wifi_espnow_init(void);
// void register_timer_cb(TimerHandle_t t);
void wifi_init(void);
esp_err_t espnow_init(void);
esp_err_t espnow_send_json(const uint8_t *mac_addr, cJSON *json);
void mac_to_str(const uint8_t *mac, char *str, size_t len);
#endif /*ESPNOW_EXAMPLE_H*/
