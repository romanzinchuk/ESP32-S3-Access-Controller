#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "gatt_svr.h"
#include "password.h"

static const char *TAG = "PC_LOCK";

/* Target smartwatch MAC Address (Little Endian format) */
static const uint8_t TARGET_MAC[6] = {0x9F, 0x68, 0x07, 0xB7, 0x02, 0x78};

/* System state */
static bool pc_is_locked = false;
static float filtered_distance = 0.0;
static uint16_t pc_conn_handle = BLE_HS_CONN_HANDLE_NONE; 

/* RTOS queue */
static QueueHandle_t rssi_queue;

/* Distance and filter constants */
#define MEASURED_POWER -70    // Expected RSSI at 1 meter
#define N_COEFF 2.5           // Environmental signal loss
#define ALPHA_BASE 0.05       // Slow filter (static noise)
#define ALPHA_FAST 0.25       // Fast filter (real movement)
#define DIFF_THRESHOLD 1.5    // Delta to trigger fast filter
#define THRESHOLD_LOCK 2.0    // Lock distance (meters)
#define THRESHOLD_RESET 1.5   // Unlock distance (meters)
#define GRACE_PERIOD_MS 60000 // Timeout grace period

static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);

/* Configures and starts BLE scanner */
static void ble_app_scan(void) {
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Address inference failed (rc=%d)", rc);
        return;
    }

    // Skip if already scanning
    if (ble_gap_disc_active()) return;

    disc_params.filter_duplicates = 0; 
    disc_params.passive = 1; // Passive scanning to save radio time
    disc_params.itvl = 160;  // 100ms
    disc_params.window = 80; // 50ms

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Scan start failed (rc=%d)", rc);
    } else {
        ESP_LOGI(TAG, "Scanning for target MAC started");
    }
}

/* Security task: calculates distance and controls HID */
void security_task(void *pvParameters) {
    int8_t current_rssi;
    bool in_timeout = false;
    TickType_t last_seen_ticks = xTaskGetTickCount(); 

    ESP_LOGI(TAG, "Security task started. Waiting for RSSI...");

    while (1) {
        // Wait 5s for new RSSI packet
        if (xQueueReceive(rssi_queue, &current_rssi, pdMS_TO_TICKS(5000)) == pdPASS) {
            
            last_seen_ticks = xTaskGetTickCount(); 
            float raw_distance = pow(10, (float)(MEASURED_POWER - current_rssi) / (10 * N_COEFF));

            // Reset filter on startup or after timeout recovery
            if (filtered_distance == 0.0 || in_timeout) {
                filtered_distance = raw_distance;
                in_timeout = false; 
            } else {
                // Adaptive EMA filter
                float diff = fabs(raw_distance - filtered_distance);
                float current_alpha = (diff > DIFF_THRESHOLD) ? ALPHA_FAST : ALPHA_BASE;
                filtered_distance = (current_alpha * raw_distance) + ((1.0 - current_alpha) * filtered_distance);
            }

            ESP_LOGI(TAG, "RSSI: %d | Raw: %.2fm | Filtered: %.2fm", current_rssi, raw_distance, filtered_distance);

        } else {
            // Check if 60s grace period expired
            TickType_t elapsed_time = (xTaskGetTickCount() - last_seen_ticks) * portTICK_PERIOD_MS;
            
            if (elapsed_time > GRACE_PERIOD_MS) {
                ESP_LOGW(TAG, "Signal lost >60s! Forcing lock.");
                filtered_distance = 10.0; 
                in_timeout = true; 
            } else {
                ESP_LOGD(TAG, "Grace period active: %d/60s", (int)(elapsed_time / 1000));
            }
        }

        // HID logic (only if PC is connected)
        if (pc_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            
            if (filtered_distance > THRESHOLD_LOCK && !pc_is_locked) {
                pc_is_locked = true;
                ESP_LOGI(TAG, "Target lost. Sending Win + L...");
                ble_hid_send_key(pc_conn_handle, 0x08, 0x0F); 
            } 
            else if (filtered_distance < THRESHOLD_RESET && pc_is_locked) {
                pc_is_locked = false;
                ESP_LOGI(TAG, "Target in range. Waking up PC...");
                
                ble_hid_send_key(pc_conn_handle, 0x00, 0x2C); // Wake (Space)
                vTaskDelay(pdMS_TO_TICKS(1500));              // Wait for OS prompt
                ble_hid_type_password(pc_conn_handle);        // Type password
            }
        }
    }
}

/* BLE GAP event handler */
static int ble_app_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                pc_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "PC Connected!");
                ble_app_scan(); 
            } else {
                ESP_LOGE(TAG, "Connection failed; status=%d", event->connect.status);
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "PC Disconnected. Reason: %d", event->disconnect.reason);
            pc_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise(); 
            return 0;

        case BLE_GAP_EVENT_DISC:
            if (memcmp(event->disc.addr.val, TARGET_MAC, 6) == 0) {
                int8_t rssi = event->disc.rssi;
                xQueueSend(rssi_queue, &rssi, 0);
            }
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGW(TAG, "Scan stopped (reason: %d). Restarting...", event->disc_complete.reason);
            ble_app_scan();
            return 0;

        default:
            return 0;
    }
}

/* Configures and starts advertising to PC */
static void ble_app_advertise(void) {
    if (ble_gap_adv_active()) return; 
    
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    fields.appearance = 0x03C1; // Keyboard
    fields.appearance_is_present = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertising; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising to PC started...");
    }
}

/* NimBLE host sync callback */
static void ble_app_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_app_advertise();
    ble_app_scan();
}

/* NimBLE host task */
void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    // Custom MAC address for static Windows pairing
    uint8_t new_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x02, 0x01};
    esp_base_mac_addr_set(new_mac);

    // NVS initialization (preserves bonding keys)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    rssi_queue = xQueueCreate(10, sizeof(int8_t));
    if (rssi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue!");
        return;
    }

    xTaskCreate(security_task, "security_task", 4096, NULL, 5, NULL);

    ESP_ERROR_CHECK(nimble_port_init());
    
    // Security Manager Config (Required for HID)
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; 
    ble_hs_cfg.sm_bonding = 1;                  
    ble_hs_cfg.sm_mitm = 0;                     
    ble_hs_cfg.sm_sc = 1;                       
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    ble_svc_gap_device_name_set("ESP-Key-V2");
    
    gatt_svr_init(); 
    nimble_port_freertos_init(ble_host_task);

    vTaskDelete(NULL); 
}