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

static const char *TAG = "PC_LOCK";

/* Target IMISW12 MAC Address (Little Endian) */
static const uint8_t TARGET_MAC[6] = {0x9F, 0x68, 0x07, 0xB7, 0x02, 0x78};

/* System state */
static bool pc_is_locked = false;
static float filtered_distance = 0.0;
static uint16_t pc_conn_handle = BLE_HS_CONN_HANDLE_NONE; 

/* RTOS Components */
static QueueHandle_t rssi_queue;

/* RSSI Config */
#define MEASURED_POWER -60  // RSSI at 1m
#define N_COEFF 3.0         // Environmental factor
#define ALPHA 0.15          // EMA filter coefficient
#define THRESHOLD_LOCK 2.0  // Lock distance (m)
#define THRESHOLD_RESET 1.5 // Unlock distance (m)

/* Declarations */
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);

static void ble_app_scan(void) {

    if (ble_gap_disc_active()) {
        return; 
    }

    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Address type inference failed (rc=%d)", rc);
        return;
    }

    disc_params.filter_duplicates = 0; 
    disc_params.passive = 1;
    
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Scan start failed (rc=%d)", rc);
    } else {
        ESP_LOGI(TAG, "Scanning for target MAC started");
    }
}

/* ---------------------------------------------------------
 * RTOS TASK: SECURITY LOGIC (Consumer)
 * --------------------------------------------------------- */
void security_task(void *pvParameters) {
    int8_t current_rssi;

    ESP_LOGI(TAG, "Security Task started. Waiting for RSSI data...");

    while (1) {
        if (xQueueReceive(rssi_queue, &current_rssi, portMAX_DELAY) == pdPASS) {
            
            float raw_distance = pow(10, (float)(MEASURED_POWER - current_rssi) / (10 * N_COEFF));

            if (filtered_distance == 0.0) {
                filtered_distance = raw_distance;
            } else {
                filtered_distance = (ALPHA * raw_distance) + ((1.0 - ALPHA) * filtered_distance);
            }

            ESP_LOGD(TAG, "RSSI: %d | Dist: %.2fm", current_rssi, filtered_distance);

            // Execute lock/unlock logic ONLY if PC is connected
            if (pc_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                if (filtered_distance > THRESHOLD_LOCK && !pc_is_locked) {
                    pc_is_locked = true;
                    ESP_LOGI(TAG, "Target lost. Sending Win + L to lock PC...");
                    
                    // Відправка Win + L
                    ble_hid_send_key(pc_conn_handle, 0x08, 0x0F);
                } 
                else if (filtered_distance < THRESHOLD_RESET && pc_is_locked) {
                    pc_is_locked = false;
                    ESP_LOGI(TAG, "Target in range. Waking up PC...");
                    
                    // Відправка Пробілу (щоб розбудити екран)
                    ble_hid_send_key(pc_conn_handle, 0x00, 0x2C);
                }
            }
        }
    }
}

/* ---------------------------------------------------------
 * BLE GAP EVENT HANDLER (Dual Role Brain)
 * --------------------------------------------------------- */
static int ble_app_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        
        /* ROLE 1: PERIPHERAL (PC connects/disconnects) */
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                pc_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "PC Connected successfully!");
            } else {
                ESP_LOGE(TAG, "Connection failed; status=%d", event->connect.status);
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "PC Disconnected. Reason: %d", event->disconnect.reason);
            pc_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise();
            return 0;

        /* ROLE 2: SCANNER (Looking for the watch) */
        case BLE_GAP_EVENT_DISC:
            if (memcmp(event->disc.addr.val, TARGET_MAC, 6) == 0) {
                int8_t rssi = event->disc.rssi;
                xQueueSend(rssi_queue, &rssi, 0);
            }
            return 0;

        default:
            return 0;
    }
}

/* ---------------------------------------------------------
 * ADVERTISING FOR PC
 * --------------------------------------------------------- */
static void ble_app_advertise(void) {
    if (ble_gap_adv_active()) {
        return; // Already advertising, do nothing
    }
    
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

    // Appearance 0x03C1 indicates a Keyboard
    fields.appearance = 0x03C1; 
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

static void ble_app_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_app_advertise();
    ble_app_scan();
}

void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
esp_err_t ret;

    uint8_t new_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x04};
    esp_base_mac_addr_set(new_mac);
    
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    rssi_queue = xQueueCreate(10, sizeof(int8_t));

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nvs_flash_init();
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
    
    /* Configure Security Manager (Required for HID) */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; 
    ble_hs_cfg.sm_bonding = 1;                  
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;                       
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    /* Встановлюємо ім'я */
    ble_svc_gap_device_name_set("ESP-Key-V2");
    
    /* Initialize GATT Server from external file */
    gatt_svr_init();

    nimble_port_freertos_init(ble_host_task);

    vTaskDelete(NULL); 
}