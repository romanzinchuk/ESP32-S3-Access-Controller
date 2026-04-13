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

/* Target smartwatch MAC Address (Little Endian format for NimBLE) */
static const uint8_t TARGET_MAC[6] = {0x9F, 0x68, 0x07, 0xB7, 0x02, 0x78};

/* System state tracking */
static bool pc_is_locked = false;
static float filtered_distance = 0.0;
static uint16_t pc_conn_handle = BLE_HS_CONN_HANDLE_NONE; 

/* RTOS communication */
static QueueHandle_t rssi_queue;

/* Distance calculation and filtering constants */
#define MEASURED_POWER -60  // Expected RSSI at 1 meter distance
#define N_COEFF 3.0         // Environmental signal loss factor
#define ALPHA 0.05          // Exponential Moving Average (EMA) filter coefficient
#define THRESHOLD_LOCK 2.0  // Distance (in meters) to trigger PC lock
#define THRESHOLD_RESET 1.5 // Distance (in meters) to trigger PC unlock

/* Function declarations */
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);

/* * Configures and starts the BLE scanner to find the target smartwatch.
 * Uses a "Hard Reset" approach to prevent the BLE controller from 
 * silently dropping the scan process during PC connections.
 */
static void ble_app_scan(void) {
    // Forcefully cancel active scanning to clear radio buffers
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }

    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Address type inference failed (rc=%d)", rc);
        return;
    }

    disc_params.filter_duplicates = 0; 
    disc_params.passive = 0; // Active scanning (sends scan requests)
    
    // 50% Duty Cycle (100ms interval, 50ms window) to share radio with HID
    disc_params.itvl = 160; 
    disc_params.window = 80; 

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Scan start failed (rc=%d)", rc);
    } else {
        ESP_LOGI(TAG, "Scanning for target MAC started (HARD RESET)");
    }
}

/* ---------------------------------------------------------
 * RTOS TASK: SECURITY LOGIC
 * Processes RSSI data, calculates distance, and triggers HID commands.
 * --------------------------------------------------------- */
void security_task(void *pvParameters) {
    int8_t current_rssi;

    ESP_LOGI(TAG, "Security Task started. Waiting for RSSI data...");

    while (1) {
        // Wait up to 10 seconds for a new RSSI packet from the watch
        if (xQueueReceive(rssi_queue, &current_rssi, pdMS_TO_TICKS(10000)) == pdPASS) {
            
            // Logarithmic distance calculation
            float raw_distance = pow(10, (float)(MEASURED_POWER - current_rssi) / (10 * N_COEFF));

            // Apply EMA filter to smooth out signal spikes and reflections
            if (filtered_distance == 0.0) {
                filtered_distance = raw_distance;
            } else {
                filtered_distance = (ALPHA * raw_distance) + ((1.0 - ALPHA) * filtered_distance);
            }

            ESP_LOGD(TAG, "RSSI: %d | Dist: %.2fm", current_rssi, filtered_distance);

        } else {
            // Timeout: Watch is out of range or in deep sleep
            ESP_LOGW(TAG, "Watch signal timeout! Forcing maximum distance.");
            filtered_distance = 10.0; 
        }

        // HID Command Logic (Only execute if connected to PC)
        if (pc_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            
            // Lock Condition: User walked away
            if (filtered_distance > THRESHOLD_LOCK && !pc_is_locked) {
                pc_is_locked = true;
                ESP_LOGI(TAG, "Target lost. Pausing scanner to send Win + L...");
                
                // Pause scanner to give 100% radio priority to HID transmission
                if (ble_gap_disc_active()) ble_gap_disc_cancel(); 
                vTaskDelay(pdMS_TO_TICKS(100)); 

                ble_hid_send_key(pc_conn_handle, 0x08, 0x0F); // Win + L

                // Resume scanner after successful transmission
                vTaskDelay(pdMS_TO_TICKS(100));
                ble_app_scan();
            } 
            // Unlock Condition: User returned
            else if (filtered_distance < THRESHOLD_RESET && pc_is_locked) {
                pc_is_locked = false;
                ESP_LOGI(TAG, "Target in range. Pausing scanner to wake up PC...");
                
                if (ble_gap_disc_active()) ble_gap_disc_cancel();
                vTaskDelay(pdMS_TO_TICKS(100));

                ble_hid_send_key(pc_conn_handle, 0x00, 0x2C); // Spacebar

                vTaskDelay(pdMS_TO_TICKS(100));
                ble_app_scan();
            }
        }
    }
}

/* ---------------------------------------------------------
 * BLE GAP EVENT HANDLER 
 * Manages dual-role operations: PC connection and Watch scanning.
 * --------------------------------------------------------- */
static int ble_app_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        
        /* ROLE 1: PERIPHERAL (Handling Windows PC) */
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                pc_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "PC Connected successfully!");
                
                // Restart scanner. Controller often aborts scanning upon connection.
                ble_app_scan(); 
            } else {
                ESP_LOGE(TAG, "Connection failed; status=%d", event->connect.status);
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "PC Disconnected. Reason: %d", event->disconnect.reason);
            pc_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise(); // Resume broadcasting to PC
            return 0;

        /* ROLE 2: CENTRAL (Scanning for Watch) */
        case BLE_GAP_EVENT_DISC:
            // Filter incoming packets by target MAC address
            if (memcmp(event->disc.addr.val, TARGET_MAC, 6) == 0) {
                int8_t rssi = event->disc.rssi;
                // Send RSSI to RTOS task (non-blocking)
                xQueueSend(rssi_queue, &rssi, 0);
            }
            return 0;

        default:
            return 0;
    }
}

/* ---------------------------------------------------------
 * ADVERTISING FOR PC
 * Configures ESP32 to appear as an HID Keyboard to Windows.
 * --------------------------------------------------------- */
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

    // 0x03C1 dictates the device icon and driver type (Keyboard) in Windows
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

/* Synchronization callback when NimBLE host and controller are synced */
static void ble_app_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_app_advertise();
    ble_app_scan();
}

/* Dedicated RTOS task for NimBLE host stack */
void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    // Override default MAC to force a clean pairing state in Windows
    uint8_t new_mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x0C};
    esp_base_mac_addr_set(new_mac);

    // Initialize Non-Volatile Storage (NVS) for BLE bonding keys
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    // Create RTOS queue for RSSI data passing
    rssi_queue = xQueueCreate(10, sizeof(int8_t));
    if (rssi_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue!");
        return;
    }

    xTaskCreate(security_task, "security_task", 4096, NULL, 5, NULL);

    ESP_ERROR_CHECK(nimble_port_init());
    
    /* Security Manager Configuration (Strictly required for HID pairing) */
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; // No display or keyboard for PIN
    ble_hs_cfg.sm_bonding = 1;                  // Save pairing keys to NVS
    ble_hs_cfg.sm_mitm = 0;                     // Man-in-the-Middle protection off
    ble_hs_cfg.sm_sc = 1;                       // Secure Connections enabled
    ble_hs_cfg.sm_our_key_dist = 1;
    ble_hs_cfg.sm_their_key_dist = 1;
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    ble_svc_gap_device_name_set("ESP-Key-V2");
    
    gatt_svr_init(); // Initialize custom HID descriptors
    nimble_port_freertos_init(ble_host_task);

    vTaskDelete(NULL); // Delete main task as FreeRTOS takes over
}