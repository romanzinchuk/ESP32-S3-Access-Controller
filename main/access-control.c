#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* NimBLE файли */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "PC_LOCK_SYSTEM";

/* Стан системи */
static bool pc_is_locked = false;
static float filtered_distance = 0.0;

/* Константи для диплома (Романе, підкоригуй MEASURED_POWER під свій телефон) */
#define MEASURED_POWER -60  // RSSI на відстані 1 метр
#define N_COEFF 3.0         // Коефіцієнт середовища (3.0 - стандарт для приміщення)
#define ALPHA 0.15          // Коефіцієнт згладжування EMA (0.15 - менше шуму)
#define THRESHOLD_LOCK 2.0  // Поріг блокування в метрах
#define THRESHOLD_RESET 1.5 // Поріг розблокування (гістерезис)

#define TARGET_DEVICE_NAME "Iphone_Roman"

/* Декларація функцій */
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);

static void ble_app_scan(void) {
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Помилка визначення типу адреси; rc=%d", rc);
        return;
    }

    disc_params.filter_duplicates = 0; 
    disc_params.passive = 1;
    
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Помилка запуску сканування; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Сканування ефіру розпочато. Шукаю: %s", TARGET_DEVICE_NAME);
    }
}

static int ble_app_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (rc != 0) return 0;

            if (fields.name != NULL && fields.name_len > 0) {
                if (fields.name_len == strlen(TARGET_DEVICE_NAME) && 
                    strncmp((char *)fields.name, TARGET_DEVICE_NAME, fields.name_len) == 0) {
                    
                    float raw_distance = pow(10, (float)(MEASURED_POWER - event->disc.rssi) / (10 * N_COEFF));

                    if (filtered_distance == 0.0) {
                        filtered_distance = raw_distance;
                    } else {
                        filtered_distance = (ALPHA * raw_distance) + ((1.0 - ALPHA) * filtered_distance);
                    }

                    ESP_LOGI(TAG, "RSSI: %d | Дистанція: %.2f м", event->disc.rssi, filtered_distance);

                    if (filtered_distance > THRESHOLD_LOCK && !pc_is_locked) {
                        printf("CMD_LOCK_PC\n");
                        pc_is_locked = true;
                        ESP_LOGW(TAG, "!!! КОМАНДА БЛОКУВАННЯ !!!");
                    } 
                    else if (filtered_distance < THRESHOLD_RESET && pc_is_locked) {
                        pc_is_locked = false;
                        ESP_LOGI(TAG, "Користувач повернувся. Готовність до блокування.");
                    }
                }
            }
            return 0;

        default:
            return 0;
    }
}

static void ble_app_on_sync(void) {
    ble_hs_util_ensure_addr(0);
    ble_app_scan();
}

void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    esp_err_t ret;

    // 1. Ініціалізація NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Ініціалізація NimBLE
    ESP_ERROR_CHECK(nimble_port_init());

    // 3. Конфігурація NimBLE
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    
    // Встановлюємо ім'я пристрою в стеку (для формальності)
    ble_svc_gap_device_name_set("ESP32S3-Lock");

    // 4. Запуск задачі
    nimble_port_freertos_init(ble_host_task);

    // 5. Heartbeat loop (щоб монітор не вилітав і ми бачили, що процесор працює)
    while (1) {
        ESP_LOGD(TAG, "System uptime: %lld ms", esp_timer_get_time() / 1000);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}