#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "gatt_svr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------------------------------------------------
 * HID REPORT MAP (Keyboard Descriptor)
 * Instructs the OS on how to parse incoming HID data bytes.
 * Defines a standard 8-byte keyboard input report.
 * --------------------------------------------------------- */
static const uint8_t hid_report_map[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    
    0x85, 0x01,  //   Report ID (1)
    
    // Byte 0: Modifier keys (Ctrl, Shift, Alt, GUI/Win)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0xE0,  //   Usage Minimum (224)
    0x29, 0xE7,  //   Usage Maximum (231)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1 bit per modifier)
    0x95, 0x08,  //   Report Count (8 modifiers)
    0x81, 0x02,  //   Input (Data, Variable, Absolute)
    
    // Byte 1: Reserved byte (OEM specific)
    0x95, 0x01,  //   Report Count (1)
    0x75, 0x08,  //   Report Size (8 bits)
    0x81, 0x01,  //   Input (Constant)
    
    // Bytes 2-7: 6-key array for simultaneous key presses
    0x95, 0x06,  //   Report Count (6 keys)
    0x75, 0x08,  //   Report Size (8 bits per key)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x65,  //   Logical Maximum (101 keys)
    0x05, 0x07,  //   Usage Page (Key Codes)
    0x19, 0x00,  //   Usage Minimum (0)
    0x29, 0x65,  //   Usage Maximum (101)
    0x81, 0x00,  //   Input (Data, Array)
    
    0xC0         // End Collection
};

/* ---------------------------------------------------------
 * GATT SERVICES DEFINITION
 * Defines Device Info, Battery, and HID services.
 * --------------------------------------------------------- */
static uint16_t hid_input_handle;

/* Callback declarations for read/write requests from the OS */
static int gatt_svr_access_device_info(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_access_battery(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_access_hid(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Primary Service Table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Device Information Service (UUID: 180A) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* PnP ID: Identifies vendor and product specifics */
                .uuid = BLE_UUID16_DECLARE(0x2A50),
                .access_cb = gatt_svr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        }
    },
    {
        /* Battery Service (UUID: 180F) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Battery Level */
                .uuid = BLE_UUID16_DECLARE(0x2A19),
                .access_cb = gatt_svr_access_battery,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        }
    },
    {
        /* Human Interface Device (HID) Service (UUID: 1812) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* HID Information (Protocol Version, Country Code) */
                .uuid = BLE_UUID16_DECLARE(0x2A4A),
                .access_cb = gatt_svr_access_hid,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* Report Map (The descriptor defined above) */
                .uuid = BLE_UUID16_DECLARE(0x2A4B),
                .access_cb = gatt_svr_access_hid,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* HID Control Point (Host state indicators like Suspend) */
                .uuid = BLE_UUID16_DECLARE(0x2A4C),
                .access_cb = gatt_svr_access_hid,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* HID Input Report (Channel for sending keystrokes) */
                .uuid = BLE_UUID16_DECLARE(0x2A4D),
                .access_cb = gatt_svr_access_hid,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &hid_input_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        /* Report Reference Descriptor: Crucial for Windows HID drivers */
                        .uuid = BLE_UUID16_DECLARE(0x2908),
                        .access_cb = gatt_svr_access_hid,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    { 0 }
                }
            },
            { 0 }
        }
    },
    { 0 } 
};

/* ---------------------------------------------------------
 * CALLBACK IMPLEMENTATIONS
 * These functions trigger when the Host (Windows) requests data.
 * --------------------------------------------------------- */
static int gatt_svr_access_device_info(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0x2A50) { 
        // Dummy PnP ID indicating standard USB vendor
        uint8_t pnp_id[] = {0x02, 0xE5, 0x02, 0xAB, 0x12, 0x00, 0x01}; 
        os_mbuf_append(ctxt->om, pnp_id, sizeof(pnp_id));
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_access_battery(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0x2A19) { 
        // Hardcoded battery level (100%)
        uint8_t batt_lvl = 100; 
        os_mbuf_append(ctxt->om, &batt_lvl, 1);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_access_hid(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t uuid16 = 0;
    
    // Determine if the OS is reading a Characteristic or a Descriptor
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        uuid16 = ble_uuid_u16(ctxt->dsc->uuid);
    }

    if (uuid16 == 0x2A4A) { 
        // HID info: bcdHID 1.11, CountryCode 0, Flags (RemoteWake)
        uint8_t hid_info[] = {0x11, 0x01, 0x00, 0x03}; 
        os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
        return 0;
    }
    if (uuid16 == 0x2A4B) { 
        os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
        return 0;
    }
    if (uuid16 == 0x2A4D) { 
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            uint8_t empty_report[8] = {0}; 
            os_mbuf_append(ctxt->om, empty_report, sizeof(empty_report));
            return 0;
        }
    }
    if (uuid16 == 0x2908) { 
        // Report Reference: Report ID = 1, Report Type = 1 (Input)
        uint8_t rep_ref[] = {0x01, 0x01}; 
        os_mbuf_append(ctxt->om, rep_ref, sizeof(rep_ref));
        return 0;
    }
    return 0;
}

/* Initializes the GATT server and loads the service table */
void gatt_svr_init(void) {
    int rc;
    
    ble_svc_gap_init();
    ble_svc_gap_device_name_set("ESP32S3-Lock");
    
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    ESP_ERROR_CHECK(rc);
    
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    ESP_ERROR_CHECK(rc);
}

/* ---------------------------------------------------------
 * HID REPORT EXECUTOR
 * Simulates human keystrokes to the connected host.
 * --------------------------------------------------------- */
void ble_hid_send_key(uint16_t conn_handle, uint8_t modifier, uint8_t key_code) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    // Optional: Send empty "Wake-up ping" here if OS Selective Suspend is an issue

    // 1. Send Key Press (Modifier + Key)
    uint8_t press_report[8] = {modifier, 0, key_code, 0, 0, 0, 0, 0};
    struct os_mbuf *om = ble_hs_mbuf_from_flat(press_report, sizeof(press_report));
    ble_gatts_notify_custom(conn_handle, hid_input_handle, om);

    // Human typing simulation delay (crucial for OS debounce filters)
    vTaskDelay(pdMS_TO_TICKS(150));

    // 2. Send Key Release (Empty Report)
    uint8_t release_report[8] = {0};
    om = ble_hs_mbuf_from_flat(release_report, sizeof(release_report));
    ble_gatts_notify_custom(conn_handle, hid_input_handle, om);
    
    // Safety delay before subsequent commands
    vTaskDelay(pdMS_TO_TICKS(50));
}