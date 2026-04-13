#ifndef GATT_SVR_H
#define GATT_SVR_H

#include <stdint.h>

// Initializes GATT server and HID services
void gatt_svr_init(void);

// Sends a keystroke to the connected PC
void ble_hid_send_key(uint16_t conn_handle, uint8_t modifier, uint8_t key_code);

#endif /* GATT_SVR_H */