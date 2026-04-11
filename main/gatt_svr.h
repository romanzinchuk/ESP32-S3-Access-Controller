#ifndef GATT_SVR_H
#define GATT_SVR_H

void gatt_svr_init(void);
void ble_hid_send_key(uint16_t conn_handle, uint8_t modifier, uint8_t key_code);

#endif /* GATT_SVR_H */