// Copyright (C) 2024, Mark Qvist

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef __RN_BT
#define __RN_BT

#include <esp_bt_main.h>
#include "esp_bt_device.h"
#include "BLESerial.h"
BLESerial SerialBT;

#include "config.h"

#define BT_PAIRING_TIMEOUT 35000
#define BLE_FLUSH_TIMEOUT 20
uint32_t bt_pairing_started = 0;

#define BT_DEV_ADDR_LEN 6
#define BT_DEV_HASH_LEN 16
uint8_t dev_bt_mac[BT_DEV_ADDR_LEN];
char bt_da[BT_DEV_ADDR_LEN];
char bt_dh[BT_DEV_HASH_LEN];
char bt_devname[11];

void bt_conf_save(bool);
bool bt_setup_hw(); 
void bt_security_setup();
BLESecurity *ble_security = new BLESecurity();
bool ble_authenticated = false;
uint32_t pairing_pin = 0;


void bt_flush() { if (bt_state == BT_STATE_CONNECTED) { SerialBT.flush(); } }

void bt_start() {
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("BT start, device: %s\n", bt_devname);
  #endif

  //display_unblank();
  if (bt_state == BT_STATE_OFF) {
    bt_state = BT_STATE_ON;
    SerialBT.begin(bt_devname);
    SerialBT.setTimeout(10);
  }
}

void bt_stop() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("BT stop");
  #endif
  //display_unblank();
  if (bt_state != BT_STATE_OFF) {
    bt_allow_pairing = false;
    bt_state = BT_STATE_OFF;
    SerialBT.end();
  }
}

bool bt_init() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("BT init");
  #endif
  bt_state = BT_STATE_OFF;
  if (bt_setup_hw()) {
    if (bt_enabled && !console_active) bt_start();
    return true;
  } else {
    return false;
  }
}

void bt_debond_all() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("Debonding all");
  #endif
  int dev_num = esp_ble_get_bond_device_num();
  esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  for (int i = 0; i < dev_num; i++) { esp_ble_remove_bond_device(dev_list[i].bd_addr); }
  free(dev_list);
}

void bt_enable_pairing() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("BT enable pairing");
  #endif
  //display_unblank();
  if (bt_state == BT_STATE_OFF) bt_start();

  bt_security_setup();

  bt_allow_pairing = true;
  bt_pairing_started = millis();
  bt_state = BT_STATE_PAIRING;
  bt_ssp_pin = pairing_pin;
}

void bt_disable_pairing() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("BT disable pairing");
  #endif
  //display_unblank();
  bt_allow_pairing = false;
  bt_ssp_pin = 0;
  bt_state = BT_STATE_ON;
}

// Must assign this in setup()
void(*bt_kiss_indicate_btpin_fn_ptr)(void);

void bt_passkey_notify_callback(uint32_t passkey) {
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("Got passkey notification: %d\n", passkey);
  #endif
  if (bt_allow_pairing) {
    bt_ssp_pin = passkey;
    bt_pairing_started = millis();
    bt_kiss_indicate_btpin_fn_ptr();
  } else {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("Pairing not allowed, re-init");
    #endif
    SerialBT.disconnect();
  }
}

bool bt_confirm_pin_callback(uint32_t pin) {
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("Confirm PIN callback: %d\n", pin);
  #endif
  return true;
}

void bt_update_passkey() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("Updating passkey");
  #endif
  pairing_pin = random(899999)+100000;
  bt_ssp_pin = pairing_pin;
}

uint32_t bt_passkey_callback() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("API passkey request");
  #endif
  if (pairing_pin == 0) { bt_update_passkey(); }
  return pairing_pin;
}

bool bt_client_authenticated() {
  return ble_authenticated;
}

bool bt_security_request_callback() {
  if (bt_allow_pairing) {
      #ifdef DEBUG_ENABLED
      DebugSerial.println("Accepting security request");
      #endif
      return true;
    } else {
      #ifdef DEBUG_ENABLED
      DebugSerial.println("Rejecting security request");
      #endif
      return false;
    }
}

void bt_authentication_complete_callback(esp_ble_auth_cmpl_t auth_result) {
  if (auth_result.success == true) {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("Authentication success");
    #endif
    ble_authenticated = true;
    if (bt_state == BT_STATE_PAIRING) {
      #ifdef DEBUG_ENABLED
      DebugSerial.println("Pairing complete, disconnecting");
      #endif
      delay(2000); 
      SerialBT.disconnect();
    } else { bt_state = BT_STATE_CONNECTED; }
  } else {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("Authentication fail");
    #endif
    ble_authenticated = false;
    bt_state = BT_STATE_ON;
    bt_update_passkey();
    bt_security_setup();
  }
  bt_allow_pairing = false;
  bt_ssp_pin = 0;
}

void bt_connect_callback(BLEServer *server) {
  uint16_t conn_id = server->getConnId();
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("Connected: %d\n", conn_id);
  #endif
  //display_unblank();
  ble_authenticated = false;
  if (bt_state != BT_STATE_PAIRING) { bt_state = BT_STATE_CONNECTED; }
  cable_state = CABLE_STATE_DISCONNECTED;
}

void bt_disconnect_callback(BLEServer *server) {
  uint16_t conn_id = server->getConnId();
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("Disconnected: %d\n", conn_id);
  #endif
  //display_unblank();
  ble_authenticated = false;
  bt_state = BT_STATE_ON;
}

void bt_conf_save(bool is_enabled) {
  prefs.begin("rnode", false);
  uint8_t ble_flag  = is_enabled ? 1 : 0;
  prefs.putUChar("ble_flag", ble_flag);
  prefs.end();
}

bool bt_setup_hw() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("BT setup hw");
  #endif
  if (!bt_ready) {
    prefs.begin("rnode", true); // read only
    uint8_t ble_flag  = prefs.getUChar("ble_flag", 0);
    #ifdef DEBUG_ENABLED
    DebugSerial.printf("Stored BT flag is: %d\n", ble_flag);
    #endif
    prefs.end();
    if (ble_flag) {
      bt_enabled = true;
    } else {
      bt_enabled = false;
    }
    if (btStart()) {
      if (esp_bluedroid_init() == ESP_OK) {
        if (esp_bluedroid_enable() == ESP_OK) {
          const uint8_t* bda_ptr = esp_bt_dev_get_address();
          //char *data = (char*)malloc(BT_DEV_ADDR_LEN+1);
          //for (int i = 0; i < BT_DEV_ADDR_LEN; i++) {
          //    data[i] = bda_ptr[i];
          //}
          //data[BT_DEV_ADDR_LEN] = EEPROM.read(eeprom_addr(ADDR_SIGNATURE));
          //unsigned char *hash = MD5::make_hash(data, BT_DEV_ADDR_LEN);
          //memcpy(bt_dh, hash, BT_DEV_HASH_LEN);
          //sprintf(bt_devname, "RNode %02X%02X", bt_dh[14], bt_dh[15]);
          //free(data);

          //uint8_t base_mac[8];
          //esp_efuse_mac_get_default(base_mac);
          //sprintf(bt_devname, "RNode %02X%02X", base_mac[4], base_mac[5]);
          strcpy(bt_devname, device_name);

          bt_security_setup();

          bt_ready = true;
          return true;

        } else { return false; }
      } else { return false; }
    } else { return false; }
  } else { return false; }
}

void bt_security_setup() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("Executing BT security setup");
  #endif
  if (pairing_pin == 0) { bt_update_passkey(); }
  uint32_t passkey = pairing_pin;
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("Passkey is %d\n", passkey);
  #endif

  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
  uint8_t oob_support = ESP_BLE_OOB_DISABLE;

  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;

  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void update_bt() {
  if (bt_allow_pairing && millis()-bt_pairing_started >= BT_PAIRING_TIMEOUT) {
    bt_disable_pairing();
  }
  if (bt_state == BT_STATE_CONNECTED && millis()-SerialBT.lastFlushTime >= BLE_FLUSH_TIMEOUT) {
    if (SerialBT.transmitBufferLength > 0) {
      bt_flush();
    }
  }
}

#endif