#ifndef __RN_CONFIG
#define __RN_CONFIG

#include <Arduino.h>
#include <cstdint>

#include "esp_efuse.h" // for name
#include "esp_mac.h" // for name

#include <Preferences.h>
Preferences prefs;

//#include <ESPCPUTemp.h>
//ESPCPUTemp tempSensor;

#if defined(BOARD_ESP32DOIT)

#ifdef DEBUG_ENABLED
#define DEBUG_RX 4
#define DEBUG_TX 5
HardwareSerial DebugSerial(1);
#endif

#define USE_WIFI
//#define USE_BT

#define M0_PIN 21
#define M1_PIN 19
#define AUX_PIN 15

#define DISPLAY_SCL 33
#define DISPLAY_SDA 32

#define BAT_ADC_PIN     25

#elif defined(BOARD_ESP32C3)

#ifdef DEBUG_ENABLED
#define DEBUG_RX 20
#define DEBUG_TX 21
#define SOFTWARE_SERIAL_DEBUG
#include <SoftwareSerial.h>
SoftwareSerial DebugSerial(DEBUG_RX,DEBUG_TX);
#endif

//#define USE_WIFI
#define USE_BT

#define E32_TX_PIN 3
#define E32_RX_PIN 4

#define M0_PIN 6
#define M1_PIN 7
#define AUX_PIN 5

#define DISPLAY_SCL 9
#define DISPLAY_SDA 8

#define BAT_ADC_PIN     0

#else
 // nothing
#endif

#define MTU 508

// KISS protocol constants (matching Reticulum RNodeInterface.py)

#define RADIO_STATE_OFF 0x00
#define RADIO_STATE_ON  0x01
#define RADIO_STATE_ASK 0xFF

const uint8_t FEND = 0xC0;
const uint8_t FESC = 0xDB;
const uint8_t TFEND = 0xDC;
const uint8_t TFESC = 0xDD;

const uint8_t CMD_UNKNOWN     = 0xFE;
const uint8_t CMD_DATA = 0x00;
const uint8_t CMD_FREQUENCY   = 0x01;
const uint8_t CMD_BANDWIDTH   = 0x02;
const uint8_t CMD_TXPOWER     = 0x03;
const uint8_t CMD_SF          = 0x04;
const uint8_t CMD_CR          = 0x05;
const uint8_t CMD_RADIO_STATE = 0x06;
const uint8_t CMD_RADIO_LOCK  = 0x07;
const uint8_t CMD_ST_ALOCK    = 0x0B;
const uint8_t CMD_LT_ALOCK    = 0x0C;
const uint8_t CMD_DETECT      = 0x08;
const uint8_t CMD_LEAVE       = 0x0A;
const uint8_t CMD_READY       = 0x0F;
const uint8_t CMD_STAT_RX     = 0x21;
const uint8_t CMD_STAT_TX     = 0x22;
const uint8_t CMD_STAT_RSSI   = 0x23;
const uint8_t CMD_STAT_SNR    = 0x24;
const uint8_t CMD_STAT_CHTM   = 0x25;
const uint8_t CMD_STAT_PHYPRM = 0x26;
const uint8_t CMD_STAT_BAT    = 0x27;
const uint8_t CMD_STAT_CSMA   = 0x28;
const uint8_t CMD_STAT_TEMP   = 0x29;
const uint8_t CMD_BLINK       = 0x30;
const uint8_t CMD_RANDOM      = 0x40;
const uint8_t CMD_FB_EXT      = 0x41;
const uint8_t CMD_FB_READ     = 0x42;
const uint8_t CMD_DISP_READ   = 0x66;
const uint8_t CMD_FB_WRITE    = 0x43;
const uint8_t CMD_BT_CTRL     = 0x46;
const uint8_t CMD_PLATFORM    = 0x48;
const uint8_t CMD_MCU         = 0x49;
const uint8_t CMD_FW_VERSION  = 0x50;
const uint8_t CMD_ROM_READ    = 0x51;
const uint8_t CMD_RESET       = 0x55;
const uint8_t CMD_BT_UNPAIR   = 0x70;
const uint8_t CMD_BT_PIN      = 0x62;
const uint8_t CMD_WIFI_MODE   = 0x6A;
const uint8_t CMD_WIFI_SSID   = 0x6B;
const uint8_t CMD_WIFI_PSK    = 0x6C;
  

const uint8_t DETECT_REQ      = 0x73;
const uint8_t DETECT_RESP     = 0x46;
    
const uint8_t CMD_ERROR           = 0x90;
const uint8_t ERROR_INITRADIO     = 0x01;
const uint8_t ERROR_TXFAILED      = 0x02;
const uint8_t ERROR_EEPROM_LOCKED = 0x03;
const uint8_t ERROR_QUEUE_FULL    = 0x04;
const uint8_t ERROR_MEMORY_LOW    = 0x05;
const uint8_t ERROR_MODEM_TIMEOUT = 0x06;

const uint8_t PLATFORM_AVR   = 0x90;
const uint8_t PLATFORM_ESP32 = 0x80;
const uint8_t PLATFORM_NRF52 = 0x70;

const uint8_t FW_MAJ           = 0x01;
const uint8_t FW_MIN           = 0x56;

#define WIFI_SSID_MAX_LEN  33
#define WIFI_PASS_MAX_LEN  65
char wifi_ssid[WIFI_SSID_MAX_LEN] = {0};
char wifi_psk[WIFI_PASS_MAX_LEN] = {0};
const uint16_t TCP_PORT   = 7633;

#define CABLE_STATE_DISCONNECTED 0x00
#define CABLE_STATE_CONNECTED    0x01
uint8_t cable_state = CABLE_STATE_DISCONNECTED;

#define BT_STATE_NA        0xff
#define BT_STATE_OFF       0x00
#define BT_STATE_ON        0x01
#define BT_STATE_PAIRING   0x02
#define BT_STATE_CONNECTED 0x03
uint8_t bt_state = BT_STATE_NA;
uint32_t bt_ssp_pin = 0;
bool bt_ready = false;
bool bt_enabled = false;
bool bt_allow_pairing = false;

bool wifi_enabled = false;
bool wifi_active = false;


bool console_active = false;

char device_name[11] = "RNode 0000";

uint8_t e32_modem_errors = 0;

uint32_t cur_f = 170000000;
uint32_t cur_bw = 250000;
uint8_t  cur_tx = 17;
uint8_t  cur_sf = 10;
uint8_t  cur_cr = 7;
uint8_t  cur_radio_state = RADIO_STATE_OFF; // turn on when RNS interface connected

void read_device_name() {
  uint8_t base_mac[8];
  esp_efuse_mac_get_default(base_mac);
  sprintf(device_name, "RNode %02X%02X", base_mac[4], base_mac[5]);
}

/*
float current_temperature = 0;

void init_temperature_sensor() {
  if (!tempSensor.begin()) {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("Failed to initialize temperature sensor");
    #endif
  }
}

float temp_measure() {
  if (tempSensor.tempAvailable()) {
    float temp = tempSensor.getTemp();
    if (!isnan(temp)) {
      //#ifdef DEBUG_ENABLED
      //DebugSerial.print("CPU Temperature: ");
      //DebugSerial.print(temp);
      //DebugSerial.println(" °C");
      //#endif
      return temp;
    } else {
      //#ifdef DEBUG_ENABLED
      //DebugSerial.println("Failed to read temperature");
      //#endif
      return 0;
    }
  } else {
    return 0;
  }
}
*/

struct BatteryPoint {
  float voltage;
  int percentage;
};

const BatteryPoint battery_curve[] = {
  {4.20, 100},
  {4.15,  95},
  {4.10,  90},
  {4.05,  85},
  {4.00,  80},
  {3.95,  75},
  {3.90,  70},
  {3.85,  65},
  {3.80,  60},
  {3.75,  52},
  {3.70,  45},
  {3.65,  37},
  {3.60,  30},
  {3.55,  22},
  {3.50,  15},
  {3.45,   8},
  {3.40,   4},
  {3.35,   1},
  {3.30,   0}
};

const int battery_curve_size = sizeof(battery_curve) / sizeof(battery_curve[0]);

int current_power_percents = 0;

void cfg_power_measure() {
  pinMode(BAT_ADC_PIN, INPUT);
  analogSetAttenuation(ADC_11db);
}

float read_battery_voltage() {
  const int samples = 32;
  uint32_t sum = 0;
  
  for(int i = 0; i < samples; i++) {
    sum += analogReadMilliVolts(BAT_ADC_PIN);
    delayMicroseconds(100);
  }
  
  float voltage_at_pin = (sum / (float)samples) / 1000.0f;
  float battery_voltage = voltage_at_pin * 2.0f;
  
  return battery_voltage;
}

int voltage_to_percentage(float voltage) {
  if (voltage >= 4.20) return 100;
  if (voltage <= 3.30) return 0;

  for (int i = 0; i < battery_curve_size - 1; i++) {
    if (voltage >= battery_curve[i+1].voltage && voltage <= battery_curve[i].voltage) {
      
      float v1 = battery_curve[i].voltage;
      float v2 = battery_curve[i+1].voltage;
      int   p1 = battery_curve[i].percentage;
      int   p2 = battery_curve[i+1].percentage;

      float ratio = (voltage - v2) / (v1 - v2);
      return p2 + (int)(ratio * (p1 - p2));
    }
  }
  
  return 0;
}

#endif