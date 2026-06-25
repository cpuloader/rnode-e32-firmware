
#define E32_TTL_1W

#include "LoRa_E32.h"

#include <AceCRC.h>
using namespace ace_crc::crc16modbus_nibble;

#include "config.h"

#ifdef USE_WIFI
#include "remote.h"
#endif

#ifdef USE_BT
#include "Bluetooth.h"
#endif

#ifdef USE_DISPLAY
#include "Display.h"
#endif

#if defined(BOARD_ESP32DOIT)
LoRa_E32 e32ttl(&Serial2, AUX_PIN, M0_PIN, M1_PIN);
#elif defined(BOARD_ESP32C3)
LoRa_E32 e32ttl(E32_TX_PIN, E32_RX_PIN, &Serial1, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600, SERIAL_8N1);
#endif

unsigned long last_power_time = 0;
//unsigned long last_temp_time = 0;

// KISS receive buffer (Reticulum Rnode MTU is 508, we use a safe size)
uint8_t kiss_rx_buffer[512];
uint16_t kiss_rx_index = 0;
bool kiss_in_frame = false;
bool kiss_escaped = false;

// RNS outgoing packet queue
#define MAX_QUEUED_PACKETS 5

uint8_t  tx_packet_queue[MAX_QUEUED_PACKETS][MTU];
uint16_t tx_packet_sizes[MAX_QUEUED_PACKETS];
uint8_t  tx_queue_head = 0;
uint8_t  tx_queue_tail = 0;
uint8_t  tx_queue_count = 0;

// E32 LoRa packets are 58 max
#define E32_MAX_SIZE_PACKET MAX_SIZE_TX_PACKET

// E32 RX buffer
#define LOCAL_E32_RX_BUFFER_SIZE 4096
uint8_t  e32_rx_buffer[LOCAL_E32_RX_BUFFER_SIZE];
uint16_t e32_rx_len = 0;
unsigned long bytes_since_last_valid = 0;
unsigned long last_rx_time = 0;

void reset_rx_buffer() {
  e32_rx_len = 0;
  bytes_since_last_valid = 0;
}

// E32 stuff
void printParameters(struct Configuration configuration);
ResponseStatus send_to_e32(const uint8_t* data, uint16_t total_len);
void e32_process_rx();

// TX queue stuff
bool tx_queue_push(const uint8_t* data, uint16_t len) {
  if (tx_queue_count >= MAX_QUEUED_PACKETS || len > MTU) return false;
  memcpy(tx_packet_queue[tx_queue_head], data, len);
  tx_packet_sizes[tx_queue_head] = len;
  tx_queue_head = (tx_queue_head + 1) % MAX_QUEUED_PACKETS;
  tx_queue_count++;
  return true;
}

bool tx_queue_pop(uint8_t* &data, uint16_t &len) {
  if (tx_queue_count == 0) return false;
  data = tx_packet_queue[tx_queue_tail];
  len = tx_packet_sizes[tx_queue_tail];
  tx_queue_tail = (tx_queue_tail + 1) % MAX_QUEUED_PACKETS;
  tx_queue_count--;
  return true;
}

void reset_tx_queue() {
  tx_queue_count = 0;
  tx_queue_head = 0;
  tx_queue_tail = 0;
}

// Traffic statistics
#define STATS_WINDOW_MINUTES 10
uint32_t rx_bytes_per_minute[STATS_WINDOW_MINUTES] = {0};
uint32_t tx_bytes_per_minute[STATS_WINDOW_MINUTES] = {0};
uint8_t stats_minute_index = 0;
unsigned long last_stats_update = 0;

uint32_t total_rx_bytes = 0;
uint32_t total_tx_bytes = 0;

void update_traffic_stats(uint32_t rx_added = 0, uint32_t tx_added = 0) {
  total_rx_bytes += rx_added;
  total_tx_bytes += tx_added;

  unsigned long now = millis();

  if (now - last_stats_update > 60000UL) {  // 1 minute window
    stats_minute_index = (stats_minute_index + 1) % STATS_WINDOW_MINUTES;
    
    rx_bytes_per_minute[stats_minute_index] = 0;
    tx_bytes_per_minute[stats_minute_index] = 0;
    
    last_stats_update = now;
  }

  rx_bytes_per_minute[stats_minute_index] += rx_added;
  tx_bytes_per_minute[stats_minute_index] += tx_added;
}

void reset_traffic_stats() {
  total_rx_bytes = 0;
  total_tx_bytes = 0;
  stats_minute_index = 0;
  for (int i = 0; i < STATS_WINDOW_MINUTES; i++) {
    rx_bytes_per_minute[i] = 0;
    tx_bytes_per_minute[i] = 0;
  }
}

void stats_for_display() {
  uint32_t rx_10min = 0, tx_10min = 0;
  for (int i = 0; i < STATS_WINDOW_MINUTES; i++) {
    rx_10min += rx_bytes_per_minute[i];
    tx_10min += tx_bytes_per_minute[i];
  }
  rx_display_stats = rx_10min;
  tx_display_stats = tx_10min;
}

// Air Time / Duty Cycle
const unsigned long AIR_TIME_WINDOW_MS = 600000UL; // 10 min
float lt_airtime_limit = 0;     // 0.0 - 1.0 of AIR_TIME_WINDOW_MS (recommended 5-15%)

unsigned long air_time_used_ms = 0;
unsigned long air_time_window_start = 0;

bool airtime_lock = false;
unsigned long last_duty_log_time = 0;

//unsigned long last_lbt_time = 0;

bool can_transmit() {
  // Kind of primitive LBT, if it is LOW, we can be receiving.
  /*if (digitalRead(AUX_PIN) == LOW) {
    last_lbt_time = millis();
    return false;
  }
  // Wait a little after last low AUX, we can be between RX chunks
  if (millis() - last_lbt_time < 150) return false;*/

  if (lt_airtime_limit == 0) return true;

  unsigned long now = millis();

  if (now - air_time_window_start > AIR_TIME_WINDOW_MS) {
    air_time_used_ms = 0;
    air_time_window_start = now;
    airtime_lock = false;
  }

  unsigned long max_allowed_ms = (unsigned long)((float)AIR_TIME_WINDOW_MS * lt_airtime_limit);

  if (air_time_used_ms > max_allowed_ms) {
    if (!airtime_lock || (now - last_duty_log_time > 10000)) {
      #ifdef DEBUG_ENABLED
      float percent = (float)air_time_used_ms * 100.0 / AIR_TIME_WINDOW_MS;
      unsigned long time_left = AIR_TIME_WINDOW_MS - (now - air_time_window_start);
      DebugSerial.printf("Duty Cycle Limit! %.1f%% used. Waiting ~%lu sec\n", 
                        percent, (time_left + 999)/1000);
      #endif
      last_duty_log_time = now;
      airtime_lock = true;
    }
    return false;
  }

  airtime_lock = false;
  return true;
}

// Update after successful transmission
void update_airtime(unsigned long tx_duration_ms) {
  air_time_used_ms += tx_duration_ms;
  
  #ifdef DEBUG_ENABLED
  static unsigned long last_stat = 0;
  if (millis() - last_stat > 10000) {
    float percent = (float)air_time_used_ms * 100.0 / AIR_TIME_WINDOW_MS;
    DebugSerial.printf("[AirTime] Used: %.1f seconds, (%.1f%%) in last 10 min\n", (float)air_time_used_ms/1000.0f, percent);
    last_stat = millis();
  }
  #endif
}

// Main stuff
void send_kiss_packet(uint8_t cmd, const uint8_t* data, uint16_t len);
void send_kiss_packet_serial(uint8_t cmd, const uint8_t* data, uint16_t len);
void send_kiss_ready();
void process_kiss_byte(uint8_t byte);
void process_kiss_frame();

void kiss_indicate_detect();
void kiss_indicate_fw_version();
void kiss_indicate_platform();
void kiss_indicate_mcu();
void kiss_indicate_frequency();
void kiss_indicate_bandwidth();
void kiss_indicate_txpower();
void kiss_indicate_sf();
void kiss_indicate_cr();
void kiss_indicate_radiostate();
void kiss_indicate_radio_lock();
void kiss_indicate_btpin();
void kiss_indicate_lt_alock();
void kiss_indicate_wifi_ssid();
void kiss_indicate_wifi_psk();
void kiss_indicate_wifi_mode();

#ifdef USE_BT
void bt_conf_save(bool is_enabled);
#endif

// Frequency is 160MHz + CHAN * 250KHz
// Range: 160-173.5MHZ 00H-36H, 54 slots
uint32_t chan_to_freq(uint8_t chan) {
  return (uint32_t)160000000 + chan * (uint32_t)250000;
}

uint8_t freq_to_chan(uint32_t freq) {
  return (freq - (uint32_t)160000000)/(uint32_t)250000;
}

// Set default parameters
void check_modem_configuration() {
  ResponseStructContainer c;
  c = e32ttl.getConfiguration();
  if (c.status.code == E32_SUCCESS) {
    Configuration configuration = *(Configuration*) c.data;
    configuration.ADDL = 0xFF;
    configuration.ADDH = 0xFF;
    configuration.CHAN = 0x28;
    configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;

    configuration.OPTION.fec = FEC_1_ON;
    configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    configuration.OPTION.transmissionPower = POWER_30;

    configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.uartParity = MODE_00_8N1;
    #ifdef DEBUG_ENABLED
    printParameters(configuration);
    #endif
    ResponseStatus rs = e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    c.close();
    if (rs.code != E32_SUCCESS) {
      #ifdef DEBUG_ENABLED
      DebugSerial.println("!!! E32 configuration set FAILED");
      #endif
      e32_modem_errors++;
    }
  } else {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("!!! E32 configuration read FAILED");
    #endif
    e32_modem_errors++;
  }
}

uint8_t configure_modem(uint8_t chan, uint8_t rate, bool setRate) {
  uint8_t rc = 0;
  ResponseStructContainer c;
  c = e32ttl.getConfiguration();
  if (c.status.code == E32_SUCCESS) {
    Configuration configuration = *(Configuration*) c.data;
    if (chan > 0) {
      configuration.CHAN = chan;
    }
    if (setRate) {
      configuration.SPED.airDataRate = rate;
      configuration.SPED.uartBaudRate = UART_BPS_9600;
      configuration.SPED.uartParity = MODE_00_8N1;
    }
    ResponseStatus rs = e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    c.close();
    if (rs.code != E32_SUCCESS) {
      #ifdef DEBUG_ENABLED
      DebugSerial.println("!!! E32 configuration set FAILED");
      #endif
      e32_modem_errors++;
      modem_ok = false;
      return -1;
    }
    modem_ok = true;
    //#ifdef DEBUG_ENABLED
    //printParameters(configuration);
    //#endif
  } else {
    rc = -1;
    e32_modem_errors++;
    modem_ok = false;
    #ifdef DEBUG_ENABLED
    DebugSerial.println("!!! E32 configuration read FAILED");
    #endif
  }
  return rc;
}

ResponseStatus send_to_e32(const uint8_t* data, uint16_t len) {
  if (len == 0 || len > MTU) {
    return ResponseStatus{.code = ERR_E32_PACKET_TOO_BIG};
  }
  #ifdef USE_POWERSAVE
  e32ttl.setMode(MODE_1_WAKE_UP); // wakeup and send with big preamble
  #endif

  uint8_t prefixed[MTU + 4];           // 2 (length) + data + 2 (CRC)
  prefixed[0] = (uint8_t)(len >> 8);
  prefixed[1] = (uint8_t)len;
  memcpy(prefixed + 2, data, len);
  
  crc_t crc = crc_init();
  crc = crc_update(crc, data, len);
  crc = crc_finalize(crc);

  prefixed[2 + len]     = (uint8_t)(crc >> 8);
  prefixed[2 + len + 1] = (uint8_t)crc;
  len += 4;

  unsigned long tx_start = millis();

  ResponseStatus status;
  status.code = E32_SUCCESS;

  uint16_t sent = 0;
  while (sent < len) {
    uint8_t chunk_size = min((uint16_t)E32_MAX_SIZE_PACKET, (uint16_t)(len - sent));
    
    unsigned long start = millis();
    while (digitalRead(AUX_PIN) == LOW) {
      if (millis() - start > 6000) {
        #ifdef DEBUG_ENABLED
        DebugSerial.println("AUX timeout during TX");
        #endif
        status.code = ERR_E32_TIMEOUT;
        return status;
      }
      delay(5);
    }

    status = e32ttl.sendMessage((const void*)(prefixed + sent), chunk_size);
    if (status.code != E32_SUCCESS) {
      #ifdef DEBUG_ENABLED
      DebugSerial.print("E32 TX chunk error: ");
      DebugSerial.println(status.getResponseDescription());
      #endif
      break;
    } else {
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("Chunk sent: %u bytes\n", chunk_size);
      #endif
    }

    sent += chunk_size;
    delay(8);

    #ifdef USE_POWERSAVE
    if (sent < len) {
      e32ttl.setMode(MODE_0_NORMAL); // send other chunks in normal mode
    }
    #endif
  }

  update_airtime((millis() - tx_start) * 2); // yes it is * 2

  update_traffic_stats(0, len-4);

  #ifdef USE_POWERSAVE
  e32ttl.setMode(MODE_2_POWER_SAVING); // go to sleep again
  #endif
  return status;
}

void e32_process_rx() {
  while (e32ttl.available() > 0 && e32_rx_len < LOCAL_E32_RX_BUFFER_SIZE) {
    ResponseContainer rc = e32ttl.receiveMessage();
    if (rc.status.code != 1 || rc.data.length() == 0) continue;

    uint16_t chunk_len = rc.data.length();
    #ifdef DEBUG_ENABLED
    DebugSerial.printf("E32 RX chunk: %d bytes (total buffered: %d)\n", chunk_len, e32_rx_len);
    #endif

    if (e32_rx_len > 0 && (millis() - last_rx_time > 10000)) {
      #ifdef DEBUG_ENABLED
      DebugSerial.println("Buffer was too old, cleaned now.");
      #endif
      reset_rx_buffer();
    }
    last_rx_time = millis();

    uint16_t can_copy = min(chunk_len, (uint16_t)(LOCAL_E32_RX_BUFFER_SIZE - e32_rx_len));
    memcpy(e32_rx_buffer + e32_rx_len, rc.data.c_str(), can_copy);
    e32_rx_len += can_copy;

    bytes_since_last_valid += can_copy;
  }

  // Assemble RNS packets and check CRC
  while (e32_rx_len >= 4) {
    uint16_t expected_len = ((uint16_t)e32_rx_buffer[0] << 8) | e32_rx_buffer[1];

    if (expected_len == 0 || expected_len > MTU) {
      reset_rx_buffer();
      #ifdef DEBUG_ENABLED
      DebugSerial.println("!!! Bad expected length, drop buffer.");
      #endif
      break;
    } else if ((expected_len + 4) > e32_rx_len) {
      break; // waiting for next chunk
    }

    // CRC check
    uint16_t received_crc = ((uint16_t)e32_rx_buffer[2 + expected_len] << 8) | 
                            e32_rx_buffer[3 + expected_len];

    crc_t calculated_crc = crc_init();
    calculated_crc = crc_update(calculated_crc, e32_rx_buffer + 2, expected_len);
    calculated_crc = crc_finalize(calculated_crc);

    if (received_crc == calculated_crc) {
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("Valid RNS packet: %d bytes\n", expected_len);
      #endif

      send_kiss_packet(CMD_DATA, e32_rx_buffer + 2, expected_len);
      update_traffic_stats(expected_len, 0);

      uint16_t remaining = e32_rx_len - (4 + expected_len);
      if (remaining > 0) {
        memmove(e32_rx_buffer, e32_rx_buffer + 4 + expected_len, remaining);
      }
      e32_rx_len = remaining;
      
      bytes_since_last_valid = 0;
    } else {
      // CRC mismatch, drop packet
      uint16_t remaining = e32_rx_len - (4 + expected_len);
      if (remaining > 0) {
        memmove(e32_rx_buffer, e32_rx_buffer + 4 + expected_len, remaining);
      }
      e32_rx_len = remaining;
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("CRC mismatch, drop packet: %d bytes\n", expected_len);
      #endif
    }
  }

  if (bytes_since_last_valid > MTU + 10) {
    #ifdef DEBUG_ENABLED
    DebugSerial.printf("No valid packet for > %d bytes since last valid, resetting buffer\n", 
                       bytes_since_last_valid);
    #endif
    reset_rx_buffer();
  }
}

void check_storage_and_name() {
  if (!prefs.begin("rnode", true)) {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("Failed to open NVS! Formatting...");
    #endif
    prefs.begin("rnode", false);
    prefs.clear();
    prefs.end();
    delay(100);
    ESP.restart();
    return;
  }

  read_device_name();
  #ifdef DEBUG_ENABLED
  DebugSerial.printf("Device name: %s\n", device_name);
  #endif

  prefs.end();
}

void setup() {
  #ifdef USE_WIFI
  wifi_process_kiss_byte_fn_ptr = process_kiss_byte; // for wifi
  #endif

  #ifdef USE_BT
  bt_kiss_indicate_btpin_fn_ptr = kiss_indicate_btpin; // for bluetooth
  #endif

  Serial.begin(115200);

  #ifdef DEBUG_ENABLED
    #ifdef SOFTWARE_SERIAL_DEBUG
      DebugSerial.begin(19200);
    #else
      DebugSerial.begin(115200, SERIAL_8N1, DEBUG_RX, DEBUG_TX);
    #endif
  #endif
  delay(300);
  check_storage_and_name();

  pinMode(AUX_PIN, INPUT_PULLUP);
  // Startup all pins and UART for E32
  e32ttl.begin();

  #ifdef USE_DISPLAY
  display_welcome();
  #endif
  
  cfg_power_measure();
  //init_temperature_sensor();
  check_modem_configuration();
  #ifdef USE_POWERSAVE
  e32ttl.setMode(MODE_2_POWER_SAVING); // always in powersave, wake up for transmit only
  #endif

  #ifdef USE_WIFI
  wifi_init();
  #endif

  #ifdef USE_BT
  bt_init();
  #endif

  //current_temperature = temp_measure();
  //last_temp_time = millis();
  current_power_percents = voltage_to_percentage(read_battery_voltage());
  last_power_time = millis();

  #ifdef DEBUG_ENABLED
  DebugSerial.println("<<< RNode-E32 UART bridge ready >>>");
  #endif
 
  send_kiss_ready();  // Initial ready signal for Reticulum detection/init
}

void loop() {
  // Read and process KISS frames from Reticulum
  while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    process_kiss_byte(byte);
  }
  
  #ifdef USE_BT
  while (SerialBT.available()) {
    process_kiss_byte(SerialBT.read());
  }
  #endif

  // Read KISS from TCP
  #ifdef USE_WIFI
  handle_tcp_kiss();
  #endif

  e32_process_rx();

  if (tx_queue_count > 0 && can_transmit()) {
    uint8_t* data = tx_packet_queue[tx_queue_tail];
    uint16_t len = tx_packet_sizes[tx_queue_tail];
    tx_queue_pop(data, len);

    #ifdef DEBUG_ENABLED
    DebugSerial.printf("Sending queued packet (%d bytes)\n", len);
    #endif

    send_to_e32(data, len);
  }

  #ifdef USE_BT
  if (!console_active && bt_ready) update_bt();
  #endif

  #ifdef USE_WIFI
  if (wifi_active && WiFi.status() != WL_CONNECTED) {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("WiFi lost, reconnecting...");
    #endif
    WiFi.reconnect();
  }
  #endif

  #ifdef USE_DISPLAY
  if (millis() - last_display_update > 2000) {
    stats_for_display();
    update_display();
    last_display_update = millis();
  }
  #endif

  //if (millis() - last_temp_time > 360000) { // 6 min 6*60*1000
  //  current_temperature = temp_measure();
  //  last_temp_time = millis();
  //}
  if (millis() - last_power_time > 300000) { // 5 min
    current_power_percents = voltage_to_percentage(read_battery_voltage());
    last_power_time = millis();
  }
}

// KISS byte processor (frame parser with escaping)
void process_kiss_byte(uint8_t byte) {
  if (byte == FEND) {
    if (kiss_in_frame && kiss_rx_index > 0) {
      process_kiss_frame();
    }
    kiss_in_frame = true;
    kiss_rx_index = 0;
    kiss_escaped = false;
    return;
  }

  if (!kiss_in_frame) return;

  if (kiss_escaped) {
    if (byte == TFEND) byte = FEND;
    else if (byte == TFESC) byte = FESC;
    kiss_escaped = false;
  } else if (byte == FESC) {
    kiss_escaped = true;
    return;
  }

  if (kiss_rx_index < sizeof(kiss_rx_buffer)) {
    kiss_rx_buffer[kiss_rx_index++] = byte;
  }
}

// Process complete KISS frame
void process_kiss_frame() {
  if (kiss_rx_index == 0) return;

  uint8_t cmd = kiss_rx_buffer[0];
  uint8_t* data = kiss_rx_buffer + 1;
  uint16_t len = kiss_rx_index - 1;

#ifdef DEBUG_ENABLED
  if (cmd != CMD_DETECT && cmd != CMD_FW_VERSION && cmd != CMD_MCU && cmd != CMD_PLATFORM) {
    DebugSerial.print("KISS CMD 0x");
    DebugSerial.print(cmd, HEX);
    DebugSerial.print(" len=");
    DebugSerial.print(len);
    DebugSerial.print("  data: ");
    for (uint16_t i = 0; i < len && i < 8; i++) {
      DebugSerial.printf("%02X ", data[i]);
    }
    DebugSerial.println();
  }
#endif

  switch (cmd) {
    case CMD_DATA:
      if (len > 0) {
        if (can_transmit()) {
          send_to_e32(data, len);
        } else {
          if (!tx_queue_push(data, len)) {
            #ifdef DEBUG_ENABLED
            DebugSerial.println("TX queue full + duty cycle limit - packet dropped");
            #endif
          } else {
            #ifdef DEBUG_ENABLED
            DebugSerial.println("Packet queued due to duty cycle");
            #endif
          }
        }
      }
      break;

    case CMD_DETECT:
      if (len > 0 && data[0] == DETECT_REQ) kiss_indicate_detect();
      break;

    case CMD_FW_VERSION: kiss_indicate_fw_version(); break;
    case CMD_PLATFORM:   kiss_indicate_platform(); break;
    case CMD_MCU:        kiss_indicate_mcu(); break;

    case CMD_FREQUENCY:
      if (len == 4) {
        uint32_t freq;
        freq = ((uint32_t)data[0]<<24) | ((uint32_t)data[1]<<16) | ((uint32_t)data[2]<<8) | data[3];
        uint8_t chan = freq_to_chan(freq);
        #ifdef DEBUG_ENABLED
        DebugSerial.printf("  Trying to set frequency = %u Hz, chan: %u\n", freq, chan);
        #endif
        if (freq < 160000000 || freq > 173500000 || chan < 0x00 || chan > 0x36) {
          #ifdef DEBUG_ENABLED
          DebugSerial.println("Error! Frequency unsupported!");
          #endif
        } else {
          if (configure_modem(chan, 0, false) == 0) {
            cur_f = freq;
            #ifdef DEBUG_ENABLED
            DebugSerial.printf("SET frequency = %u Hz\n", cur_f);
            #endif
          }
        }
      }
      kiss_indicate_frequency();
      break;

    case CMD_BANDWIDTH:
      if (len == 4) {
        cur_bw = ((uint32_t)data[0]<<24) | ((uint32_t)data[1]<<16) | ((uint32_t)data[2]<<8) | data[3];
        #ifdef DEBUG_ENABLED
        DebugSerial.printf("SET bandwidth = %u Hz\n", cur_bw);
        #endif
      }
      kiss_indicate_bandwidth();
      break;

    case CMD_TXPOWER:
      if (len == 1) cur_tx = data[0];
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("SET txpower = %d dBm\n", cur_tx);
      #endif
      kiss_indicate_txpower();
      break;

    case CMD_SF:
      if (len == 1) cur_sf = data[0];
      AIR_DATA_RATE newRate;
      switch(cur_sf) {
        case 7: case 8: newRate = AIR_DATA_RATE_100_96; break;
        case 9: newRate = AIR_DATA_RATE_011_48; break;
        case 10: newRate = AIR_DATA_RATE_010_24; break;
        case 11: newRate = AIR_DATA_RATE_001_12; break;
        case 12: newRate = AIR_DATA_RATE_000_03; break;
        default: newRate = AIR_DATA_RATE_010_24;
      }
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("Trying to set SF = %d\n", cur_sf);
      #endif
      if (configure_modem(0, newRate, true) == 0) {
        #ifdef DEBUG_ENABLED
        switch(cur_sf) {
          case 7: case 8: DebugSerial.println("  New air data rate is 9.6K"); break;
          case 9: DebugSerial.println("  New air data rate is 4.8K"); break;
          case 10: DebugSerial.println("  New air data rate is 2.4K (default)"); break;
          case 11: DebugSerial.println("  New air data rate is 1.2K"); break;
          case 12: DebugSerial.println("  New air data rate is 0.3K"); break;
          default: DebugSerial.println("  New air data rate is 2.4K (default)");
        }
        #endif
      }
      kiss_indicate_sf();
      break;

    case CMD_CR:
      if (len == 1) cur_cr = data[0];
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("SET CR = %d\n", cur_cr);
      #endif
      kiss_indicate_cr();
      break;

    case CMD_RADIO_STATE:
      if (len == 1) cur_radio_state = data[0];
      #ifdef DEBUG_ENABLED
      DebugSerial.printf("SET radio_state = %d\n", cur_radio_state);
      #endif
      if (cur_radio_state == RADIO_STATE_OFF) {
        reset_traffic_stats();
        reset_rx_buffer();
        reset_tx_queue();
      }
      kiss_indicate_radiostate();
      break;

    case CMD_RADIO_LOCK:
      kiss_indicate_radio_lock();
      break;

    case CMD_LT_ALOCK: {
      if (len == 2) {
        uint16_t at = ((uint16_t)data[0]<<8) | (uint16_t)data[1];
        if (at == 0) {
          lt_airtime_limit = 0;
        } else {
          lt_airtime_limit = (float)at / (100.0*100.0); // because interface is sending percents*100
          if (lt_airtime_limit > 1.0) lt_airtime_limit = 1.0;
        }
        #ifdef DEBUG_ENABLED
        DebugSerial.printf("Long-term Air Time Limit set to %f%%\n", lt_airtime_limit*100);
        #endif
      }
      kiss_indicate_lt_alock();
    } break;

    case CMD_BT_CTRL: {
      #ifdef USE_BT
      uint8_t sb = data[0];
      if (sb == 0x00) {
        bt_stop();
        bt_conf_save(false);
      } else if (sb == 0x01) {
        bt_start();
        bt_conf_save(true);
      } else if (sb == 0x02) {
        if (bt_state == BT_STATE_OFF) {
          bt_start();
          bt_conf_save(true);
        }
        if (bt_state != BT_STATE_CONNECTED) {
          bt_enable_pairing();
        }
      }
      #endif
    } break;

    case CMD_BT_UNPAIR:
      #ifdef USE_BT
        if (data[0] == 0x01) { bt_debond_all(); }
      #endif
      break;

    case CMD_WIFI_SSID:
      #ifdef USE_WIFI
      if (len > 0 && len < WIFI_SSID_MAX_LEN) {
        memset(wifi_ssid, 0, WIFI_SSID_MAX_LEN);
        memcpy(wifi_ssid, data, len);
        wifi_save_ssid();
        #ifdef DEBUG_ENABLED
        DebugSerial.printf("SET WiFi SSID: %s\n", wifi_ssid);
        #endif
      }
      #endif
      kiss_indicate_wifi_ssid();
      break;

    case CMD_WIFI_PSK:
      #ifdef USE_WIFI
      if (len > 0 && len < WIFI_PASS_MAX_LEN) {
        memset(wifi_psk, 0, WIFI_PASS_MAX_LEN);
        memcpy(wifi_psk, data, len);
        wifi_save_psk();
        #ifdef DEBUG_ENABLED
        DebugSerial.println("SET WiFi PSK (hidden)");
        #endif
      }
      #endif
      kiss_indicate_wifi_psk();
      break;

    case CMD_WIFI_MODE:
      #ifdef USE_WIFI
      if (len == 1) {
        wifi_enabled = (data[0] != 0);
        wifi_save_mode();
        #ifdef DEBUG_ENABLED
        DebugSerial.printf("SET WiFi mode: %s\n", wifi_enabled ? "ENABLED" : "DISABLED");
        #endif
      }
      #endif
      kiss_indicate_wifi_mode();
      break;  

    default:
      //DebugSerial.println("  Unknown command");
      send_kiss_ready();
      break;
  }
}

// Send KISS frame (with proper escaping)
void send_kiss_packet(uint8_t cmd, const uint8_t* data, uint16_t len) {
  #ifdef USE_BT
    if (bt_state != BT_STATE_CONNECTED) {
			#ifdef USE_WIFI
				if (wifi_connected()) {
           send_kiss_packet_tcp(cmd, data, len);
        } else {
           send_kiss_packet_serial(cmd, data, len);
        }
			#else
				send_kiss_packet_serial(cmd, data, len);
			#endif
		} else {
			SerialBT.write(FEND);
      SerialBT.write(cmd);

      if (data != nullptr && len > 0) {
        for (uint16_t i = 0; i < len; i++) {
          uint8_t b = data[i];
          if (b == FEND) {
            SerialBT.write(FESC);
            SerialBT.write(TFEND);
          } else if (b == FESC) {
            SerialBT.write(FESC);
            SerialBT.write(TFESC);
          } else {
            SerialBT.write(b);
          }
        }
      }

      SerialBT.write(FEND);
    }
  #else
    #ifdef USE_WIFI
      if (wifi_connected()) {
        send_kiss_packet_tcp(cmd, data, len);
      } else {
        send_kiss_packet_serial(cmd, data, len);
      }
    #else
      send_kiss_packet_serial(cmd, data, len);
    #endif
  #endif
}

void send_kiss_packet_serial(uint8_t cmd, const uint8_t* data, uint16_t len) {
  Serial.write(FEND);
  Serial.write(cmd);

  if (data != nullptr && len > 0) {
    for (uint16_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      if (b == FEND) {
        Serial.write(FESC);
        Serial.write(TFEND);
      } else if (b == FESC) {
        Serial.write(FESC);
        Serial.write(TFESC);
      } else {
        Serial.write(b);
      }
    }
  }

  Serial.write(FEND);
}

void send_kiss_ready() {
  send_kiss_packet(CMD_READY, nullptr, 0);
}

void kiss_indicate_detect()      { uint8_t r[] = {DETECT_RESP}; send_kiss_packet(CMD_DETECT, r, 1); send_kiss_ready(); }
void kiss_indicate_fw_version()  { uint8_t r[] = {FW_MAJ, FW_MIN}; send_kiss_packet(CMD_FW_VERSION, r, 2); send_kiss_ready(); }
void kiss_indicate_platform()    { uint8_t r[] = {PLATFORM_ESP32}; send_kiss_packet(CMD_PLATFORM, r, 1); send_kiss_ready(); }
void kiss_indicate_mcu()         { uint8_t r[] = {0x00}; send_kiss_packet(CMD_MCU, r, 1); send_kiss_ready(); }

void kiss_indicate_frequency()   { 
  uint8_t d[4] = {(uint8_t)(cur_f>>24),(uint8_t)(cur_f>>16),(uint8_t)(cur_f>>8),(uint8_t)cur_f};
  send_kiss_packet(CMD_FREQUENCY, d, 4);
  send_kiss_ready(); 
}
void kiss_indicate_bandwidth()   { 
  uint8_t d[4] = {(uint8_t)(cur_bw>>24),(uint8_t)(cur_bw>>16),(uint8_t)(cur_bw>>8),(uint8_t)cur_bw};
  send_kiss_packet(CMD_BANDWIDTH, d, 4); 
  send_kiss_ready(); 
}

void kiss_indicate_txpower()     { send_kiss_packet(CMD_TXPOWER, &cur_tx, 1); send_kiss_ready(); }
void kiss_indicate_sf()          { send_kiss_packet(CMD_SF, &cur_sf, 1); send_kiss_ready(); }
void kiss_indicate_cr()          { send_kiss_packet(CMD_CR, &cur_cr, 1); send_kiss_ready(); }
void kiss_indicate_radiostate()  { send_kiss_packet(CMD_RADIO_STATE, &cur_radio_state, 1); send_kiss_ready(); }
void kiss_indicate_radio_lock() { uint8_t l = 0; send_kiss_packet(CMD_RADIO_LOCK, &l, 1); send_kiss_ready(); }

void kiss_indicate_lt_alock() {
  uint16_t at = (uint16_t)(lt_airtime_limit*100*100);
  uint8_t d[2] = {(uint8_t)(at>>8),(uint8_t)at};
  send_kiss_packet(CMD_LT_ALOCK, d, 2);
  send_kiss_ready(); 
}

void kiss_indicate_btpin() {
	#ifdef USE_BT
    uint8_t d[4] = {(uint8_t)(bt_ssp_pin>>24),(uint8_t)(bt_ssp_pin>>16),(uint8_t)(bt_ssp_pin>>8),(uint8_t)bt_ssp_pin};
    send_kiss_packet(CMD_BT_PIN, d, 4);
	#endif
}

void kiss_indicate_wifi_ssid() {
  send_kiss_packet(CMD_WIFI_SSID, (const uint8_t*)wifi_ssid, strlen(wifi_ssid));
  send_kiss_ready();
}

void kiss_indicate_wifi_psk() {
  uint8_t resp = (strlen(wifi_psk) > 0) ? 1 : 0;
  send_kiss_packet(CMD_WIFI_PSK, &resp, 1);
  send_kiss_ready();
}

void kiss_indicate_wifi_mode() {
  uint8_t mode = wifi_enabled ? 1 : 0;
  send_kiss_packet(CMD_WIFI_MODE, &mode, 1);
  send_kiss_ready();
}

void printParameters(struct Configuration configuration) {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("----------------------------------------");
	DebugSerial.print(F("HEAD : "));  DebugSerial.print(configuration.HEAD, BIN);DebugSerial.print(" ");DebugSerial.print(configuration.HEAD, DEC);DebugSerial.print(" ");DebugSerial.println(configuration.HEAD, HEX);
	DebugSerial.print(F("AddH : "));  DebugSerial.println(configuration.ADDH, DEC);
	DebugSerial.print(F("AddL : "));  DebugSerial.println(configuration.ADDL, DEC);
	DebugSerial.print(F("Chan : "));  DebugSerial.print(configuration.CHAN, DEC); DebugSerial.print(" -> "); DebugSerial.println(chan_to_freq(configuration.CHAN), DEC);
	DebugSerial.print(F("SpeedParityBit     : "));  DebugSerial.print(configuration.SPED.uartParity, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.SPED.getUARTParityDescription());
	DebugSerial.print(F("SpeedUARTBaudRate  : "));  DebugSerial.print(configuration.SPED.uartBaudRate, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.SPED.getUARTBaudRate());
	DebugSerial.print(F("SpeedAirDataRate   : "));  DebugSerial.print(configuration.SPED.airDataRate, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.SPED.getAirDataRate());
	DebugSerial.print(F("OptionTrans        : "));  DebugSerial.print(configuration.OPTION.fixedTransmission, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.OPTION.getFixedTransmissionDescription());
	DebugSerial.print(F("OptionPullup       : "));  DebugSerial.print(configuration.OPTION.ioDriveMode, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.OPTION.getIODroveModeDescription());
	DebugSerial.print(F("OptionWakeup       : "));  DebugSerial.print(configuration.OPTION.wirelessWakeupTime, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	DebugSerial.print(F("OptionFEC          : "));  DebugSerial.print(configuration.OPTION.fec, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.OPTION.getFECDescription());
	DebugSerial.print(F("OptionPower        : "));  DebugSerial.print(configuration.OPTION.transmissionPower, BIN);DebugSerial.print(" -> "); DebugSerial.println(configuration.OPTION.getTransmissionPowerDescription());
	DebugSerial.println("----------------------------------------");
  #endif
}