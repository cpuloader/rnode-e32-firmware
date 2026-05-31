#ifndef __RN_WIFI
#define __RN_WIFI

#include <WiFi.h>
#include "config.h"

WiFiServer kissServer(TCP_PORT);
WiFiClient kissClient;

bool wifi_connected();
bool wifi_save_ssid();
bool wifi_save_psk();
bool wifi_save_mode();

bool wifi_connected() {
  if (!wifi_active) return false;
  if (!kissClient || !kissClient.connected()) return false;
  return true;
}

void wifi_init() {
  #ifdef DEBUG_ENABLED
  DebugSerial.println("Checking stored WIFI configuration...");
  #endif

  prefs.begin("rnode", true); // read only
  uint8_t wf_flag  = prefs.getUChar("wf_flag", 0);
  String ssid_str = prefs.getString("wifi_ssid", "no");
  String pass_str = prefs.getString("wifi_pass", "no");
  prefs.end();

  wifi_enabled = (wf_flag == 1) ? true : false;
  if (!wifi_enabled) {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("WIFI disabled");
    #endif
    return;
  }
  ssid_str.toCharArray(wifi_ssid, WIFI_SSID_MAX_LEN);
  pass_str.toCharArray(wifi_psk, WIFI_PASS_MAX_LEN);
  if (ssid_str.length() < 8 || pass_str.length() < 8) {
    wifi_enabled = false;
    #ifdef DEBUG_ENABLED
    DebugSerial.printf("!!! No WIFI data: %s %s\n", wifi_ssid, wifi_psk);
    #endif
    return;
  }// else {
  //  #ifdef DEBUG_ENABLED
  //  DebugSerial.printf("WIFI data: %s %s\n", wifi_ssid, wifi_psk);
  //  #endif
  //}
  #ifdef DEBUG_ENABLED
  DebugSerial.println("Connecting to WiFi...");
  #endif
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifi_ssid, wifi_psk);

  unsigned long timeout = millis() + 20000;
  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
    delay(500);
    //DebugSerial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("\nWiFi connected!");
    DebugSerial.print("IP address: ");
    DebugSerial.println(WiFi.localIP());
    DebugSerial.print("TCP KISS port: ");
    DebugSerial.println(TCP_PORT);
    #endif

    kissServer.begin();
    kissServer.setNoDelay(true);
    delay(100);
    wifi_active = true;
  } else {
    #ifdef DEBUG_ENABLED
    DebugSerial.println("\nWiFi connection FAILED!");
    #endif
  }
}

void(*wifi_process_kiss_byte_fn_ptr)(uint8_t);

void handle_tcp_kiss() {
  if (!wifi_active) return;

  if (kissServer.hasClient()) {
    if (kissClient && kissClient.connected()) {
      kissClient.stop();
    }
    kissClient = kissServer.accept();
    if (kissClient) {
      kissClient.setNoDelay(true);
      #ifdef DEBUG_ENABLED
      DebugSerial.println("New TCP KISS client connected from " + kissClient.remoteIP().toString());
      #endif
    }
  }

  if (kissClient && kissClient.connected()) {
    while (kissClient.available()) {
      uint8_t byte = kissClient.read();
      wifi_process_kiss_byte_fn_ptr(byte);
    }
  } else if (kissClient) {
    kissClient.stop();
  }
}


void send_kiss_packet_tcp(uint8_t cmd, const uint8_t* data, uint16_t len) {
  if (!kissClient || !kissClient.connected()) return;

  kissClient.write(FEND);
  kissClient.write(cmd);

  if (data != nullptr && len > 0) {
    for (uint16_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      if (b == FEND) { kissClient.write(FESC); kissClient.write(TFEND); }
      else if (b == FESC) { kissClient.write(FESC); kissClient.write(TFESC); }
      else kissClient.write(b);
    }
  }
  kissClient.write(FEND);
}

bool wifi_save_ssid() {
  prefs.begin("rnode", false);
  prefs.putString("wifi_ssid", wifi_ssid);
  prefs.end();
}

bool wifi_save_psk() {
  prefs.begin("rnode", false);
  prefs.putString("wifi_pass", wifi_psk);
  prefs.end();
}

bool wifi_save_mode() {
  prefs.begin("rnode", false);
  prefs.putUChar("wf_flag", wifi_enabled ? 1 : 0);
  prefs.end();
}

#endif