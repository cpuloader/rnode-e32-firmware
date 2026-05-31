#ifndef __RN_DISPLAY
#define __RN_DISPLAY

#include <U8g2lib.h>

#include "config.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, DISPLAY_SCL, DISPLAY_SDA);
bool display_enabled = true;

unsigned long last_display_update = 0;

uint32_t rx_display_stats = 0;
uint32_t tx_display_stats = 0;

void display_welcome();
void update_display();
void draw_ble_pin();


void display_welcome() {
  u8g2.begin();
  u8g2.setContrast(180);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "RNode-E32 starting...");
  u8g2.sendBuffer();
}

void update_display() {
  if (bt_state == BT_STATE_PAIRING) {
    draw_ble_pin();
    return;
  }

  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, device_name);

  u8g2.setCursor(100, 12);
  u8g2.print(current_power_percents);
  u8g2.print("%");

  u8g2.setCursor(0, 25);
  u8g2.print(cur_f / 1000000);
  u8g2.print(".");
  u8g2.print((cur_f % 1000000) / 1000);
  u8g2.print(" MHz, SF: ");
  u8g2.print(cur_sf); 

  if (cur_radio_state == RADIO_STATE_ON) {
    // RX
    u8g2.drawStr(0, 43, "RX");
    u8g2.setCursor(25, 43);
    if (rx_display_stats >= 1024) {
      float k = rx_display_stats / 1024.0;
      u8g2.print(k, 2);
      u8g2.print(" K");
    } else {
      u8g2.print(rx_display_stats);
      u8g2.print(" B");
    }

    // TX
    u8g2.drawStr(65, 43, "TX");
    u8g2.setCursor(85, 43);
    if (tx_display_stats >= 1024) {
      float k = tx_display_stats / 1024.0;
      u8g2.print(k, 2);
      u8g2.print(" K");
    } else {
      u8g2.print(tx_display_stats);
      u8g2.print(" B");
    }
  } else {
    u8g2.drawStr(0, 43, "Disconnected");
  }
  #ifdef USE_BT
  u8g2.drawStr(0, 63, bt_enabled ? "BT ON" : "BT OFF");
  #endif
  #ifdef USE_WIFI
  u8g2.drawStr(45, 63, wifi_enabled ? "WIFI ON" : "WIFI OFF");
  #endif

  //u8g2.setCursor(90, 63);
  //u8g2.print((int)current_temperature);
  //u8g2.print("C");

  u8g2.sendBuffer();
}

void draw_ble_pin() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "Pairing pin");
  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.setCursor(0, 25);
  u8g2.print(bt_ssp_pin);
  u8g2.sendBuffer();
}

#endif