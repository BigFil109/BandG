#include <Arduino.h>
#include "fastnet.h"

#define FASTNET_BAUD 28800
#define RS485_EN 13  // DE + RE tied

Fastnet fastnet(0x2F);

// ---------------- RX FRAME STATE ----------------

uint8_t rx_buf[64];
uint8_t rx_len = 0;
bool rx_active = false;
uint32_t last_rx_time = 0;

// ---------------- DISPLAY STATE ----------------

bool display_registered = false;
uint8_t display_id = 0;

// ---------------- PAGE STATE ----------------

enum PageState {
  PAGE_IDLE,
  PAGE_CHANGE_SENT
};

//config state
bool runSetup = true;

PageState page_state = PAGE_IDLE;
uint32_t page_change_time = 0;

const uint32_t PAGE_CONFIRM_TIMEOUT_MS = 250;

// ---------------- TIMERS ----------------

uint32_t who_timer = 0;
uint32_t page_timer = 0;
uint32_t data_timer = 0;
uint32_t light_timer = 0;

const uint32_t WHO_INTERVAL_MS = 2000;
const uint32_t PAGE_INTERVAL_MS = 5000;
const uint32_t DATA_INTERVAL_MS = 200;
const uint32_t LIGHT_INTERVAL_MS = 5000;

// ---------------- RS485 ----------------

inline void rs485_rx() {
  digitalWrite(RS485_EN, LOW);
}
inline void rs485_tx() {
  digitalWrite(RS485_EN, HIGH);
}

// ---------------- DEBUG ----------------

void dump_frame(uint8_t *buf, uint8_t len) {
  Serial.print("RX FRAME [");
  Serial.print(len);
  Serial.print("] ");
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// ---------------- RX HANDLER ----------------

void handle_frame(uint8_t *buf, uint8_t len) {
  if (len < 5) return;

  dump_frame(buf, len);

  uint8_t from = buf[1];
  uint8_t cmd = buf[3];

  switch (cmd) {

    case CM_HELLO:
      Serial.print("HELLO from display 0x");
      Serial.println(from, HEX);

      if (!display_registered) {
        display_registered = true;
        display_id = from;
        fastnet.register_device(from);
        fastnet.set_device(from);
        Serial.println("Display registered");
      }
      break;

    case CM_CURRENT_PAGE:
      {
        uint8_t page = buf[5];
        fastnet.set_page(page);

        Serial.print("PAGE CONFIRMED → ");
        Serial.println(page);

        page_state = PAGE_IDLE;  // 🔑 unlock TX
        break;
      }

    default:
      break;
  }
}

// ---------------- RX POLL ----------------

void fastnet_rx_poll() {
  rs485_rx();

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    uint32_t now = millis();

    if (rx_active && (now - last_rx_time > 20)) {
      handle_frame(rx_buf, rx_len);
      rx_len = 0;
      rx_active = false;
    }
    last_rx_time = now;

    if (!rx_active) {
      if (b != 0xFF) continue;
      rx_active = true;
      rx_len = 0;
    }

    if (rx_len < sizeof(rx_buf))
      rx_buf[rx_len++] = b;
  }
}

// ---------------- DATA ----------------

// Simulated data
uint16_t boat_spd = 62;  // 6.2 kts
uint16_t depth = 134;    // 13.4 m
uint16_t vmg = 28;       // 2.8 kts
int16_t awa = 32;        // deg
uint16_t aws = 145;      // 14.5 kts
int16_t twa = -179;      // deg
uint16_t tws = 100;      // 17.8 kts


void send_broadcast_data() {
  static uint16_t t = 0;
  fastnet.boat_speed(boat_spd);
  boat_spd += 111;
  if (boat_spd > 50000) { boat_spd = 0; }

  vmg += 111;
  if (vmg > 50000) { vmg = 0; }

  twa += 1;
  if (twa > 179) { twa = -180; }

  tws += 111;
  if (tws > 50000) { tws = 0; }

  fastnet.depth(depth);
  delay(10);
  fastnet.vmg(vmg);
  delay(10);
  fastnet.app_wind(awa, aws);
  delay(10);
  fastnet.true_wind(twa, tws);
  delay(10);
  fastnet.heading(330);
  delay(10);

  // Optional timer (some pages expect this)
  fastnet.timer((t++ / 10) % 600);
}


void config() {
  fastnet.config_page(
    CH_BOAT_SPD_KT,
    0x2F,
    "PH B SPD",
    "KT");
  fastnet.store_page();
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  Serial.println("\nFastnet Display Controller");

  pinMode(RS485_EN, OUTPUT);
  rs485_tx();

  Serial1.begin(FASTNET_BAUD, SERIAL_8O2);
  delay(100);
  //fix stuck page issue
  fastnet.register_device(0x42);
  fastnet.set_device(0x42);
  fastnet.change_page(0);
  delay(100);
  fastnet.store_page();
  Serial1.flush();
  rs485_rx();
  delay(100);
}

// ---------------- LOOP ----------------

void loop() {
  fastnet_rx_poll();

  // ---- WHO until display found ----
  if (!display_registered && millis() - who_timer > WHO_INTERVAL_MS) {
    Serial.println("TX: WHO");
    rs485_tx();
    fastnet.who();
    Serial1.flush();
    rs485_rx();
    who_timer = millis();
  }

  /*if (millis() - light_timer > LIGHT_INTERVAL_MS) {
    rs485_tx();
    fastnet.backlight(4);
    Serial1.flush();
    rs485_rx();
    light_timer = millis();
  }*/

  if (display_registered && runSetup) {
    Serial.println("Running page setup!!");
    delay(5000);
    rs485_tx();
    config();
    Serial1.flush();
    rs485_rx();

    runSetup = false;
  }



  // ---- Page rotation ----
  /* if (display_registered && page_state == PAGE_IDLE && millis() - page_timer > PAGE_INTERVAL_MS) {

    //   Serial.println("TX: Change page (NEXT)");
    rs485_tx();
    fastnet.change_page(0);
    page_state = PAGE_CHANGE_SENT;
    // test config reply
    delay(100);
    fastnet.store_page();
    delay(100);
    fastnet.store_page();
    delay(100);
    fastnet.store_page();
    Serial1.flush();
    rs485_rx();

    page_state = PAGE_IDLE;
    page_change_time = millis();
    page_timer = millis();
  }


  // ---- Data TX (blocked during page change) ----
  if (display_registered && page_state == PAGE_IDLE && millis() - data_timer > DATA_INTERVAL_MS) {

    rs485_tx();
    send_broadcast_data();
    Serial1.flush();
    rs485_rx();
    data_timer = millis();
  }*/
}
