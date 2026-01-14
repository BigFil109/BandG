#include <Arduino.h>
#include <AltSoftSerial.h>
#include "fastnet.h"

// ================= CONFIG =================

#define FASTNET_BAUD 28800
#define RS485_EN 13
#define NMEA_RX 8
#define LIGHT_BUTTON 7

#define DEPTH_LIMIT_MM 1500

// ================= OBJECTS =================

Fastnet fastnet(0x2F);
AltSoftSerial nmea;

// ================= RS485 =================

inline void rs485_rx() {
  digitalWrite(RS485_EN, LOW);
}
inline void rs485_tx() {
  digitalWrite(RS485_EN, HIGH);
}

// ================= DATA =================

uint16_t sog = 0;  // 0.1 kt
uint16_t depth_mm = 0;
int16_t brg_mark = 0;
char brg_mark_name[9] ="       ";
bool brg_valid = true;

int32_t countdown_sec = -1;

// ================= DISPLAY MODE =================

enum DisplayMode {
  MODE_NONE,
  MODE_TIMER,
  MODE_SPEED,
  MODE_BRG,
  MODE_DEPTH
};

DisplayMode mode = MODE_SPEED;
DisplayMode prev_mode = MODE_NONE;

// ================= LIGHT =================

uint8_t light_level = 2;
bool button_last = HIGH;

// ================= TIMERS =================

uint32_t t_nmea = 0;
uint32_t t_data = 0;
uint32_t t_page = 0;
uint32_t t_brg = 0;
uint32_t t_depth_ok = 0;

// ================= RX =================

uint8_t rx_buf[64];
uint8_t rx_len = 0;
bool rx_active = false;
uint32_t last_rx_time = 0;

bool display_registered = false;

// ================= FASTNET RX =================

void handle_fastnet(uint8_t *b, uint8_t l) {
  if (l < 5) return;
  if (b[3] == CM_HELLO && !display_registered) {
    fastnet.register_device(b[1]);
    fastnet.set_device(b[1]);
    display_registered = true;
  }
}

void fastnet_rx() {
  rs485_rx();
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    uint32_t now = millis();

    if (rx_active && now - last_rx_time > 20) {
      handle_fastnet(rx_buf, rx_len);
      rx_len = 0;
      rx_active = false;
    }
    last_rx_time = now;

    if (!rx_active) {
      if (c != 0xFF) continue;
      rx_active = true;
      rx_len = 0;
    }
    if (rx_len < sizeof(rx_buf)) rx_buf[rx_len++] = c;
  }
}

// ================= NMEA =================

// out must be char[9]  (8 chars + '\0')
void get_last6_with_prefix(const char *src, char out[9]) {
  size_t len = strlen(src);

  // Prefix
  out[0] = 'M';
  out[1] = ' ';

  if (len >= 6) {
    memcpy(out + 2, src + len - 6, 6);
  } else {
    // Pad left with spaces if shorter than 6
    memset(out + 2, ' ', 6);
    memcpy(out + 2 + (6 - len), src, len);
  }

  out[8] = '\0';
}


char nmea_line[82];
uint8_t nmea_pos = 0;

void parse_nmea(char *s) {
  if (strstr(s, "RMC")) {
    char *p = strchr(s, ',');
    for (uint8_t i = 0; i < 6 && p; i++) p = strchr(p + 1, ',');
    if (p) sog = atof(p + 1) * 10;
  }

  if (strstr(s, "DPT")) {
    depth_mm = atof(strchr(s, ',') + 1) * 1000;
  }

  if (strstr(s, "BWC")) {
    float bearing_true = 0.0;

    if (strstr(s, "BWC")) {

      char buf[96];
      strncpy(buf, s, sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';

      char *token;
      uint8_t field = 0;

      float bearing_true = 0.0;
      char mark_id_raw[16] = { 0 };   // holds "004*29"
      char mark_id_clean[8] = { 0 };  // holds "004"

      token = strtok(buf, ",");

      while (token) {

        if (field == 6) {  // 051.9 (true bearing)
          bearing_true = atof(token);

        } else if (field == 12) {  // 004*29
          strncpy(mark_id_raw, token, sizeof(mark_id_raw) - 1);
        }

        token = strtok(NULL, ",");
        field++;
      }

      // Strip checksum (*xx)
      char *star = strchr(mark_id_raw, '*');
      if (star) *star = '\0';

      strncpy(mark_id_clean, mark_id_raw, sizeof(mark_id_clean) - 1);

      // Build 6-char page label
      get_last6_with_prefix(mark_id_clean, brg_mark_name);

      // Store values
      brg_mark = bearing_true *10;
      brg_valid = true;
    }
  }

  //$IIZDA,00:00:4,01,01,2024*6F
  if (strstr(s, "ZDA")) {
    char *timeField = strchr(s, ',');  // points to ",00:00:04"
    if (!timeField) return;

    timeField++;  // skip comma → "00:00:04"

    int hh = 0, mm = 0, ss = 0;
    sscanf(timeField, "%d:%d:%d", &hh, &mm, &ss);

    countdown_sec = ss;  // <-- seconds only
  }
}

void nmea_rx() {
  while (nmea.available()) {

    char c = nmea.read();
    if (c == '\n') {
      nmea_line[nmea_pos] = 0;
      parse_nmea(nmea_line);
      nmea_pos = 0;
    } else if (nmea_pos < sizeof(nmea_line) - 1) {
      nmea_line[nmea_pos++] = c;
    }
  }
}

// ================= DISPLAY LOGIC =================
uint16_t c_page = 0;
void update_mode() {


  if (millis() - c_page > 7000) {
    if (mode == MODE_BRG) {
      mode = MODE_DEPTH;
    } else if (mode == MODE_DEPTH) {
      mode = MODE_TIMER;
    } else if (mode == MODE_TIMER) {
      mode = MODE_SPEED;
    } else if (mode == MODE_SPEED) {
      mode = MODE_BRG;
    }
    c_page = millis();
  }


  /* if (depth_mm < DEPTH_LIMIT_MM) {
    mode = MODE_DEPTH;
    t_depth_ok = millis();
    fastnet.backlight(3);
    return;
  }

  if (mode == MODE_DEPTH && millis() - t_depth_ok < 10000) return;

  if (countdown_sec >= 0) {
    mode = MODE_TIMER;
    return;
  }

  if (brg_valid && millis() - t_brg > 20000) {
    mode = MODE_BRG;
    t_brg = millis();
    return;
  }

  if (mode == MODE_BRG && millis() - t_brg > 3000) {
    mode = MODE_SPEED;
  }*/
}

// ================= PAGE =================

void set_page() {
  if (prev_mode == mode) {
    return;
  }
  rs485_tx();

  if (mode == MODE_TIMER)
    fastnet.config_page(CH_TIMER, WIND_CPU, "TIMER   ", "  ");
  else if (mode == MODE_BRG)
    fastnet.config_page(CH_TRUE_WA, WIND_CPU, brg_mark_name, "%T");
  else if (mode == MODE_DEPTH)
    fastnet.config_page(CH_DEPTH_M, DEPTH_CPU, "DEPTH   ", " M");
  else
    fastnet.config_page(CH_BOAT_SPD_KT, DEPTH_CPU, "SOG     ", "KT");


  fastnet.store_page();
  delay(10);
  fastnet.backlight(4);

  Serial1.flush();
  rs485_rx();
  prev_mode = mode;
}

// ================= DATA =================

void send_data() {
  rs485_tx();
  fastnet.boat_speed(sog);
  delay(10);
  fastnet.depth(depth_mm / 10);
  delay(10);
  fastnet.timer(countdown_sec >= 0 ? countdown_sec : 0);
  delay(10);
  fastnet.true_wind(brg_mark, 0);
  delay(10);

  Serial1.flush();
  rs485_rx();
}

// ================= SETUP =================

void setup() {
  pinMode(RS485_EN, OUTPUT);
  pinMode(LIGHT_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Serial1.begin(FASTNET_BAUD, SERIAL_8O2);
  nmea.begin(4800);

  rs485_rx();
}

// ================= LOOP =================

void loop() {

  fastnet_rx();
  nmea_rx();
  update_mode();

  if (millis() - t_page > 4000) {
    set_page();
    t_page = millis();
  }

  if (millis() - t_data > 200) {
    send_data();
    t_data = millis();
  }

  bool b = digitalRead(LIGHT_BUTTON);
  if (b == LOW && button_last == HIGH) {
    light_level++;
    if (light_level > 3) light_level = 1;
    rs485_tx();
    fastnet.backlight(light_level);
    Serial1.flush();
    rs485_rx();
  }
  button_last = b;
}
