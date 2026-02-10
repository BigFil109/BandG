#include <Arduino.h>
#include <avr/wdt.h>
#include "fastnet.h"

// ================= MARINE DISPLAY INTERFACE =================
// Bridges NMEA 0183 data to Fastnet displays:
//   - Fastnet 2020 display (Serial1, RS485)
//   - Fastnet Quad display (Serial3)
// Receives NMEA sentences on Serial2 and converts them to Fastnet protocol

// ================= CONFIG =================

#define FASTNET_BAUD 28800
#define RS485_EN 13
#define NMEA_RX 8
#define DEPTH_LIMIT_MM 1500
#define LIGHT_LEVEL 3

// ================= OBJECTS =================

Fastnet fastnet(0x2F);

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
uint16_t brg_mark = 0;
char brg_mark_name[9] = "M   NONE ";
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
DisplayMode prev_mode = MODE_SPEED;

// ================= TIMERS =================

uint32_t t_nmea = 0;
uint32_t t_data = 0;
uint32_t t_page = 0;
uint32_t t_brg = 0;
uint32_t t_light = 0;
uint32_t t_quad_header = 0;


// ================= RX =================

uint8_t rx_buf[64];
uint8_t rx_len = 0;
bool rx_active = false;
uint32_t last_rx_time = 0;

bool display_registered = false;


// ============= Network ========
const uint8_t FASTNET_FMT_BYTES[] = {
  //  id:  0  1  2  3
  0, 4, 4, 5,
  //      4  5  6  7
  6, 6, 0, 6,
  //      8  9 10 11
  4, 0, 0, 6,
  //     12 13 14 15
  0, 0, 0, 0
};

#define FASTNET_CH_AWS 0x50
#define FASTNET_CH_AWA 0x51
#define FASTNET_CH_VOLTAGE 0x8D  // no nema feed atm
#define FASTNET_CH_DEPTH 0xC4
#define FASTNET_CH_SPEED_KNOTS 0x41
#define FASTNET_CH_WATER_TEMP_C 0x20
#define FASTNET_CH_VMG 0x7F
#define FASTNET_CH_LOG_TRIM 0xCF
#define FASTNET_CH_TIMER 0x75  // Timer channel set to time
#define FASTNET_CH_TRUE_WIND_SPEED 0x57
#define FASTNET_CH_TRUE_WIND_ANGLE 0x59

uint8_t fastnet_header[5] = { 0xFF, 0x75, 0x14, 0x01, 0x77 };
uint8_t fastnet_buf[81];
uint8_t fastnet_buf_size = 0;

unsigned long lastSend = 0;

// ---------------- NMEA VALUES --------------
float nmea_depth = 0;
float nmea_awa = 0;
float nmea_twa = 0;
float nmea_tws = 0;
float nmea_aws = 0;
float nmea_sog = 0;
float nmea_temp = 0;
float nmea_volt = 0;
float nmea_vmg = 0;

int hour = 0;
int minute = 0;

// ---------------- NMEA BUFFER --------------
#define NMEA_MAX 82
char nmea_buf[NMEA_MAX];
// ------------------- CRC -------------------
uint8_t fastnet_crc(const uint8_t *data, uint8_t size, uint8_t init = 0) {
  uint16_t crc = init;
  for (uint8_t i = 0; i < size; i++) crc += data[i];
  return (0x100 - (crc & 0xFF)) & 0xFF;
}

// -------------- FASTNET PACKER (numeric) -------------
void fastnet_add_channel(uint8_t ch, uint8_t fmt, uint8_t size, uint8_t divisor, float value) {
  uint8_t need = FASTNET_FMT_BYTES[fmt];
  if (need == 0) return;  // unsupported format id
  if (fastnet_buf_size + need >= sizeof(fastnet_buf)) return;

  uint8_t *p = &fastnet_buf[fastnet_buf_size];
  p[0] = ch;
  p[1] = (fmt & 0xF) | ((size & 0x3) << 4) | ((divisor & 0x3) << 6);

  int32_t val = (int32_t)value;

  if (fmt == 1 && val < 0)
    val += 0x10000;  // 16-bit two's complement

  p[2] = (val >> 8) & 0xFF;
  p[3] = val & 0xFF;

  fastnet_buf_size += need;
}

// =====================================================
// FASTNET QUAD DEPTH (embedded in normal frame)
// Adds C1/C2/C3 blocks inside existing fastnet buffer
// =====================================================

void fastnet_add_depth_quad(uint16_t depth_mm) {
  const uint8_t NEED = 7;  // one C4 block

  if (fastnet_buf_size + NEED >= sizeof(fastnet_buf))
    return;

  // ---- convert depth ----
  // From observed packets, quad likely uses meters ×10
  uint16_t depth_val = depth_mm / 100;  // mm -> decimetres

  // ---- build C4 block ----

  fastnet_buf[fastnet_buf_size++] = 0x47;  // block marker
  fastnet_buf[fastnet_buf_size++] = 0xC4;  // depth channel
  fastnet_buf[fastnet_buf_size++] = 0x04;  // payload length LSB
  fastnet_buf[fastnet_buf_size++] = 0x00;  // payload length MSB

  fastnet_buf[fastnet_buf_size++] = depth_val & 0xFF;         // depth LSB
  fastnet_buf[fastnet_buf_size++] = (depth_val >> 8) & 0xFF;  // depth MSB

  // trailing byte (seen in real packets — likely checksum or flag)
  fastnet_buf[fastnet_buf_size++] = 0x00;
}


//speed
void fastnet_add_boatspeed_quad(float speed_knots) {
  const uint8_t NEED = 4;

  if (fastnet_buf_size + NEED >= sizeof(fastnet_buf))
    return;

  // Quad expects knots * 100
  uint16_t v = (uint16_t)(speed_knots * 100.0f + 0.5f);

  fastnet_buf[fastnet_buf_size++] = 0x41;  // speed channel
  fastnet_buf[fastnet_buf_size++] = 0x48;  // formatter (from sniffed frames)

  fastnet_buf[fastnet_buf_size++] = v & 0xFF;         // LSB
  fastnet_buf[fastnet_buf_size++] = (v >> 8) & 0xFF;  // MSB
}

// Sends initialization/header frames to Quad display
// Contains three frames with configuration data
void fastnet_add_quad_dump() {
  const uint8_t payload[] = {
    // ----- FRAME 1 -----
    0x70, 0x20, 0x01, 0x70,
    0x8D, 0x48, 0x00,
    0x8B, 0x50, 0x88, 0x00, 0x01,
    0x4E, 0x88, 0x00, 0x02,
    0x51, 0x01, 0x00, 0x03,
    0x52, 0x08, 0x00, 0x04,
    0x57, 0x88, 0x00, 0x05,
    0x59, 0x01, 0x00, 0x06,
    0x7F, 0x88, 0x00, 0x07,
    0xB0,

    // ----- FRAME 2 -----
    0x30, 0x14, 0x01, 0xBC,
    0x31, 0x04, 0x00,
    0x20, 0x46, 0x20,
    0xFE, 0x81, 0x12,
    0x84,
    0xFF, 0x81, 0xFF,
    0x47,
    0xC4, 0x04, 0x02,
    0x2D, 0x2D, 0x2D,
    0xD7,

    // ----- FRAME 3 -----
    0x30, 0x23, 0x01, 0xAD,
    0x41, 0x88, 0x00, 0x13,
    0x42, 0x88, 0x00, 0x14,
    0xCD, 0x42, 0x00, 0x01,
    0x03, 0x7C,
    0xCF, 0x48,
    0x7F, 0x09,
    0x81, 0x48, 0x00, 0x15,
    0xD3, 0x01, 0x00, 0x16,
    0x20, 0x41,
    0xFF,
    0x9D, 0x75, 0x03, 0x55
  };

  const uint16_t NEED = sizeof(payload);

  if (fastnet_buf_size + NEED >= sizeof(fastnet_buf))
    return;

  memcpy(&fastnet_buf[fastnet_buf_size], payload, NEED);
  fastnet_buf_size += NEED;
}

// ----------- TIMER CHANNEL PACKER (mm:ss, BCD, formatter 0101) -----------

// Public API: send timer value in seconds (converted to mm:ss)
// NOTE: this sends absolute minutes/seconds; sign/negative not encoded.
void fastnet_add_channel_timer_hhmm(uint8_t ch) {
  uint8_t fmt_id = 5;  // timer format
  uint8_t need = FASTNET_FMT_BYTES[fmt_id];

  if (need == 0) return;
  if (fastnet_buf_size + need >= sizeof(fastnet_buf)) return;

  uint8_t *p = &fastnet_buf[fastnet_buf_size];

  p[0] = ch;

  // Format byte: ZZ YY XXXX
  // For now: ZZ=00 (no divisor), YY=00 (4 digits), XXXX=0101 (timer)
  p[1] = 0x05;

  // Data bytes: minute and hour
  p[2] = minute;
  p[3] = hour;

  fastnet_buf_size += need;
}






// ---------------- SEND PACKET --------------
void fastnet_flush() {
  if (fastnet_buf_size >= sizeof(fastnet_buf)) return;  // safety check

  fastnet_header[2] = fastnet_buf_size;
  fastnet_header[4] = fastnet_crc(fastnet_header, 4);

  fastnet_buf[fastnet_buf_size] = fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

  Serial3.write(fastnet_header, 5);
  Serial3.write(fastnet_buf, fastnet_buf_size + 1);

  fastnet_buf_size = 0;
}

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

  // ---------- RMC ----------
  if (strncmp(s + 3, "RMC", 3) == 0) {
    char buf[96];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p = strtok(buf, ",");  // $IIRMC
    uint8_t field = 0;

    while (p) {
      if (field == 7) {  // SOG
        sog = atof(p) * 10;
      }
      if (field == 1 && strlen(p) >= 4) {  // time
        hour = (p[0] - '0') * 10 + (p[1] - '0');
        minute = (p[2] - '0') * 10 + (p[3] - '0');
      }
      p = strtok(NULL, ",");
      field++;
    }
    return;
  }

  // ---------- DPT ----------
  if (strncmp(s + 3, "DPT", 3) == 0) {
    char *p = strchr(s, ',');
    if (p) {
      depth_mm = atof(p + 1) * 1000;
    }
    return;
  }

  // ---------- MWV ----------
  if (strncmp(s + 3, "MWV", 3) == 0) {
    char buf[96];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p[8];
    uint8_t pc = 0;

    char *tok = strtok(buf, ",");
    while (tok && pc < 8) {
      p[pc++] = tok;
      tok = strtok(NULL, ",");
    }

    if (pc >= 5) {
      if (p[2][0] == 'R') {
        nmea_awa = atof(p[1]);
        if (nmea_awa > 180) nmea_awa -= 360;
        nmea_aws = atof(p[3]) * 10;
      }
      if (p[2][0] == 'T') {
        nmea_twa = atof(p[1]);
        if (nmea_twa > 180) nmea_twa -= 360;
        nmea_tws = atof(p[3]) * 10;
      }
    }
    return;
  }

  // ---------- VMG ----------
  if (strncmp(s + 3, "VMG", 3) == 0) {
    char *p = strchr(s, ',');
    if (p) {
      nmea_vmg = atof(p + 1);
    }
    return;
  }

  // ---------- BWC ----------
  if (strncmp(s + 3, "BWC", 3) == 0) {
    char buf[96];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p[20];
    uint8_t pc = 0;

    char *tok = strtok(buf, ",");
    while (tok && pc < 20) {
      p[pc++] = tok;
      tok = strtok(NULL, ",");
    }

    if (pc >= 13) {
      float bearing_true = atof(p[6]);

      char mark_id_raw[16];
      strncpy(mark_id_raw, p[12], sizeof(mark_id_raw) - 1);
      mark_id_raw[sizeof(mark_id_raw) - 1] = '\0';

      char *star = strchr(mark_id_raw, '*');
      if (star) *star = '\0';

      get_last6_with_prefix(mark_id_raw, brg_mark_name);

      brg_mark = bearing_true * 10;
      brg_valid = true;
    }
    return;
  }

  // ---------- ZDA ----------
  if (strncmp(s + 3, "ZDA", 3) == 0) {
    char *p = strchr(s, ',');
    if (!p) return;

    int hh = 0, mm = 0, ss = 0;
    sscanf(p + 1, "%d:%d:%d", &hh, &mm, &ss);

    countdown_sec = ss;
    return;
  }

  // ---------- XDR ----------
  if (strncmp(s + 3, "XDR", 3) == 0) {
    char buf[96];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p[8];
    uint8_t pc = 0;

    char *tok = strtok(buf, ",");
    while (tok && pc < 8) {
      p[pc++] = tok;
      tok = strtok(NULL, ",");
    }

    if (pc >= 3) {
      if (p[1][0] == 'C') {
        nmea_temp = atof(p[2]);
      }
      if (p[1][0] == 'U') {
        nmea_volt = atof(p[2]);
      }
    }
    return;
  }
}


void nmea_rx() {
  while (Serial2.available()) {

    char c = Serial2.read();
    if (c == '\n') {
      nmea_buf[nmea_pos] = 0;
      parse_nmea(nmea_buf);
      nmea_pos = 0;
    } else if (c != '\r') {
      if (nmea_pos < NMEA_MAX - 1)
        nmea_buf[nmea_pos++] = c;
      else
        nmea_pos = 0;  // overflow protection
    }
  }
}

// ================= DISPLAY LOGIC =================
uint16_t c_page = 0;
void update_mode() {
  if (depth_mm < DEPTH_LIMIT_MM) {
    mode = MODE_DEPTH;
    return;
  } else {
    mode = prev_mode;
  }

  // Timer mode disabled until implemented
  // if (countdown_sec >= 0) {
  //   mode = MODE_TIMER;
  //   return;
  // }

  if (brg_valid && millis() - t_brg > 15000) {  // show every 15 seconds
    mode = MODE_BRG;
    t_brg = millis();
    return;
  }

  if (millis() - t_brg > 1500) {  // back to SOG after 1.5 seconds
    mode = MODE_SPEED;
  }
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
    fastnet.config_page(CH_DR_COURSE, DEPTH_CPU, brg_mark_name, "%T");
  else if (mode == MODE_DEPTH)
    fastnet.config_page(CH_DEPTH_M, DEPTH_CPU, "DEPTH   ", " M");
  else
    fastnet.config_page(CH_BOAT_SPD_KT, DEPTH_CPU, "SOG     ", "KT");

  fastnet.store_page();
  delay(10);

  Serial1.flush();
  rs485_rx();
  prev_mode = mode;
}

// ================= DATA =================

void send_data() {
  rs485_tx();
  fastnet.boat_speed(sog);
  delay(7);  // display needs time to breathe
  fastnet.depth(depth_mm / 100);
  delay(10);
  fastnet.deadrecon(brg_mark / 10);

  Serial1.flush();
  rs485_rx();
}

void send_light() {
  rs485_tx();
  fastnet.backlight(LIGHT_LEVEL);
  Serial1.flush();
  rs485_rx();
}

// ================= SETUP =================

void setup() {
  pinMode(RS485_EN, OUTPUT);

  Serial.begin(115200);                     // debug USB
  Serial1.begin(FASTNET_BAUD, SERIAL_8O2);  // Fastnet 2020 display
  Serial2.begin(4800);                      // NMEA0183 input
  Serial3.begin(11000, SERIAL_8O2);         // Fastnet Quad display

  // Initialize backlight (send multiple times to ensure reception)
  for (int i = 0; i < 3; i++) {
    send_light();
    delay(10);
  }

  rs485_rx();
  delay(30);
  send_data();

  Serial.println("Starting..");
}

// ================= LOOP =================

void loop() {

  fastnet_rx();
  nmea_rx();
  update_mode();
  set_page();


  if (millis() - t_data > 100) {
    send_data();
    t_data = millis();
  }

  if (millis() - t_light > 60000) {  // 60 seconds
    send_light();
    t_light = millis();
  }

  // Send Quad header block every 10 seconds
  if (millis() - t_quad_header > 10000) {
    fastnet_add_quad_dump();
    t_quad_header = millis();
  }

  // Send Fastnet data every 100ms
  if (millis() - lastSend >= 100) {
    lastSend = millis();

    fastnet_add_channel(FASTNET_CH_SPEED_KNOTS, 8, 0, 1, sog);
    fastnet_add_channel(FASTNET_CH_VOLTAGE, 8, 0, 1, nmea_volt);
    fastnet_add_channel(FASTNET_CH_AWA, 8, 0, 0, nmea_awa);
    fastnet_add_channel(FASTNET_CH_AWS, 1, 0, 1, nmea_aws);
    fastnet_add_channel(FASTNET_CH_VMG, 8, 0, 1, sog * 10);
    fastnet_add_channel(FASTNET_CH_TRUE_WIND_SPEED, 1, 0, 1, nmea_tws);
    fastnet_add_channel(FASTNET_CH_TRUE_WIND_ANGLE, 1, 0, 1, nmea_twa);
    fastnet_add_channel(FASTNET_CH_DEPTH, 8, 0, 1, depth_mm / 100);
    fastnet_add_channel_timer_hhmm(FASTNET_CH_TIMER);

    fastnet_flush();
  }
}
