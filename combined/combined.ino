#include <Arduino.h>
#include <avr/wdt.h>
#include "fastnet.h"

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
const uint8_t FASTNET_DIVISORS[] = { 1, 10, 100, 1000 };


//*******
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

  //int32_t val = (int32_t)round(value * FASTNET_DIVISORS[divisor]);
  int32_t val = (int32_t)value;

  if (fmt == 1 && val < 0)
    val += 0x10000;  // 16-bit two's complement

  // For these numeric formats we only use the low 16 bits
  p[2] = (val >> 8) & 0xFF;
  p[3] = val & 0xFF;

  fastnet_buf_size += need;
}

// ----------- SPECIAL DEPTH FORMAT: C1/C2/C3 -------------
// ----------- SPECIAL DEPTH FORMAT: C1/C2/C3 -------------
// Emits depth in three units as:
//   C1 57 00 80 00 0F   (metres)
//   C2 57 00 80 00 32   (feet)
//   C3 57 00 80 00 08   (fathoms)
//
// We keep 57 00 80 fixed, and put the 16-bit depth in the last two bytes.
// The frame-level checksum is handled later by fastnet_flush().
void fastnet_add_depth_c123(float depth_m) {
  const uint8_t bytes_per_chan = 6;  // Cx 57 00 80 hi lo
  const uint8_t total_need = bytes_per_chan * 3;

  if (fastnet_buf_size + total_need > sizeof(fastnet_buf)) return;

  uint8_t *p = &fastnet_buf[fastnet_buf_size];

  // Convert to metres / feet / fathoms as integers
  if (depth_m < 0) depth_m = 0;  // no negative depths

  float depth_ft = depth_m * 3.28084f;
  float depth_fath = depth_m / 1.8288f;

  uint16_t v_m = (uint16_t)round(depth_m);
  uint16_t v_ft = (uint16_t)round(depth_ft);
  uint16_t v_fth = (uint16_t)round(depth_fath);

  // Clamp to 16-bit
  if (v_m > 0xFFFF) v_m = 0xFFFF;
  if (v_ft > 0xFFFF) v_ft = 0xFFFF;
  if (v_fth > 0xFFFF) v_fth = 0xFFFF;

  // Helper macro to write one Cx block
  auto write_depth_block = [&](uint8_t ch, uint16_t value) {
    *p++ = ch;    // C1 / C2 / C3
    *p++ = 0x57;  // format: fmt-id 7, matches your examples
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = (value >> 8) & 0xFF;  // high byte of depth
    *p++ = value & 0xFF;         // low byte of depth
  };

  // C1 – metres
  write_depth_block(0xC1, v_m);
  // C2 – feet
  write_depth_block(0xC2, v_ft);
  // C3 – fathoms
  write_depth_block(0xC3, v_fth);

  fastnet_buf_size = (uint8_t)(p - fastnet_buf);
}



// ----------- TIMER CHANNEL PACKER (mm:ss, BCD, formatter 0101) -----------

// Public API: send timer value in seconds (converted to mm:ss)
// NOTE: this sends absolute minutes/seconds; sign/negative not encoded.
void fastnet_add_channel_timer_hhmm(uint8_t ch)
{
    uint8_t fmt_id = 5;                  // timer format
    uint8_t need   = FASTNET_FMT_BYTES[fmt_id];

    if (need == 0) return;
    if (fastnet_buf_size + need >= sizeof(fastnet_buf)) return;

    uint8_t *p = &fastnet_buf[fastnet_buf_size];

    p[0] = ch;

    // Format byte: ZZ YY XXXX
    // For now: ZZ=00 (no divisor), YY=00 (4 digits), XXXX=0101 (timer)
    p[1] = 0x05;

    // Data bytes as per pyfastnet:
    //   d0: "useless" (can be 0)
    //   d1: hours (may exceed 24)
    //   d2: minutes
    p[2] = minute;          // useless
    p[3] = hour;
   // p[4] = -1;//alarm
   // p[5] = -1;
   // p[6] = -1;
  

    fastnet_buf_size += need;
}






// ---------------- SEND PACKET --------------
void fastnet_flush() {
  fastnet_header[2] = fastnet_buf_size;
  fastnet_header[4] = fastnet_crc(fastnet_header, 4);

  fastnet_buf[fastnet_buf_size] = fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

  Serial3.write(fastnet_header, 5);
  Serial3.write(fastnet_buf, fastnet_buf_size + 1);

  fastnet_buf_size = 0;
}
//******





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

  // Raw sentence log
  //Serial.print(F("[NMEA] "));
 // Serial.println(s);

  // ---------- RMC ----------
  if (strstr(s, "$IIRMC")) {
    char buf[96];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p = strtok(buf, ",");  // $IIRMC
    uint8_t field = 0;

    while (p) {
      if (field == 7) {  // SOG
        sog = atof(p) * 10;
      //  Serial.print(F("[RMC] SOG set = "));
      //  Serial.println(sog);
      }
      if (field == 1 && strlen(p) >= 4) {  // time
        hour = (p[0] - '0') * 10 + (p[1] - '0');
        minute = (p[2] - '0') * 10 + (p[3] - '0');
       // Serial.print(F("[RMC] Time = "));
       // Serial.print(hour);
       // Serial.print(':');
       // Serial.println(minute);
      }
      p = strtok(NULL, ",");
      field++;
    }
    return;
  }

  // ---------- DPT ----------
  if (strstr(s, "IIDPT")) {
    char *p = strchr(s, ',');
    if (p) {
      depth_mm = atof(p + 1) * 1000;
    //  Serial.print(F("[DPT] Depth mm = "));
     // Serial.println(depth_mm);
    }
    return;
  }

  // ---------- MWV ----------
  if (strstr(s, "IIMWV")) {
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
     //   Serial.print(F("[MWV] AWA/AWS = "));
      //  Serial.print(nmea_awa);
      //  Serial.print(F(" / "));
      //  Serial.println(nmea_aws);
      }
      if (p[2][0] == 'T') {
        nmea_twa = atof(p[1]);
        if (nmea_twa > 180) nmea_twa -= 360;
        nmea_tws = atof(p[3]) * 10;
       // Serial.print(F("[MWV] TWA/TWS = "));
       // Serial.print(nmea_twa);
       // Serial.print(F(" / "));
        //Serial.println(nmea_tws);
      }
    }
    return;
  }

  // ---------- VMG ----------
  if (strstr(s, "IIVMG")) {
    char *p = strchr(s, ',');
    if (p) {
      nmea_vmg = atof(p + 1);
     // Serial.print(F("[VMG] VMG = "));
     // Serial.println(nmea_vmg);
    }
    return;
  }

  // ---------- BWC ----------
  if (strstr(s, "GPBWC")) {
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

     // Serial.print(F("[BWC] Bearing = "));
     // Serial.print(brg_mark);
     // Serial.print(F("  Mark = "));
     // Serial.println(brg_mark_name);
    }
    return;
  }

  // ---------- ZDA ----------
  if (strstr(s, "IIZDA")) {
    char *p = strchr(s, ',');
    if (!p) return;

    int hh = 0, mm = 0, ss = 0;
    sscanf(p + 1, "%d:%d:%d", &hh, &mm, &ss);

    countdown_sec = ss;

   // Serial.print(F("[ZDA] Countdown seconds = "));
   // Serial.println(countdown_sec);
    return;
  }

  // ---------- XDR ----------
  if (strstr(s, "XDR")) {
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
     //   Serial.print(F("[XDR] Temp = "));
     //   Serial.println(nmea_temp);
      }
      if (p[1][0] == 'U') {
        nmea_volt = atof(p[2]);
      //  Serial.print(F("[XDR] Volt = "));
      //  Serial.println(nmea_volt);
      }
    }
    return;
  }
}


void nmea_rx() {
  while (Serial2.available()) {

    char c = Serial2.read();
     if (c=='\n')
        {
            nmea_buf[nmea_pos] = 0;
            parse_nmea(nmea_buf);
            nmea_pos = 0;
        }
        else if (c != '\r')
        {
            if (nmea_pos < NMEA_MAX-1)
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

 /* untill we get the timer on imp 
 if (countdown_sec >= 0) {
    mode = MODE_TIMER;
    return;
  }*/

  if (brg_valid && millis() - t_brg > 15000) {  //show everu 20 seconds
    mode = MODE_BRG;
    t_brg = millis();
    return;
  }

  if (millis() - t_brg > 1500) {  //back to sog after 2 seconds
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
  delay(10);  // display needs time to breath :)
  fastnet.depth(depth_mm / 100);
  delay(10);
  fastnet.timer(countdown_sec >= 0 ? countdown_sec : 0);
  delay(10);
  fastnet.deadrecon(brg_mark / 10);
  delay(10);

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
 
  Serial.begin(115200);                     //debug usb
  Serial1.begin(FASTNET_BAUD, SERIAL_8O2);  //2020 ouput
  Serial2.begin(4800);                      //nema0183
  Serial3.begin(11000, SERIAL_8O2);         //network output ??

  send_light();
  delay(10);
  send_light();
  delay(10);
  send_light();
  delay(10);

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

   if (millis() - t_light > 60444) {
    send_light();
    t_light = millis();
  }

  if (millis() - lastSend >= 100) {
    lastSend = millis();
    //working
    fastnet_add_channel(FASTNET_CH_VOLTAGE, 8, 0, 1, nmea_volt);
    fastnet_add_channel(FASTNET_CH_AWA, 8, 0, 0, nmea_awa);
    fastnet_add_channel(FASTNET_CH_AWS, 1, 0, 1, nmea_aws);
    fastnet_add_channel(FASTNET_CH_DEPTH, 8, 0, 1, depth_mm/100);
    //fastnet_add_channel(0xC1, 8, 0, 1, depth_mm / 200);
    // fastnet_add_channel(0xC2, 8, 0, 1, depth_mm / 300);
    // fastnet_add_channel(0xC3, 8, 0, 1, depth_mm / 400);
    // *** NEW: depth as C1/C2/C3 in 57 00 80 00 xx format ***
    //fastnet_add_depth_c123(depth_mm/100);

   
    fastnet_add_channel(FASTNET_CH_SPEED_KNOTS, 8, 0, 1, sog);
    fastnet_add_channel(FASTNET_CH_WATER_TEMP_C, 1, 0, 1, nmea_temp);
    fastnet_add_channel(FASTNET_CH_VMG, 8, 0, 1, sog*10);
    fastnet_add_channel(FASTNET_CH_TRUE_WIND_SPEED, 1, 0, 1, nmea_tws);
    fastnet_add_channel(FASTNET_CH_TRUE_WIND_ANGLE, 1, 0, 1, nmea_twa);
    fastnet_add_channel_timer_hhmm(FASTNET_CH_TIMER);


  delay(100);
    fastnet_flush();
  }
}
