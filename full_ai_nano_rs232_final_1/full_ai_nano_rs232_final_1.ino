#include <Arduino.h>
#include <AltSoftSerial.h>
#include <avr/wdt.h>

// ---------------- NMEA INPUT ----------------
AltSoftSerial altSerial;  // RX=D8, TX=D9

// ---------------- FASTNET CONSTANTS ----------
// Divisors (00, 01, 10, 11 in the top 2 bits of format byte)
const uint8_t FASTNET_DIVISORS[] = {1, 10, 100, 1000};

// FASTNET_FMT_BYTES: total bytes per channel record = 1(ch) +1(fmt)+ data bytes
// index = format_id (lower 4 bits of the format byte)
//  - 0: 1+1+2 = 4
//  - 1: 1+1+2 = 4
//  - 2: 1+1+2 = 4
//  - 3: 1+1+3 = 5
//  - 4: 1+1+4 = 6
//  - 5: 1+1+2 = 4  (we now use this as "xx:xx timer" -> 2 data bytes, mm:ss BCD)
//  - 8: 1+1+2 = 4
//  - 11:1+1+4 = 6
// index: 0 1 2 3 4 5 6 7 8 9 10 11 ...
/*const uint8_t FASTNET_FMT_BYTES[] = {
    0,4,4,5,
    6,6,0,0,
    4,0,0,6,
    0,0,0,0
};*/

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

#define FASTNET_CH_AWS               0x50
#define FASTNET_CH_AWA               0x51
#define FASTNET_CH_VOLTAGE           0x8D // no nema feed atm
#define FASTNET_CH_DEPTH             0xC4
#define FASTNET_CH_SPEED_KNOTS       0x41
#define FASTNET_CH_WATER_TEMP_C      0x20
#define FASTNET_CH_VMG               0x7F
#define FASTNET_CH_LOG_TRIM          0xCF
#define FASTNET_CH_TIMER             0x75   // Timer channel set to time
#define FASTNET_CH_TRUE_WIND_SPEED   0x57
#define FASTNET_CH_TRUE_WIND_ANGLE   0x59

uint8_t fastnet_header[5] = {0xFF, 0x75, 0x14, 0x01, 0x77};
uint8_t fastnet_buf[81];
uint8_t fastnet_buf_size = 0;

// ---------------- NMEA VALUES --------------
float nmea_depth = 0;
float nmea_awa   = 0;
float nmea_twa   = 0;
float nmea_tws   = 0;
float nmea_aws   = 0;
float nmea_sog   = 0;
float nmea_temp  = 0;
float nmea_volt  = 0;
float nmea_vmg   = 0;

int hour = 0;
int minute = 0;

// ---------------- NMEA BUFFER --------------
#define NMEA_MAX 82
char nmea_buf[NMEA_MAX];
uint8_t nmea_pos = 0;


// ------------------- CRC -------------------
uint8_t fastnet_crc(const uint8_t *data, uint8_t size, uint8_t init = 0)
{
    uint16_t crc = init;
    for (uint8_t i=0;i<size;i++) crc += data[i];
    return (0x100 - (crc & 0xFF)) & 0xFF;
}

// -------------- FASTNET PACKER (numeric) -------------
void fastnet_add_channel(uint8_t ch, uint8_t fmt, uint8_t size, uint8_t divisor, float value)
{
    uint8_t need = FASTNET_FMT_BYTES[fmt];
    if (need == 0) return;  // unsupported format id
    if (fastnet_buf_size + need >= sizeof(fastnet_buf)) return;

    uint8_t *p = &fastnet_buf[fastnet_buf_size];
    p[0] = ch;
    p[1] = (fmt & 0xF) | ((size & 0x3)<<4) | ((divisor & 0x3)<<6);

    int32_t val = (int32_t)round(value * FASTNET_DIVISORS[divisor]);

    if (fmt == 1 && val < 0)
        val += 0x10000;  // 16-bit two's complement

    // For these numeric formats we only use the low 16 bits
    p[2] = (val>>8)&0xFF;
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
void fastnet_add_depth_c123(float depth_m)
{
    const uint8_t bytes_per_chan = 6;      // Cx 57 00 80 hi lo
    const uint8_t total_need     = bytes_per_chan * 3;

    if (fastnet_buf_size + total_need > sizeof(fastnet_buf)) return;

    uint8_t *p = &fastnet_buf[fastnet_buf_size];

    // Convert to metres / feet / fathoms as integers
    if (depth_m < 0) depth_m = 0;      // no negative depths

    float depth_ft   = depth_m * 3.28084f;
    float depth_fath = depth_m / 1.8288f;

    uint16_t v_m   = (uint16_t)round(depth_m);
    uint16_t v_ft  = (uint16_t)round(depth_ft);
    uint16_t v_fth = (uint16_t)round(depth_fath);

    // Clamp to 16-bit
    if (v_m   > 0xFFFF) v_m   = 0xFFFF;
    if (v_ft  > 0xFFFF) v_ft  = 0xFFFF;
    if (v_fth > 0xFFFF) v_fth = 0xFFFF;

    // Helper macro to write one Cx block
    auto write_depth_block = [&](uint8_t ch, uint16_t value) {
        *p++ = ch;          // C1 / C2 / C3
        *p++ = 0x57;        // format: fmt-id 7, matches your examples
        *p++ = 0x00;
        *p++ = 0x80;
        *p++ = (value >> 8) & 0xFF;   // high byte of depth
        *p++ = value & 0xFF;          // low byte of depth
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


// Public API: send timer value in seconds (converted to mm:ss)
// NOTE: this sends absolute minutes/seconds; sign/negative not encoded.


// ---------------- SEND PACKET --------------
void fastnet_flush()
{
    fastnet_header[2] = fastnet_buf_size;
    fastnet_header[4] = fastnet_crc(fastnet_header, 4);

    fastnet_buf[fastnet_buf_size] = fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

    Serial.write(fastnet_header, 5);
    Serial.write(fastnet_buf, fastnet_buf_size+1);

    fastnet_buf_size = 0;
}

// ------------------ NMEA PARSER ------------
void process_sentence(char *s)
{
    if (s[0] != '$') return;

    char *p[20];
    uint8_t count = 0;
    p[count++] = s+1;

    for (uint8_t i=1; s[i]; i++)
    {
        if (s[i] == ',' || s[i]=='*') {
            s[i] = 0;
            if (count < 20) p[count++] = &s[i+1];
        }
    }

    // ---- MWV ----
    if (!strcmp(p[0], "IIMWV"))
    {
            if (p[2][0]=='R')
            {
                nmea_awa = atof(p[1]);
                if (nmea_awa > 180) nmea_awa = nmea_awa - 360; // fastnet runs from -180 to +180
                nmea_aws = (atof(p[3]))*10.0;//real wind seed, was showing out by 10x on display
            }
            if (p[2][0]=='T')
            {
                nmea_twa = atof(p[1]);
                if (nmea_twa >= 180) nmea_twa = nmea_twa - 360; // fastnet runs from -180 to +180
                nmea_twa = nmea_twa / 10; //180 send 18.0
                nmea_tws = (atof(p[3]))*10.0;//was showing out by 10x on display
            }
        
    }

    else if (!strcmp(p[0],"IIMTW"))
        nmea_temp = atof(p[1]);

    else if (!strcmp(p[0],"IIDPT"))
        nmea_depth = atof(p[1]);

    else if (!strcmp(p[0],"IIVLW"))
        nmea_sog = atof(p[3]);

    else if (!strcmp(p[0],"IIVPW"))
        nmea_vmg = atof(p[1]);

    else if (!strcmp(p[0],"PGBV"))
        nmea_volt = atof(p[1]);

     else if (!strcmp(p[0], "IIRMC")) {

        String t = p[1];

        if (t.length() >= 4 && isDigit(t[0]) && isDigit(t[1]) &&
                                isDigit(t[2]) && isDigit(t[3])) {

            hour   = t.substring(0, 2).toInt();
            minute = t.substring(2, 4).toInt();
        }
    }
}

// --------------------- SETUP ----------------
void setup()
{
    Serial.begin(11000, SERIAL_8O2);
    altSerial.begin(4800);

    // ---- ENABLE WATCHDOG ----
    wdt_disable();
    delay(100);
    wdt_enable(WDTO_4S); 


}

// --------------------- LOOP -----------------
unsigned long lastSend = 0;
void loop()
{
    wdt_reset();

    while (altSerial.available())
    {
        char c = altSerial.read();

        if (c=='\n')
        {
            nmea_buf[nmea_pos] = 0;
            process_sentence(nmea_buf);
            nmea_pos = 0;
        }
        else if (c != '\r')
        {
            if (nmea_pos < NMEA_MAX-1)
                nmea_buf[nmea_pos++] = c;
            else
                nmea_pos = 0;  // overflow protection
        }
        wdt_reset();   
    }

    if (millis() - lastSend >= 750) {
        lastSend = millis();
        //working
        fastnet_add_channel(FASTNET_CH_VOLTAGE,         8, 0, 1, nmea_volt);
        fastnet_add_channel(FASTNET_CH_AWA,             8, 0, 0, nmea_awa);
        fastnet_add_channel(FASTNET_CH_AWS,             1, 0, 1, nmea_aws);
        fastnet_add_channel(FASTNET_CH_DEPTH,           8, 0, 1, 2.2);
        // *** NEW: depth as C1/C2/C3 in 57 00 80 00 xx format ***
        //fastnet_add_depth_c123(nmea_depth);


        fastnet_add_channel(FASTNET_CH_SPEED_KNOTS,     8, 0, 1, nmea_sog);
        fastnet_add_channel(FASTNET_CH_WATER_TEMP_C,    1, 0, 1, nmea_temp);
        fastnet_add_channel(FASTNET_CH_VMG,             8, 0, 2, nmea_vmg);
        fastnet_add_channel(FASTNET_CH_TRUE_WIND_SPEED, 1, 0, 1, nmea_tws);
        fastnet_add_channel(FASTNET_CH_TRUE_WIND_ANGLE, 1, 0, 1, nmea_twa);
        fastnet_add_channel_timer_hhmm(FASTNET_CH_TIMER);        
        
        fastnet_flush();
    }
}
