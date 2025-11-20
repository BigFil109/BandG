/*
   FASTNET Sender + Full NMEA0183 Reader
   Nano Version — NO serial console output

   FASTNET OUT : Hardware Serial TX (Pin 1)
   NMEA IN     : SoftwareSerial RX on Pin 8
*/

#include <SoftwareSerial.h>

// ---------------- SOFTWARE SERIAL FOR NMEA0183 ----------------
SoftwareSerial nmeaSerial(8, 9);  // RX=8, TX=9 (TX unused)

// ---------------- FASTNET CONSTANTS ----------------
const uint8_t FASTNET_DIVISORS[] = {1, 10, 100, 1000};
const uint8_t FASTNET_FMT_BYTES[] = {
    0,4,4,5,6,0,0,0,4,0,0,6,0,0,0,0
};

#define FASTNET_CH_AWA        0x51
#define FASTNET_CH_AWS        0x50
#define FASTNET_CH_TWS        0x52
#define FASTNET_CH_DEPTH      0x40
#define FASTNET_CH_SOG        0x60
#define FASTNET_CH_STW        0x61
#define FASTNET_CH_RNG_NXT    0x70
#define FASTNET_CH_BRNG_NXT   0x71
#define FASTNET_CH_VOLTAGE    0x8D

uint8_t fastnet_header[5] = {0xFF, 0x75, 0x14, 0x01, 0x77};
uint8_t fastnet_buf[81];
uint8_t fastnet_buf_size = 0;

// ---------------- FASTNET CRC ---------------------
uint8_t fastnet_crc(uint8_t *data, uint8_t size, uint8_t init = 0)
{
    uint16_t crc = init;
    for (int i = 0; i < size; i++)
        crc += data[i];
    return (0x100 - (crc & 0xFF)) & 0xFF;
}

// --------------- FASTNET CHANNEL PACKING ----------
void fastnet_add_channel(uint8_t ch, uint8_t fmt,
                         uint8_t size, uint8_t divisor, float value)
{
    if ((sizeof(fastnet_buf) - fastnet_buf_size) <= FASTNET_FMT_BYTES[fmt])
        return;

    uint8_t offset = fastnet_buf_size;

    fastnet_buf[offset] = ch;
    fastnet_buf[offset + 1] =
        (fmt & 0xF) | ((size & 0x3) << 4) | ((divisor & 0x3) << 6);
    offset += 2;

    int32_t val = round(value * FASTNET_DIVISORS[divisor]);

    if (fmt == 1) { 
        if (val < 0) val = 0x10000 + val;
        fmt = 8;
    }

    fastnet_buf[offset]     = (val >> 8) & 0xFF;
    fastnet_buf[offset + 1] = val & 0xFF;

    fastnet_buf_size = offset + 2;
}

// ---------------- SEND FASTNET --------------------
void fastnet_flush()
{
    fastnet_header[2] = fastnet_buf_size;
    fastnet_header[4] = fastnet_crc(fastnet_header, 4);

    fastnet_buf[fastnet_buf_size] =
        fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

    Serial.write(fastnet_header, 5);
    Serial.write(fastnet_buf, fastnet_buf_size + 1);

    fastnet_buf_size = 0;
}

// ---------------- NMEA BUFFER ---------------------
String nmeaLine = "";

// ===================================================
// ===============  PARSER FUNCTIONS  ================
// ===================================================

void parseMWV(String s)
{
    String p[10];
    int n = 0;
    while (s.length() && n < 10) {
        int c = s.indexOf(',');
        if (c == -1) { p[n++] = s; break; }
        p[n++] = s.substring(0, c);
        s.remove(0, c + 1);
    }
    if (n < 6) return;
    if (!p[5].startsWith("A")) return;

    float angle = p[1].toFloat();
    float speed = p[3].toFloat();

    if (angle > 180.0) {
        angle = (float)angle - 360.0;
    }


    if (p[2] == "R") {  // Apparent
        fastnet_add_channel(FASTNET_CH_AWA, 8, 0, 0, angle);
        fastnet_add_channel(FASTNET_CH_AWS, 1, 0, 2, speed);
    }

    if (p[2] == "T") {  // True
        fastnet_add_channel(FASTNET_CH_TWS, 1, 0, 2, speed);
    }

    fastnet_flush();
}

void parseDPT(String s)
{
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    float depth = s.substring(c1 + 1, c2).toFloat();

    fastnet_add_channel(FASTNET_CH_DEPTH, 1, 0, 2, depth);
    fastnet_flush();
}

void parseVTG(String s)
{
    String p[12];
    int n = 0;
    while (s.length() && n < 12) {
        int c = s.indexOf(',');
        if (c == -1) { p[n++] = s; break; }
        p[n++] = s.substring(0, c);
        s.remove(0, c + 1);
    }

    float sog = p[7].toFloat();
    fastnet_add_channel(FASTNET_CH_SOG, 1, 0, 2, sog);
    fastnet_flush();
}

void parseVBW(String s)
{
    String p[10];
    int n = 0;
    while (s.length() && n < 10) {
        int c = s.indexOf(',');
        if (c == -1) { p[n++] = s; break; }
        p[n++] = s.substring(0, c);
        s.remove(0, c + 1);
    }

    float stw = p[1].toFloat();
    fastnet_add_channel(FASTNET_CH_STW, 1, 0, 2, stw);
    fastnet_flush();
}

void parseRMB(String s)
{
    String p[20];
    int n = 0;
    while (s.length() && n < 20) {
        int c = s.indexOf(',');
        if (c == -1) { p[n++] = s; break; }
        p[n++] = s.substring(0, c);
        s.remove(0, c + 1);
    }

    float range = p[9].toFloat();
    float bearing = p[10].toFloat();

    fastnet_add_channel(FASTNET_CH_RNG_NXT, 1, 0, 2, range);
    fastnet_add_channel(FASTNET_CH_BRNG_NXT, 1, 0, 0, bearing);

    fastnet_flush();
}

// ===================================================
// ================   READ NMEA  =====================
// ===================================================
void readNMEA()
{
    while (nmeaSerial.available())
    {
        char c = nmeaSerial.read();

        if (c == '$') {
            nmeaLine = "$";
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            if (!nmeaLine.startsWith("$")) { nmeaLine = ""; continue; }

            String body = nmeaLine.substring(0, nmeaLine.indexOf('*'));

            if (body.indexOf("MWV") > 0) parseMWV(body);
            if (body.indexOf("DPT") > 0) parseDPT(body);
            if (body.indexOf("VTG") > 0) parseVTG(body);
            if (body.indexOf("VBW") > 0) parseVBW(body);
            if (body.indexOf("RMB") > 0) parseRMB(body);

            nmeaLine = "";
        }
        else {
            nmeaLine += c;
        }
    }
}

// ===================================================
// ==================== SETUP ========================
// ===================================================
void setup()
{
    Serial.begin(11000, SERIAL_8O2);   // FASTNET OUT on TX pin (D1)
    nmeaSerial.begin(4800);            // NMEA IN on pin 8
}

// ==================== LOOP =========================
void loop()
{
    readNMEA();
}
