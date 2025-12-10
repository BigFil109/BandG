#include <Arduino.h>
#include <AltSoftSerial.h>

// ====================== NMEA INPUT ==========================
//SoftwareSerial NMEAserial(8, 7);
AltSoftSerial altSerial(8,7);   
// RX = D8  (connect NMEA0183 → RS323 TTL → D8)
// TX = D7  (unused)

// ============================================================
// =============== FASTNET CONSTANTS ==========================
// ============================================================
const uint8_t FASTNET_DIVISORS[] = {1, 10, 100, 1000};

const uint8_t FASTNET_FMT_BYTES[] = {
    0, 4, 4, 5,
    6, 0, 0, 0,
    4, 0, 0, 6,
    0, 0, 0, 0
};

// FASTNET Channels
#define FASTNET_CH_AWS               0x50
#define FASTNET_CH_AWA               0x51
#define FASTNET_CH_VOLTAGE           0x8D
#define FASTNET_CH_DEPTH             0xC4
#define FASTNET_CH_SPEED_KNOTS       0x41
#define FASTNET_CH_SPEED_KNOTS_TRIG  0x75
#define FASTNET_CH_WATER_TEMP_C      0x20
#define FASTNET_CH_VMG               0x7F
#define FASTNET_CH_LOG_TRIM          0xCF

uint8_t fastnet_header[5] = {0xFF, 0x75, 0x14, 0x01, 0x77};
uint8_t fastnet_buf[81];
uint8_t fastnet_buf_size = 0;


// ============================================================
// ===================== CRC ==================================
// ============================================================
uint8_t fastnet_crc(uint8_t *data, uint8_t size, uint8_t init = 0)
{
    uint16_t crc = init;
    for (int i = 0; i < size; i++)
        crc += data[i];
    return (0x100 - (crc & 0xFF)) & 0xFF;
}


// ============================================================
// ========== FASTNET CHANNEL PACK (unchanged) ================
// ============================================================
void fastnet_add_channel(uint8_t ch, uint8_t fmt, uint8_t size, uint8_t divisor, float value)
{
    if ((sizeof(fastnet_buf) - fastnet_buf_size) <= FASTNET_FMT_BYTES[fmt])
        return;

    uint8_t offset = fastnet_buf_size;
    fastnet_buf[offset] = ch;
    fastnet_buf[offset+1] =
        (fmt & 0xF) | ((size & 0x3) << 4) | ((divisor & 0x3) << 6);

    offset += 2;

    int32_t val = round(value * FASTNET_DIVISORS[divisor]);

    if (fmt == 1) {
        if (val < 0) val = 0x10000 + val;
        fmt = 8;
    }

    if (fmt == 8) {
        fastnet_buf[offset]   = (val >> 8) & 0xFF;
        fastnet_buf[offset+1] = (val & 0xFF);
        fastnet_buf_size = offset + 2;
    }
}


// ============================================================
// ====================== SEND PACKET =========================
// ============================================================
void fastnet_flush()
{
    fastnet_header[2] = fastnet_buf_size;
    fastnet_header[4] = fastnet_crc(fastnet_header, 4);

    fastnet_buf[fastnet_buf_size] = fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

    Serial.write(fastnet_header, 5);
    Serial.write(fastnet_buf, fastnet_buf_size + 1);

    fastnet_buf_size = 0;
}


// ============================================================
// ========== PARSED NMEA VALUES ==============================
// ============================================================
float nmea_depth = 0;
float nmea_awa   = 0;
float nmea_awa_true   = 0;
float nmea_aws   = 0;
float nmea_sog   = 0;
float nmea_temp  = 0;
float nmea_volt  = 0;
float nmea_vmg   = 0;

String nmea = "";


// ============================================================
// ===================== SPLIT ================================
// ============================================================
int split(String s, char d, String out[], int max)
{
    int count = 0;
    int start = 0;

    for (int i = 0; i < s.length(); i++) {
        if (s[i] == d) {
            out[count++] = s.substring(start, i);
            start = i + 1;
            if (count >= max) break;
        }
    }
    out[count++] = s.substring(start);
    return count;
}


// ============================================================
// =================== NMEA PROCESSING ========================
// ============================================================
void processSentence(String s)
{
    if (!s.startsWith("$")) return;

    String p[20];
    
     int idx = 0;
    for (int i = 0; i < s.length(); i++)
    {
        if (s[i] == ',' || s[i] == '*')
        {
            idx++;
            continue;
        }
        p[idx] += s[i];
    }

   // int n = split(s, ',', p, 20);
  
    // 17:43:17.700 -> $IIMWV,015,R,00.0,N,A*17
    if (s.startsWith("$IIMWV") ) {

        if(p[2] == "L" || p[2] =="R"){
            nmea_awa = p[1].toFloat();
            if(p[2].startsWith("L")){
                nmea_awa *= -1;
            }
            nmea_aws = p[3].toFloat();
        }

        if(p[2] == "T"){
           nmea_awa_true = p[3].toFloat();
        }


    }

    // MTW: Water Temp : IMP
    else if (s.startsWith("$IIMTW"))
    {
        // $IIMTW,+08.0,C*30
        String value = p[1];
        nmea_temp = value.toFloat();
    }


    // DPT: Depth
    else if (s.startsWith("$IIDPT")) {
        nmea_depth = p[1].toFloat();
    }

    // VTG: Speed SOG $IIVLW,01471,N,000.0,N*50
    else if (s.startsWith("$IIVLW")) {
        nmea_sog = p[3].toFloat();
    }

    //VMG $IIVPW,5.4,N*3C
    else if (s.startsWith("$IIVPW")) {
        nmea_vmg = p[1].toFloat();
    }

    
}


// ============================================================
// ======================= SETUP ===============================
// ============================================================
void setup()
{
    // FASTNET out
    Serial.begin(11000, SERIAL_8O2);

    // SoftwareSerial NMEA input
   // NMEAserial.begin(4800);
   altSerial.begin(4800);

   // altSerial.reserve(82);
}


// ============================================================
// ======================== LOOP ===============================
// ============================================================
void loop()
{
    // -------------- READ NMEA0183 from SoftwareSerial ----------
    while (altSerial.available()) {
        char c = altSerial.read();

        if (c == '\n') {
            processSentence(nmea);
            nmea = "";
        }
        else if (c != '\r') {
            nmea += c;
        }
    }

    // -------------- SEND FASTNET -------------------------------

    fastnet_add_channel(FASTNET_CH_VOLTAGE, 8, 0, 1, nmea_volt);
    fastnet_add_channel(FASTNET_CH_AWA,     8, 0, 0, nmea_awa);
    fastnet_add_channel(FASTNET_CH_AWS,     1, 0, 1, nmea_aws);

    fastnet_add_channel(FASTNET_CH_DEPTH,   1, 0, 1, nmea_depth);

    fastnet_add_channel(FASTNET_CH_SPEED_KNOTS, 8, 0, 1, nmea_sog);
    fastnet_add_channel(FASTNET_CH_SPEED_KNOTS_TRIG, 8, 0, 1, 0.0);

    fastnet_add_channel(FASTNET_CH_WATER_TEMP_C, 1, 0, 1, nmea_temp);

    fastnet_add_channel(FASTNET_CH_VMG,     8, 0, 2, nmea_vmg);

    fastnet_add_channel(FASTNET_CH_LOG_TRIM, 8, 0, 2, 0.0);

    fastnet_flush();

    delay(250);
}
