#include <Arduino.h>
#include <AltSoftSerial.h>
#include <avr/wdt.h>

// ---------------- NMEA INPUT ----------------
AltSoftSerial altSerial;  // RX=D8, TX=D9

// ---------------- FASTNET CONSTANTS ----------
const uint8_t FASTNET_DIVISORS[] = {1, 10, 100, 1000};
const uint8_t FASTNET_FMT_BYTES[] = {
    0,4,4,5,
    6,0,0,0,
    4,0,0,6,
    0,0,0,0
};

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

// ---------------- NMEA VALUES --------------
float nmea_depth = 0;
float nmea_awa   = 0;
float nmea_awa_true = 0;
float nmea_tws   = 0;
float nmea_aws   = 0;
float nmea_sog   = 0;
float nmea_temp  = 0;
float nmea_volt  = 0;
float nmea_vmg   = 0;

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

// -------------- FASTNET PACKER -------------
void fastnet_add_channel(uint8_t ch, uint8_t fmt, uint8_t size, uint8_t divisor, float value)
{
    uint8_t need = FASTNET_FMT_BYTES[fmt];
    if (fastnet_buf_size + need >= sizeof(fastnet_buf)) return;

    uint8_t *p = &fastnet_buf[fastnet_buf_size];
    p[0] = ch;
    p[1] = (fmt & 0xF) | ((size & 0x3)<<4) | ((divisor & 0x3)<<6);

    int32_t val = (int32_t)round(value * FASTNET_DIVISORS[divisor]);

    if (fmt == 1 && val < 0)
        val += 0x10000;

    p[2] = (val>>8)&0xFF;
    p[3] = val & 0xFF;

    fastnet_buf_size += need;
}

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
        if (p[2][0]=='L' || p[2][0]=='R')
        {
            nmea_awa = atof(p[1]);
            if (p[2][0]=='L') nmea_awa = -nmea_awa;
            nmea_aws = atof(p[3]);
        }
        if (p[2][0]=='T')
        {
            nmea_awa_true = atof(p[1]);
            nmea_tws      = atof(p[3]);
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

    if (millis() - lastSend >= 200) {
        lastSend = millis();

        fastnet_add_channel(FASTNET_CH_VOLTAGE, 8, 0, 1, nmea_volt);
        fastnet_add_channel(FASTNET_CH_AWA,     8, 0, 0, nmea_awa);
        fastnet_add_channel(FASTNET_CH_AWS,     1, 0, 1, nmea_aws);
        fastnet_add_channel(FASTNET_CH_DEPTH,   1, 0, 1, nmea_depth);
        fastnet_add_channel(FASTNET_CH_SPEED_KNOTS, 8, 0, 1, nmea_sog);
        fastnet_add_channel(FASTNET_CH_SPEED_KNOTS_TRIG, 8, 0, 1, 0);
        fastnet_add_channel(FASTNET_CH_WATER_TEMP_C, 1, 0, 1, nmea_temp);
        fastnet_add_channel(FASTNET_CH_VMG, 8, 0, 2, nmea_vmg);
        fastnet_add_channel(FASTNET_CH_LOG_TRIM, 8, 0, 2, 0);
        fastnet_add_channel(0x75, 8, 0, 1, );

        fastnet_flush();
    }
}
