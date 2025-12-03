/*
    FASTNET Test Signal Generator
    Produces cycling test values for:
      - Depth
      - Speed through water (STW)
      - AWA / AWS
      - TWA / TWS
      - Battery voltage
*/

#include <Arduino.h>

// ---------------- FASTNET CONSTANTS ----------------
const uint8_t FASTNET_DIVISORS[] = {1, 10, 100, 1000};

const uint8_t FASTNET_FMT_BYTES[] = {
    0, 4, 4, 5,
    6, 0, 0, 0,
    4, 0, 0, 6,
    0, 0, 0, 0
};

//Good
#define FASTNET_CH_AWS          0x50
#define FASTNET_CH_AWA          0x51
#define FASTNET_CH_VOLTAGE      0x8D

#define FASTNET_CH_DEPTH                  0xC4  
#define FASTNET_CH_SPEED_KNOTS            0x41
#define FASTNET_CH_SPEED_KNOTS_TRIGGER    0x75

#define FASTNET_CH_WATER_TEMP_C           0x20
#define FASTNET_CH_VMG          0x7F

#define FASTNET_CH_LOG_TRIM  0xCF


uint8_t fastnet_header[5] = {0xFF, 0x75, 0x14, 0x01, 0x77};
uint8_t fastnet_buf[81];
uint8_t fastnet_buf_size = 0;

// ---------------- CRC ----------------
uint8_t fastnet_crc(uint8_t *data, uint8_t size, uint8_t init = 0)
{
    uint16_t crc = init;
    for (int i = 0; i < size; i++)
        crc += data[i];
    return (0x100 - (crc & 0xFF)) & 0xFF;
}

// ---------------- FASTNET CHANNEL PACKING ----------------
void fastnet_add_channel(uint8_t ch, uint8_t fmt, uint8_t size, uint8_t divisor, float value)
{
    if ((sizeof(fastnet_buf) - fastnet_buf_size) <= FASTNET_FMT_BYTES[fmt])
        return;

    uint8_t offset = fastnet_buf_size;
    fastnet_buf[offset] = ch;
    fastnet_buf[offset + 1] = (fmt & 0xF) | ((size & 0x3) << 4) | ((divisor & 0x3) << 6);
    offset += 2;

    int32_t val = round(value * FASTNET_DIVISORS[divisor]);

    if (fmt == 1) {
        if (val < 0)
            val = 0x10000 + val;
        fmt = 8;
    }

    if (fmt == 8) {
        fastnet_buf[offset]     = (val >> 8) & 0xFF;
        fastnet_buf[offset + 1] = (val & 0xFF);
        fastnet_buf_size = offset + 2;
    }
}

// ---------------- SEND PACKET ----------------
void fastnet_flush()
{
    fastnet_header[2] = fastnet_buf_size;
    fastnet_header[4] = fastnet_crc(fastnet_header, 4);

    fastnet_buf[fastnet_buf_size] = fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

    Serial.write(fastnet_header, 5);
    Serial.write(fastnet_buf, fastnet_buf_size + 1);

    fastnet_buf_size = 0;
}

// -----------------------------------------------------------
// ---------------------- MAIN PROGRAM ------------------------
// -----------------------------------------------------------

float depth_val = 5;    // metres
float speed_val = 0.0;  // knots
float batt_volts = 12.7;

void setup()
{
    Serial.begin(11000, SERIAL_8O2);
    randomSeed(analogRead(A0));
}


void loop()
{
    
    //  -- batt volts 
    batt_volts += 0.2;
    if (batt_volts > 14.6) batt_volts = 6.0;

    // ---- Cycle depth 5m → 50m then reset ----
    depth_val += 0.5;
    if (depth_val > 50) depth_val = 5;

    // ---- Cycle boat speed 0 → 12 knots ----
    speed_val += 0.2;
    if (speed_val > 12) speed_val = 0;

    float awa = random(-20, 01);
    float aws = random(0, 250) / 10.0;
    float tws = aws;

    //Good
    fastnet_add_channel(FASTNET_CH_VOLTAGE,     8, 0, 1, batt_volts);//battery 
    fastnet_add_channel(FASTNET_CH_AWA,       8, 0, 0, awa); //app wind angle
    fastnet_add_channel(FASTNET_CH_AWS,       1, 0, 1, aws); //app wind speed
    fastnet_add_channel(FASTNET_CH_DEPTH, 1, 0, 1, depth_val);//depth M C4


    //need to send pair, second value does nothing
    fastnet_add_channel(FASTNET_CH_SPEED_KNOTS, 8, 0, 1, speed_val);//knots  41
    fastnet_add_channel(FASTNET_CH_SPEED_KNOTS_TRIGGER, 8, 0, 1, 0.0);//knots  75

    fastnet_add_channel(FASTNET_CH_WATER_TEMP_C, 1, 0, 1, batt_volts); //Temp C
    fastnet_add_channel(FASTNET_CH_VMG, 8, 0, 2, speed_val); //VMG
    fastnet_add_channel(FASTNET_CH_LOG_TRIM, 8, 0, 2, 0.0);//set trip to 0

    // Send packet
    fastnet_flush();

    delay(500);
}
