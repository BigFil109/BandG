/*
    Arduino FASTNET packet generator
    Creates random AWA (0–180°) and AWS (0–25 kn)
    Transmits using Serial at 11000 8E2 (even parity, 2 stop bits)
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

#define FASTNET_CH_AWA      0x51
#define FASTNET_CH_AWS      0x50
#define FASTNET_CH_VOLTAGE  0x8D

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
    // Insert payload size
    fastnet_header[2] = fastnet_buf_size;

    // Header CRC
    fastnet_header[4] = fastnet_crc(fastnet_header, 4);

    // Payload CRC
    fastnet_buf[fastnet_buf_size] = fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

    // Send header
    Serial.write(fastnet_header, 5);

    // Send payload + CRC
    Serial.write(fastnet_buf, fastnet_buf_size + 1);

    fastnet_buf_size = 0;
}

// -----------------------------------------------------------
// ---------------------- MAIN PROGRAM ------------------------
// -----------------------------------------------------------

void setup()
{
    // FASTNET UART specification → 11000 baud, 8 data bits, EVEN parity, 2 stop bits
    Serial.begin(11000, SERIAL_8O2);

    randomSeed(analogRead(A0));
}

void loop()
{
    float awa = random(-90, 90);         // Random 0–180°
    float aws = random(0, 250) / 10.0;  // Random 0–25.0 knots
    float tws = aws;
    float voltage = 10.0;               // Fake battery

    // Build FASTNET packet
    fastnet_add_channel(FASTNET_CH_AWA, 8, 0, 0, awa);
    fastnet_add_channel(FASTNET_CH_AWS, 1, 0, 1, aws);
    fastnet_add_channel(FASTNET_CH_VOLTAGE, 8, 0, 2, voltage);

    // Transmit
    fastnet_flush();

    delay(3000);  // Send fast updates
}
