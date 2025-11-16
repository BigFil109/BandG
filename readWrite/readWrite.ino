/*
   FASTNET Sender + NMEA0183 MWV Reader
   Arduino Mega Version

   Serial0 (USB)       -> Debug log ONLY, 115200 baud
   Serial1 (RX1 pin 19)-> NMEA0183 input, 4800 baud
   Serial2 (TX2 pin 17)-> FASTNET output, 11000 baud 8O2
*/

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


// ---------------- FASTNET PACKING ----------------
void fastnet_add_channel(uint8_t ch, uint8_t fmt, uint8_t size, uint8_t divisor, float value)
{
    if ((sizeof(fastnet_buf) - fastnet_buf_size) <= FASTNET_FMT_BYTES[fmt])
        return;

    uint8_t offset = fastnet_buf_size;
    fastnet_buf[offset] = ch;
    fastnet_buf[offset + 1] =
        (fmt & 0xF) | ((size & 0x3) << 4) | ((divisor & 0x3) << 6);

    offset += 2;

    int32_t val = round(value * FASTNET_DIVISORS[divisor]);

    if (fmt == 1)
    {
        if (val < 0) val = 0x10000 + val;
        fmt = 8;
    }

    if (fmt == 8)
    {
        fastnet_buf[offset]     = (val >> 8) & 0xFF;
        fastnet_buf[offset + 1] = val & 0xFF;
        fastnet_buf_size = offset + 2;
    }
}


// ---------------- SEND FASTNET ON SERIAL2 ----------------
void fastnet_flush()
{
    fastnet_header[2] = fastnet_buf_size;
    fastnet_header[4] = fastnet_crc(fastnet_header, 4);

    fastnet_buf[fastnet_buf_size] =
        fastnet_crc(fastnet_buf, fastnet_buf_size, 0x56);

    Serial2.write(fastnet_header, 5);
    Serial2.write(fastnet_buf, fastnet_buf_size + 1);

    fastnet_buf_size = 0;
}


// ---------------------- NMEA READER --------------------------
String nmeaLine = "";

void processMWV(String s)
{
    // Split CSV
    int idx = 0;
    String part[10];
    while (s.length() && idx < 10)
    {
        int comma = s.indexOf(',');
        if (comma == -1)
        {
            part[idx++] = s;
            break;
        }
        part[idx++] = s.substring(0, comma);
        s.remove(0, comma + 1);
    }

    if (idx < 6) return;

    if (part[0].endsWith("MWV") && part[5].startsWith("A"))
    {
        float awa = part[1].toFloat();
        float aws = part[3].toFloat();

        Serial.print("MWV received AWA=");
        Serial.print(awa);
        Serial.print(" AWS=");
        Serial.println(aws);

        fastnet_add_channel(FASTNET_CH_AWA, 8, 0, 0, awa);
        fastnet_add_channel(FASTNET_CH_AWS, 1, 0, 2, aws);
        fastnet_add_channel(FASTNET_CH_VOLTAGE, 8, 0, 2, 11.6);

        fastnet_flush();
    }
}

void readNMEA()
{
    while (Serial1.available())
    {
        char c = Serial1.read();

        Serial.write(c);  // USB log only

        if (c == '$') {
            nmeaLine = "$";
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            if (nmeaLine.startsWith("$") && nmeaLine.indexOf("MWV") > 0)
            {
                String noChecksum = nmeaLine.substring(0, nmeaLine.indexOf('*'));
                processMWV(noChecksum);
            }
            nmeaLine = "";
        }
        else {
            nmeaLine += c;
        }
    }
}


// --------------------------- SETUP ----------------------------
void setup()
{
    Serial.begin(115200);                    // USB debug only
    Serial1.begin(4800);                     // NMEA input on pin 19
    Serial2.begin(11000, SERIAL_8O2);        // FASTNET output on pin 17

    Serial.println("NMEA->FASTNET bridge ready.");
    Serial.println("Listening on pin 19 (RX1).");
    Serial.println("Sending FASTNET on pin 17 (TX2).");
}


// --------------------------- LOOP -----------------------------
void loop()
{
    readNMEA();
}
