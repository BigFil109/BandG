/*
    FASTNET Light-Off Brute Force Scanner
    - sends all known Fastnet light formats
    - prints channel + format to USB serial monitor
    - no NMEA input
    - Fastnet output = pin 1 at 11000 baud 8E2
*/

#include <Arduino.h>

// ----------- SEND RAW FASTNET FRAME -----------
void sendFastnet(uint8_t channel, uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x56;
    for (uint8_t i = 0; i < len; i++)
        crc += data[i];
    crc = (0x100 - (crc & 0xFF)) & 0xFF;

    uint8_t header[5] = {0xFF, 0x75, len, 0x01, 0x00};
    uint8_t hcrc = 0;
    for (int i = 0; i < 4; i++) hcrc += header[i];
    header[4] = (0x100 - (hcrc & 0xFF)) & 0xFF;

    // Header
    Serial.write(header, 5);
    // Channel byte first
    Serial.write(channel);
    // Payload bytes
    Serial.write(data, len - 1);
    // CRC
    Serial.write(crc);
}

// ----------- PACKET FORMAT FUNCTIONS -----------

void sendType_A(uint8_t ch)   // Known B&G pattern: v1=0, v2=0x20
{
    uint8_t payload[3] = { ch, 0x00, 0x20 };
    sendFastnet(ch, payload, 3);
}

void sendType_B(uint8_t ch)   // Single byte = 0
{
    uint8_t payload[2] = { ch, 0x00 };
    sendFastnet(ch, payload, 2);
}

void sendType_C(uint8_t ch)   // 2-byte unsigned = 0
{
    uint8_t payload[3] = { ch, 0x00, 0x00 };
    sendFastnet(ch, payload, 3);
}

void sendType_D(uint8_t ch)   // Signed int16 = 0
{
    uint8_t payload[3] = { ch, 0x00, 0x00 };
    sendFastnet(ch, payload, 3);
}

void sendType_E(uint8_t ch)   // 4-byte zero block
{
    uint8_t payload[5] = { ch, 0x00, 0x00, 0x00, 0x00 };
    sendFastnet(ch, payload, 5);
}

void sendType_F(uint8_t ch)   // 6-byte zero block
{
    uint8_t payload[7] = { ch, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sendFastnet(ch, payload, 7);
}

// ------------------------------------------------------

void setup()
{
    // USB debug serial
    Serial1.begin(115200);  // <-- USB Serial Monitor
    Serial1.println("Fastnet Light-Off Scanner Starting...");

    // Fastnet bus OUT (TX pin 1)
    Serial.begin(11000, SERIAL_8E2);
}

void loop()
{
   Serial1.print("Channel 0x");
        Serial1.print("HI");

        delay(500);
    for (uint16_t ch = 0; ch <= 0xFF; ch++)
    {
        Serial1.print("Channel 0x");
        Serial1.print(ch, HEX);
        Serial1.println(" Type A (00 20)");
        sendType_A(ch);
        delay(150);

        Serial1.print("Channel 0x");
        Serial1.print(ch, HEX);
        Serial1.println(" Type B (00)");
        sendType_B(ch);
        delay(150);

        Serial1.print("Channel 0x");
        Serial1.print(ch, HEX);
        Serial1.println(" Type C (00 00)");
        sendType_C(ch);
        delay(150);

        Serial1.print("Channel 0x");
        Serial1.print(ch, HEX);
        Serial1.println(" Type D (signed 0)");
        sendType_D(ch);
        delay(150);

        Serial1.print("Channel 0x");
        Serial1.print(ch, HEX);
        Serial1.println(" Type E (4x00)");
        sendType_E(ch);
        delay(200);

        Serial1.print("Channel 0x");
        Serial1.print(ch, HEX);
        Serial1.println(" Type F (6x00)");
        sendType_F(ch);
        delay(200);

        Serial1.print("Completed channel 0x");
        Serial1.println(ch, HEX);
        Serial1.println();
        
        delay(500);
    }

    Serial1.println("Scan complete. Restarting...");
    delay(9999999);
}
