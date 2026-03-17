#include <SoftwareSerial.h>

#define BUS_RX   2
#define BUS_BAUD 11000     // known working speed

SoftwareSerial bus(BUS_RX, -1);

// packet buffer
#define MAX_FRAME 128

uint8_t frame[MAX_FRAME];
uint8_t frame_pos = 0;

unsigned long lastByteTime = 0;
const uint16_t FRAME_TIMEOUT = 8;   // ms gap = end of frame

// -------------------------------------------------

void printFrame()
{
  if (frame_pos == 0) return;

  Serial.print("FRAME: ");

  for (uint8_t i = 0; i < frame_pos; i++)
  {
    if (frame[i] < 16) Serial.print('0');
    Serial.print(frame[i], HEX);
    Serial.print(' ');
  }

  Serial.print("\r\n");   // proper line ending
}

// -------------------------------------------------

void setup()
{
  Serial.begin(115200);
  bus.begin(BUS_BAUD);

  Serial.println("Fastnet RAW listener ready");
}

// -------------------------------------------------

void loop()
{
  while (bus.available())
  {
    uint8_t c = bus.read();
    lastByteTime = millis();

    // New frame starts at 0xFF
    if (c == 0xFF && frame_pos > 0)
    {
      printFrame();
      frame_pos = 0;
    }

    if (frame_pos < MAX_FRAME)
      frame[frame_pos++] = c;
  }

  // timeout = end of frame
  if (frame_pos > 0 && (millis() - lastByteTime) > FRAME_TIMEOUT)
  {
    printFrame();
    frame_pos = 0;
  }
}
