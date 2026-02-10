#include <SoftwareSerial.h>

#define BUS_RX 2
#define BUS_BAUD 11000

SoftwareSerial bus(BUS_RX, -1);

#define MAX_FRAME 128

uint8_t frame[MAX_FRAME];
uint8_t framePos = 0;
int expectedLen = -1;

void setup()
{
  Serial.begin(115200);
  bus.begin(BUS_BAUD);

  Serial.println("FASTNET QUAD ANALYSER STARTED");
}

void printHex(uint8_t b)
{
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

void decodeFrame(uint8_t *f, int len)
{
  Serial.print("FRAME: ");

  for (int i=0;i<len;i++)
  {
    printHex(f[i]);
    Serial.print(' ');
  }
  Serial.println();

  // scan inside frame for C4 messages
  for (int i=0;i<len-3;i++)
  {
    if (f[i] == 0x47 && f[i+1] == 0xC4)
    {
     // Serial.println(">>> DEPTH MESSAGE FOUND");

      uint8_t d1 = f[i+4];
      uint8_t d2 = f[i+5];

      uint16_t rawDepth = (d2 << 8) | d1;

     // Serial.print("RAW DEPTH = ");
    //  Serial.println(rawDepth);

      // try common scalings
     // Serial.print("Depth /10 = ");
     // Serial.println(rawDepth / 10.0);

     // Serial.print("Depth /100 = ");
//Serial.println(rawDepth / 100.0);
    }
  }
}

void loop()
{
  while (bus.available())
  {
    uint8_t b = bus.read();

    // start frame
    if (b == 0xFF && framePos == 0)
    {
      frame[framePos++] = b;
      expectedLen = -1;
      continue;
    }

    if (framePos > 0)
    {
      frame[framePos++] = b;

      // length byte detection
      if (framePos == 3)
      {
        expectedLen = frame[2];   // suspected length position
      }

      // frame complete?
      if (expectedLen > 0 && framePos >= expectedLen+3)
      {
        decodeFrame(frame, framePos);
        framePos = 0;
        expectedLen = -1;
      }

      if (framePos >= MAX_FRAME)
      {
        framePos = 0;
      }
    }
  }
}
