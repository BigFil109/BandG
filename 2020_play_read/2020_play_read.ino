#define FASTNET_BAUD 28800
#define RS485_EN 13   // DE + /RE tied together

// Backlight ON frame (broadcast, CPU=0x2F)
uint8_t backlight_on[] = {
  0xFF,   // broadcast
  0x2F,   // CPU addr
  0x02,   // length
  0xC9,   // CM_SET_BACKLIGHT
  0x07,   // header checksum
  0x04,   // level = HIGH
  0x20,   // padding
  0xDC    // data checksum
};

uint8_t shutoff[] = {
  0xFF,   // broadcast
  0x2F,   // CPU addr
  0x00,   // length
  0xC8,   // CM_SET_BACKLIGHT
  0x0A
};

//FF 2F 0 FB D7 
uint8_t who[] = {
  0xFF,   // broadcast
  0x2F,   // CPU addr
  0x00,   // length
  0xFB,   // CM_SET_BACKLIGHT
  0xD7
};

unsigned long last_tx = 0;
unsigned long last_tx_who = 0;

void rs485_send(uint8_t *buf, uint8_t len) {
  digitalWrite(RS485_EN, HIGH);        // TX enable
  delayMicroseconds(50);

  Serial1.write(buf, len);
  Serial1.flush();                     // wait for stop bits

  delayMicroseconds(50);
  digitalWrite(RS485_EN, LOW);         // RX enable
}

void setup() {
  Serial.begin(115200);
  Serial.println("Fastnet RX + TX test");

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);         // RX mode

  Serial1.begin(FASTNET_BAUD, SERIAL_8O2);
}

void loop() {
  // ---------- RX ----------
  if (Serial1.available()) {
    Serial.print("RX: ");
    while (Serial1.available()) {
      uint8_t b = Serial1.read();
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // ---------- TX ----------
  if (millis() - last_tx > 1500) {
    Serial.println("TX: Backlight ON");
    rs485_send(backlight_on, sizeof(backlight_on));
    last_tx = millis();
  }

  if (millis() - last_tx_who > 5000) {
    Serial.println("TX: who");
    rs485_send(who, sizeof(who));
    last_tx_who = millis();
  }
}
