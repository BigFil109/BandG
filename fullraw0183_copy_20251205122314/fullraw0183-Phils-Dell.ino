// ============================================================
// NMEA-0183 RS422 / RS485 Listener for Arduino Mega
// Auto-formats incoming sentences and prints readable output
// Serial1 RX1 = Pin 19
// Serial monitor: 115200 baud
// NMEA device:   4800 baud, 7O1
// ============================================================

String nmeaLine = "";

bool verifyChecksum(const String &line) {
  int star = line.indexOf('*');
  if (star < 0) return false;

  uint8_t calc = 0;
  for (int i = 1; i < star; i++)
    calc ^= line[i];

  String hex = line.substring(star + 1, star + 3);
  uint8_t sent = strtoul(hex.c_str(), NULL, 16);

  return calc == sent;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("=== NMEA-0183 Listener @ 4800 7O1 ===");
  Serial.println("Listening on Serial1 (RX1 / pin 19)");
  Serial.println("------------------------------------");

  Serial1.begin(4800);
}

void loop() {

  while (Serial1.available()) {
 
    char c = Serial1.read();
    String x = "X:" + c;
    x += ":";
    if (x.length()>3) { Serial.println(x);}
    if (c == '$') {
      nmeaLine = "$";
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (nmeaLine.startsWith("$") && nmeaLine.length() > 6) {

        // ---- Show raw line ----
        Serial.print("RAW:   ");
        Serial.println(nmeaLine);

        // ---- Checksum ----
        bool ok = verifyChecksum(nmeaLine);
        Serial.print("CHK:   ");
        Serial.println(ok ? "OK" : "BAD");

        // ---- Sentence ID ----
        if (nmeaLine.length() >= 6) {
          String talker = nmeaLine.substring(1, 3);
          String id     = nmeaLine.substring(3, 6);

          Serial.print("TYPE:  ");
          Serial.print(talker);
          Serial.print(" / ");
          Serial.println(id);
        }

        Serial.println();
      }
      nmeaLine = "";
    } else {
      nmeaLine += c;
    }
  }
}
