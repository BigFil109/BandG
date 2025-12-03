/*
   Auto NMEA Baud Scanner for Arduino Mega
   Scans Serial1 at multiple baud rates and looks for '$'
*/

uint32_t baudRates[] = {
  4800, 9600, 19200, 38400, 57600, 115200
};
const int NUM_BAUDS = sizeof(baudRates) / sizeof(baudRates[0]);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting NMEA baud scanner…");
}

void loop() {
  for (int i = 0; i < NUM_BAUDS; i++) {

    uint32_t br = baudRates[i];
    Serial1.begin(br);
    delay(300);  // allow Serial1 to stabilise

    Serial.print("Scanning baud: ");
    Serial.println(br);

    unsigned long start = millis();
    while (millis() - start < 2000) { // scan this baud for 2 seconds
      if (Serial1.available()) {
        char c = Serial1.read();

        // We found an NMEA-style start byte!
        if (c == '$') {
          Serial.println("\n==============================");
          Serial.print("FOUND NMEA at baud: ");
          Serial.println(br);
          Serial.println("==============================");

          // Print full sentence
          Serial.print("Sentence: $");
          while (millis() - start < 3000) {
            if (Serial1.available()) {
              char nc = Serial1.read();
              Serial.print(nc);
              if (nc == '\n') break;
            }
          }

          // Lock to this baud permanently
          while (true) {
            if (Serial1.available()) {
              Serial.write(Serial1.read());
            }
          }
        }
      }
    }

    Serial.println("No '$' found.\n");
  }

  Serial.println("Finished scan. Restarting…\n");
}
