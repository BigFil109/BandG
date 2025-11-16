/*
   NMEA 0183 Wind Generator
   Sends MWV (wind direction & apparent wind speed)
   - Direction increases 0 → 359 degrees
   - Speed increases 0 → 20 knots then loops
   - Output rate: 5 Hz
*/

int windAngle = 0;
float windSpeed = 0.0;

unsigned long lastSend = 0;

String nmeaChecksum(String sentence) {
    byte checksum = 0;
    for (int i = 1; i < sentence.length(); i++) {
        checksum ^= sentence[i];
    }
    char hex[3];
    sprintf(hex, "%02X", checksum);
    return String(hex);
}

void setup() {
    Serial.begin(4800);   // Standard NMEA 0183 rate
    delay(1000);
}

void loop() {
    if (millis() - lastSend >= 200) {   // 5 messages per second
        lastSend = millis();

        // ---- Update simulated values ----
        windAngle++;
        if (windAngle >= 360) windAngle = 0;

        windSpeed += 0.2;
        if (windSpeed > 20.0) windSpeed = 0.0;

        // ---- Build MWV sentence ----
        String sentence = "$WIMWV,";
        sentence += String(windAngle);
        sentence += ",R,";   // Apparent (Relative) wind
        sentence += String(windSpeed, 1);
        sentence += ",N,A";

        // ---- Add checksum ----
        String fullSentence = sentence + "*" + nmeaChecksum(sentence);

        // ---- Send out serial ----
        Serial.println(fullSentence);
    }
}
