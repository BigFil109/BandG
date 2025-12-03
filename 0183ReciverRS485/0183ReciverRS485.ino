// =====================
// NMEA Reader for Arduino Mega (Pin 19 - RX1)
// =====================

String nmea = "";

void setup() {
  Serial.begin(115200);
  Serial1.begin(4800);       // RX1 on pin 19 for NMEA input
  
  Serial.println("NMEA0183 Reader Started...");
}

void loop() {

  // Read from NMEA input on Serial1
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      processSentence(nmea);
      nmea = "";
    } else if (c != '\r') {
      nmea += c;
    }
  }
}

// --------------------
// Parse NMEA sentences
// --------------------
void processSentence(String s) {
  if (!s.startsWith("$")) return;

  Serial.print("RAW: ");
  Serial.println(s);

  // Split by comma
  String parts[20];
  int count = split(s, ',', parts, 20);

  // --------------------
  // MWV (wind angle/speed)
  // $WIMWV,045,R,12.3,N,A*checksum
  // --------------------
  if (s.startsWith("$WIMWV")) {
    float awa = parts[1].toFloat();
    float aws = parts[3].toFloat();
    Serial.print("AWA: "); Serial.print(awa);
    Serial.print("  AWS: "); Serial.println(aws);
  }

  // --------------------
  // DPT (Depth)
  // $IIDPT,5.4,0.0,*hh
  // --------------------
  else if (s.startsWith("$IIDPT")) {
    float depth = parts[1].toFloat();
    Serial.print("Depth: "); Serial.println(depth);
  }

  // --------------------
  // VTG (Speed Over Ground)
  // $IIVTG,,T,,M,2.15,N,3.98,K*hh
  // --------------------
  else if (s.startsWith("$IIVTG")) {
    float sog = parts[5].toFloat();
    Serial.print("SOG: "); Serial.println(sog);
  }

  // --------------------
  // XDR (Voltage + Temp)
  // Voltage example: $IIXDR,U,12.5,V,MAIN*hh
  // Temp example:    $IIXDR,C,18.2,C,AIRT*hh
  // --------------------
  else if (s.startsWith("$IIXDR")) {
    String type = parts[1];

    if (type == "U") {
      float voltage = parts[2].toFloat();
      Serial.print("Voltage: "); Serial.println(voltage);
    }

    if (type == "C") {
      float tempC = parts[2].toFloat();
      Serial.print("Water Temp: "); Serial.println(tempC);
    }
  }

  // --------------------
  // VMG (custom test sentence)
  // $IIVMG,2.1,N*hh
  // --------------------
  else if (s.startsWith("$IIVMG")) {
    float vmg = parts[1].toFloat();
    Serial.print("VMG: "); Serial.println(vmg);
  }
}

// --------------------
int split(String s, char delimiter, String arr[], int maxParts) {
  int count = 0;
  int start = 0;

  for (int i = 0; i < s.length(); i++) {
    if (s[i] == delimiter) {
      arr[count++] = s.substring(start, i);
      start = i + 1;
      if (count >= maxParts) break;
    }
  }
  arr[count++] = s.substring(start);
  return count;
}
