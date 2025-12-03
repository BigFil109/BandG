/*
   NMEA 0183 Mega Bench Tester Generator
   Generates:
     - MWV (Apparent Wind)
     - MWV (True Wind)
     - DBT (Depth)
     - VTG (SOG + COG)
     - VHW (STW)
     - RMB (Bearing & Distance)
     - ZDA (Time & Date)

   Output: Serial TX0 (Pin 1), 4800 baud
*/

unsigned long lastFast = 0;
unsigned long lastZDA = 0;

// --- Simulated values ---
float awa = 0;               
float aws = 0.0;

int twa = 0;               
float tws = 0.0;

float depth_m = 1.0;

float sog = 0.0;
float stw = 0.0;

float bearing = 0.0;
float distance_nm = 0.1;

float vmg = 0.0;          // Velocity Made Good

// ---- Time Simulation ----
// Starts at 12:00:00 UTC on 2025-01-01
int hour_utc   = 12;
int minute_utc = 0;
int second_utc = 0;
int day_utc    = 1;
int month_utc  = 1;
int year_utc   = 2025;

// ---------------- CHECKSUM ----------------
String nmeaChecksum(String sentence) {
    byte checksum = 0;
    for (int i = 1; i < sentence.length(); i++)
        checksum ^= sentence[i];

    char hex[3];
    sprintf(hex, "%02X", checksum);
    return String(hex);
}

// -------------- SEND NMEA -----------------
void sendNMEA(String s) {
    Serial.println(s + "*" + nmeaChecksum(s));
}

// -------------- TIME UPDATE ---------------
void updateTime() {
    second_utc++;
    if (second_utc >= 60) {
        second_utc = 0;
        minute_utc++;
        if (minute_utc >= 60) {
            minute_utc = 0;
            hour_utc++;
            if (hour_utc >= 24) {
                hour_utc = 0;
                day_utc++;

                // Very simple month rollover (good for testing)
                if (day_utc > 30) {
                    day_utc = 1;
                    month_utc++;
                    if (month_utc > 12) {
                        month_utc = 1;
                        year_utc++;
                    }
                }
            }
        }
    }
}

// ---------------- SETUP -------------------
void setup() {
    Serial.begin(4800);
    delay(500);
}
  
// ---------------- MAIN LOOP --------------
void loop() {

    unsigned long now = millis();

    // ---------------------------------------------------
    // 1 Hz TIME OUTPUT (ZDA)
    // ---------------------------------------------------
    if (now - lastZDA >= 1000) {
        lastZDA = now;

        updateTime();

        char buf[60];
        sprintf(buf,
                "$IIZDA,%02d%02d%02d.00,%02d,%02d,%04d,,",
                hour_utc, minute_utc, second_utc,
                day_utc, month_utc, year_utc);

        sendNMEA(String(buf));
    }

    // ---------------------------------------------------
    // 5 Hz FAST DATA (MWV, DBT, VTG, VHW, RMB)
    // ---------------------------------------------------
    if (now - lastFast >= 200) {
        lastFast = now;

        // --- Simulated values ---
        awa = (awa + 3.0);
        if(awa > 360.0) {
            awa = 0.0;
        }
  
        aws += 0.2; if (aws > 20) aws = 0;

        twa = (twa + 2) % 360;
        tws += 0.15; if (tws > 20) tws = 0;

        depth_m += 0.05; if (depth_m > 50) depth_m = 1;

        sog += 0.1; if (sog > 12) sog = 0;
        stw += 0.08; if (stw > 12) stw = 0;

        bearing += 1; if (bearing >= 360) bearing = 0;
        distance_nm -= 0.01; if (distance_nm < 0.1) distance_nm = 5.0;

        vmg += 0.2; if (vmg > 20) vmg = 0;

        // ------ MWV Apparent ------
        sendNMEA("$WIMWV," +
                 String(awa) + ",R," +
                 String(aws,1) + ",N,A");

        // ------ MWV True ------
        sendNMEA("$WIMWV," +
                 String(twa) + ",T," +
                 String(tws,1) + ",N,A");

        // ------ DBT ------
        float depth_f = depth_m * 3.28084;
        sendNMEA("$IIDBT," +
                 String(depth_f,1) + ",f," +
                 String(depth_m,1) + ",M," +
                 String(depth_m,1) + ",M");

        // ------ VTG ------
        sendNMEA("$IIVTG," +
                 String(bearing,1) + ",T," +
                 ",M," +
                 String(sog,2) + ",N," +
                 ",K");

        // ------ VHW ------
        sendNMEA("$IIVHW,,T,,M," +
                 String(stw,2) + ",N," +
                 String(stw * 1.852,2) + ",K");

        // ------ RMB ------
        sendNMEA("$IIRMB,A,1.0,,START,END,,,,," +
                 String(distance_nm,2) + ",N," +
                 String(bearing,1) + ",T,,,");
         // ------ VMG ------
        sendNMEA("$IIVMG," +
                 String(vmg,2) + ",N," +  // Use 'N' for knots
                 "A");                     // 'A' for valid/active


    }
}
