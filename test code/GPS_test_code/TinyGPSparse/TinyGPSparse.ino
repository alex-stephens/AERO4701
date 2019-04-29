#include <TinyGPS.h>

TinyGPS gps;
char s[80];
long lat, lon;
unsigned long fix_age, time, date;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available() > 0) {
          // read the incoming byte:
          char c = Serial1.read();
          
          //Serial.write(c);
          
           
          if (gps.encode(c)) {
            Serial.println("Updated");
            sprintf(s, "Lat: %ld Lon: %ld Time: %ld", lat, lon, time);
          }
          
          gps.get_position(&lat, &lon, &fix_age);
          gps.get_datetime(&date, &time, &fix_age);

          if (fix_age == TinyGPS::GPS_INVALID_AGE)
            Serial.println("No fix detected");
          else if (fix_age > 5000)
            Serial.println("Warning: possible stale data!");
          else
            Serial.println(s);
  }
}
