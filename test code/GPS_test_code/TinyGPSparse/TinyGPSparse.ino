#include <TinyGPS.h>

TinyGPS gps;
char s[80];
long lat, lon;
unsigned long fix_age, time, date;
unsigned char led;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(9600);
  pinMode(13, OUTPUT);
  led = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available() > 0) {
          // read the incoming byte:
          char c = Serial2.read();
          led = 1 - led;
          if (led==0) {
            digitalWrite(13, HIGH);
          } else {
            digitalWrite(13, LOW);
          }
          //Serial.write(c);
          
           
          if (gps.encode(c)) {
            Serial.println("Updated");
            sprintf(s, "Lat: %ld Lon: %ld Time: %ld", lat, lon, time);
            Serial.println(s);
            Serial1.println(s);
          }
          
          gps.get_position(&lat, &lon, &fix_age);
          gps.get_datetime(&date, &time, &fix_age);

          if (fix_age == TinyGPS::GPS_INVALID_AGE) {
            Serial.println("No fix detected");
            Serial1.println("NFD");
          }
          else if (fix_age > 5000) {
            Serial.println("Warning: possible stale data!");
            Serial1.println("Stale");
          }
          else {
            
          }
  }
}
