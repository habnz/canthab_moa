#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define GPS_RX_PIN 10
#define GPS_TX_PIN 11
#define LED_PIN 13

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

void flashLed()
{
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  flashLed();
}

void loop()
{
  long lat, lon, altitude;
  unsigned long fix_age, date, time, speed, course;
  unsigned short satellites;
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    // Serial.print(c);
    if(gps.encode(c)) {
      flashLed();
      gps.get_position(&lat, &lon, &fix_age);
      gps.get_datetime(&date, &time, &fix_age);
      speed = gps.speed();
      course = gps.course();
      satellites = gps.satellites();
      Serial.println("=========");
      Serial.print("Lat: ");
      Serial.print(lat);
      Serial.print(" Lon: ");
      Serial.println(lon);
      Serial.print("Date: ");
      Serial.print(date);
      Serial.print(" Time: ");
      Serial.println(time);
      Serial.print("Speed: ");
      Serial.print(speed);
      Serial.print(" Course: ");
      Serial.print(course);
      Serial.print(" Altitude: ");
      Serial.println(altitude);
      Serial.print("Satellites: ");
      Serial.print(satellites);
      if (fix_age == TinyGPS::GPS_INVALID_AGE) {
        Serial.println("NO FIX!");
      } else {
        Serial.print(" Fix age: ");
        Serial.println(fix_age);
      }
    }
  }
}
