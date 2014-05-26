#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define GPS_RX_PIN 10
#define GPS_TX_PIN 11
#define LED_PIN 13

/**
  * Set fldigi to 45.5 baud, 2 stop bits, 7 bit ascii
  */
#define AUDIO_PIN 12

#define MARK_FREQ 1585
#define SPACE_FREQ 1415

// Baud rate in millis.  (1 second / 45.45) * 1_000_000
#define BAUD_RATE 22002

// Analog
#define TEMPERATURE_PIN 0

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

char datastring[80];

void flashLed()
{
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}
// Returns microseconds
int frequencyToDelay(int frequency)
{
  return (1000000/frequency) / 2;
}

void rtty_txbyte(char c) {
  int i;
  
  rtty_txbit(0);
  
  for (i=0;i<7;i++) {
    if (c & 1) {
      rtty_txbit(1);
    } else {
      rtty_txbit(0);
    }
    
    c = c >> 1;
  }
  
  // 2 stop bits
  rtty_txbit(1);
  rtty_txbit(1);
}

void rtty_txbit(int bit)
{
  if (bit) {
    playFrequency(MARK_FREQ);
  } else {
    playFrequency(SPACE_FREQ);
  }
}

void playFrequency(int frequency)
{
  int delayLength = frequencyToDelay(frequency);
  int cycles = 22002 / (delayLength) / 2;

  //long targetTime = millis() + BAUD_RATE;
  //while(millis() < targetTime) {
  for(int i = 0; i < cycles; i++) {
    digitalWrite(AUDIO_PIN, HIGH);
    delayMicroseconds(delayLength);
    digitalWrite(AUDIO_PIN, LOW);
    delayMicroseconds(delayLength);
  }
}

void tx(char *string) {
  
  char c;
  
  c = *string++;
  
  while (c != '\0') {
    rtty_txbyte(c);
    c = *string++;
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(AUDIO_PIN, OUTPUT);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  flashLed();
}

float getTemperature()
{
  int tempReading = analogRead(TEMPERATURE_PIN);
  float voltage = (tempReading * 3.3) / 1024.0;
  return (voltage - 0.5) * 100;
}

void loop()
{
  long lat, lon, altitude;
  unsigned long fix_age, date, time, speed, course;
  unsigned short satellites;
  
  gpsSerial.begin(9600);
  
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

      float temperature = getTemperature();
      
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
        Serial.println(" NO FIX!");
      } else {
        Serial.print(" Fix age: ");
        Serial.println(fix_age);
      }
 
      Serial.print("Temperature: ");
      Serial.println(temperature);
      
      gpsSerial.end();
      
      sprintf(
        datastring,
        "$$$ZL3ML,%ld,%ld,%ld,%d\n",
        lat, lon, altitude, temperature
      );
     
      Serial.print("datastring = ");
      Serial.println(datastring);
      // TODO TX
      tx(datastring);
    }
  }
}
