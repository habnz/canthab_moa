#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <util/crc16.h>

#define GPS_RX_PIN 10
#define GPS_TX_PIN 11
#define LED_PIN 13

/**
  * Set fldigi to 45.5 baud, 2 stop bits, 7 bit ascii
  */
#define AUDIO_PIN 12

//#define MARK_FREQ 1585
#define MARK_FREQ 1649
//#define SPACE_FREQ 1415
#define SPACE_FREQ 1472

// Baud rate in millis.  (1 second / 45.45) * 1_000_000
#define BAUD_RATE 22002

// Analog
#define TEMPERATURE_PIN 0

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

char datastring[80];
char temperaturestring[6];
char checksumstring[6];
char latitudestring[11];
char longitudestring[11];
int sentence_id = 0;
unsigned int checksum;

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
  
  /**
   * Begin each sentence with 4 null bytes to allow the
   * receivers to sync.
  **/
  rtty_txbyte(0x00);
  rtty_txbyte(0x00);
  rtty_txbyte(0x00);
  rtty_txbyte(0x00);

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

uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;
 
  crc = 0xFFFF;
 
  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
 
  return crc;
}

void loop()
{
  float lat, lon;
  long altitude;
  unsigned long fix_age, speed, course;
  unsigned short satellites;
  int year;
  byte month, day, hours, minutes, seconds, hundredths;
  
  gpsSerial.begin(9600);
  
  while (gpsSerial.available()) {

    char c = gpsSerial.read();
    // Serial.print(c);
    if(gps.encode(c)) {
      flashLed();
      gps.f_get_position(&lat, &lon, &fix_age);
      gps.crack_datetime(&year, &month, &day,
        &hours, &minutes, &seconds, &hundredths, &fix_age);
      speed = gps.speed();
      course = gps.course();
      satellites = gps.satellites();

      float temperature = getTemperature();
      
      Serial.println("=========");
      Serial.print("Lat: ");
      Serial.print(lat);
      Serial.print(" Lon: ");
      Serial.println(lon);
      Serial.print(" Time: ");
      Serial.print(hours); Serial.print(":");
      Serial.print(minutes); Serial.print(":");
      Serial.println(seconds);
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

      // FIXME - Double check widths
      dtostrf(temperature, 5, 2, temperaturestring);
      dtostrf(lat, 10, 6, latitudestring);
      dtostrf(lon, 10, 6, longitudestring);
      
      sprintf(
        datastring,
        "$$ZL3ML,%d,%02d:%02d:%02d,%s,%s,%ld,%lu,%lu,%s",
        sentence_id++, hours, minutes, seconds,
        latitudestring, longitudestring,
        altitude, speed, course, temperaturestring
      );
      checksum = gps_CRC16_checksum(datastring);
      sprintf(checksumstring, "*%04X\n", checksum);
      strcat(datastring, checksumstring);
     
      Serial.print("datastring = ");
      Serial.println(datastring);
      // TODO TX
      tx(datastring);
    }
  }
}
