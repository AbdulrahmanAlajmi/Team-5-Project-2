#include <Adafruit_GPS.h>

Adafruit_GPS GPS(&Wire);

#define GPSECHO false

uint32_t timer = millis();


void setup()
{
  
  Serial.begin(9600);
  Serial.println("Searching for Satellites");

  GPS.begin(0x10);  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  GPS.println(PMTK_Q_RELEASE);
}

void loop() 
{
 
  char c = GPS.read();
 
  if (GPSECHO)
    if (c) Serial.print(c);
 
  if (GPS.newNMEAreceived()) {
    
    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
   
    if (GPS.fix) {
      
    Serial.print("Latitude: ");  Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
    Serial.print("Longitude: ");  Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Number of Locked Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
