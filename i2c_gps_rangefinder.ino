// Copyright 2018 Stefan Schmidt

#include "WireMW.h"
#include "ndebug.h"
#include "HardwareSerial.h"
#include "gps.h"
#include "rangefinder.h"

#ifdef SONAR
extern void sonarInit();
extern bool sonarUpdate();
extern int16_t sonarReadDistance();
#endif


I2C_REGISTERS   i2c_dataset;
I2C_REGISTERS   gps_dataset;
uint32_t        lastReadGPS = 0;
uint8_t cmd = 0;


void gpsSerialInit();
bool GPS_NMEA_newFrame(char c);

void _traceGPS()
{
#ifdef DEBUG_SERIAL
  debugSerial.print(i2c_dataset.status.gps3dfix==1 ? "3D=" : "S=");
  debugSerial.print(i2c_dataset.status.numsats);
  debugSerial.print(", p=");
  debugSerial.print(i2c_dataset.lat);
  debugSerial.print("/");
  debugSerial.print(i2c_dataset.lon);
  debugSerial.print(", a=");
  debugSerial.println(i2c_dataset.altitude);
#endif
}

void initData()
{
  memset(&i2c_dataset, 0, sizeof(i2c_dataset));
  memset(&gps_dataset, 0, sizeof(gps_dataset));
  /*
  i2c_dataset.altitude = 100;
  i2c_dataset.ground_speed = 100;
  i2c_dataset.ground_course = 0;
  i2c_dataset.gps_loc.lat = 490000000;
  i2c_dataset.gps_loc.lon = 110000000;
  i2c_dataset.status.new_data = 1;
  i2c_dataset.status.gps3dfix = 1;
  i2c_dataset.status.numsats = 6;
  i2c_dataset.hdop = 5;
  */
}


void blink_update()
{
  static uint32_t nextTime = 0;
  uint32_t now = millis();
  static byte blinkStatus = 0;

  if(now >= nextTime)
  {
//    i2c_dataset.status.numsats = 4;
    bool hasFix = i2c_dataset.status.numsats >= 4;
    if(blinkStatus == 0)
    {
      digitalWrite(13, LOW);   
      nextTime = now + 2000;

      if(hasFix)
      {
        blinkStatus = 2*i2c_dataset.status.numsats;
      }
      else
      {
        blinkStatus = 255;
      }
      _traceGPS();
    }
    else if(blinkStatus == 255)
    {
      digitalWrite(13, HIGH);   
      nextTime = now + 100;
      blinkStatus = 0;
    }
    else
    {
      digitalWrite(13, (blinkStatus%2==0) ? HIGH : LOW);   // set the LED off
      blinkStatus--;
      nextTime = now + 400;  
    }
  }
  //_D(blinkStatus);

    /*
  if(blinkStatus == 0)
  {
    if(i2c_dataset.status.numsats >= 4)
      blinkStatus = 2*i2c_dataset.status.numsats;
    else 
      blinkStatus = 2;

      digitalWrite(13, LOW);   // set the LED off
      nextTime = now + 1000;
  }

  if(now >= nextTime)
  {
    digitalWrite(13, (blinkStatus%2==0) ? HIGH : LOW);   // set the LED off
    blinkStatus--;
    nextTime = now + 1000;  
  }*/
}


void setup() 
{
  _debugInit();
  _D("+setup");
  initData();

#ifdef SONAR
  sonarInit();
  _D("sonarInit");
#endif

#ifdef GPS
  gpsSerialInit();
  _D("gpsInit");
#endif

  Wire2.begin(I2C_GPS_ADDRESS);
  Wire2.onReceive(receiveEvent);
  Wire2.onRequest(requestEvent);
  _D("-setup");
}


void loop() 
{
#ifdef GPS
  while (Serial.available()) 
  {
    if (GPS_NMEA_newFrame(Serial.read())) 
    {
      // We have a valid GGA frame and we have lat and lon in GPS_read_lat and GPS_read_lon, apply moving average filter
      // this is a little bit tricky since the 1e7/deg precision easily overflow a long, so we apply the filter to the fractions
      // only, and strip the full degrees part. This means that we have to disable the filter if we are very close to a degree line

      i2c_dataset = gps_dataset;
      memset(&gps_dataset, 0, sizeof(gps_dataset));
      lastReadGPS = millis();
//      _traceGPS();
    } 
  } 
#endif // GPS

  blink_update();

#ifdef SONAR
  if(sonarUpdate())
  {
    int16_t v = sonarReadDistance();
    _D(v);
  }
#endif

  if(millis() > lastReadGPS + 1200)
  {
    i2c_dataset.status.gps2dfix = 0;
    i2c_dataset.status.gps3dfix = 0;
    i2c_dataset.status.numsats = 0;
    i2c_dataset.status.new_data = 1;
    lastReadGPS = millis();
  }
}


void receiveEvent(int howMany) 
{
  if(howMany == 1) {
    if(Wire2.available()) {
      cmd = Wire2.read();  
    }  
  }
  else {
    while(Wire2.available()) {
      Wire2.read();
    }
  }
}


void requestEvent() 
{
  switch(cmd)
  {
    case HCSR04_I2C_REGISTRY_STATUS: {
      static uint8_t c = 0;
      int16_t val = sonarReadDistance();
      uint8_t b[] = { c++, val>>8, val&0xFF };
      Wire2.write(b, sizeof(b));
    } break;

    case I2C_GPS_STATUS_00: {
      Wire2.write((uint8_t*)&i2c_dataset.status, 1);
    } break;

    case I2C_GPS_LOCATION_LAT: {
      Wire2.write((uint8_t*)&i2c_dataset.lat, 4);
    } break;

    case I2C_GPS_LOCATION_LON: {
      Wire2.write((uint8_t*)&i2c_dataset.lon, 4);
    } break;

    case I2C_GPS_GROUND_SPEED: {
      Wire2.write((uint8_t*)&i2c_dataset.ground_speed, 2);
    } break;

    case I2C_GPS_GROUND_COURSE: {
      Wire2.write((uint8_t*)&i2c_dataset.ground_course, 2);
    } break;

    case I2C_GPS_ALTITUDE: {
      Wire2.write((uint8_t*)&i2c_dataset.altitude, 2);
    } break;

    default:
    break;
  }
}
