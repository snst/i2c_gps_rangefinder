#ifndef GPS_H_
#define GPS_H_

#define I2C_GPS_ADDRESS    0x20
#define I2C_GPS_STATUS      01
#define I2C_GPS_DATA      02
#define I2C_SONAR_DATA    03

#define I2C_GPS_STATUS_00             00 
#define I2C_GPS_REG_VERSION           03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_LOCATION              07    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_LOCATION_LAT          07    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_LOCATION_LON          11    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_GROUND_SPEED          31    // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE              33    // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE         35    // GPS ground course (uint16_t)
#define I2C_GPS_TIME                  39    // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_PACKETS               40
#define I2C_GPS_PING                  41

typedef struct 
{
  uint8_t    new_data:1;
  uint8_t    gps2dfix:1;
  uint8_t    gps3dfix:1;
  uint8_t    reserved:1;
  uint8_t    numsats:4;
} __attribute__ ((__packed__))  STATUS_REGISTER;


typedef struct 
{
    uint32_t              lat;
    uint32_t              lon;
    uint16_t              ground_speed;             // ground speed from gps m/s*100
    uint16_t              ground_course;            // GPS ground course
    int16_t               altitude;                 // gps altitude
    uint16_t              hdop;
    uint32_t              time;
    STATUS_REGISTER       status;                   // 0x00  status register
} __attribute__ ((__packed__))  I2C_REGISTERS;

#endif

