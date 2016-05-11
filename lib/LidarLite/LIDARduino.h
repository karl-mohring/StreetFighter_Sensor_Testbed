/**************************************************************************/
/*!
@file     LIDARduino.h
@author   Stuart Feichtinger
@license  MIT (see license.txt)

Arduino Library for the LIDAR-Lite from PulsedLight, Inc.

@section  HISTORY
v0.0.1 - First release
*/
/**************************************************************************/

#pragma once

#include "Arduino.h"
#include "LIDARLite_registers.h"
#include <I2C/I2C.h>

typedef enum{
  LL_DC = USE_DC, //Correct for DC (More accurate)
  LL_NO_DC = NO_DC //Don't correct (Faster)

} LL_READ_MODE_E;


typedef enum{
  LL_10_MS = 1, // sets the velocity measurement separation to 10msec resulting in a velocity calibration in meters/sec
  LL_100_MS = 0 // sets the  measurement separation to 100msec.

} LL_V_RESOLUTION_E;

class LIDAR_Lite{
  public:
    virtual void
      begin()=0;

    void
      enablePowerCtrl( uint8_t powerPin ) ; // allow for multiple sensors on one I2C bus or PWM pin.
    bool
      power( bool state ) ,
      powerStatus( void ) const ;

      /*Example
        #define NUM_SENSORS 4  //Number of LIDAR_Lites to read.
        LIDAR_Lite_I2C sensors[ NUM_SENSORS ]; // Could be LIDAR_Lite_PWM also
        uint8_t sensorPwrPins[] = {6, 7, 8, 9}; // Hook up each arduino pin to each PWR_EN wire of LIDAR Lites (PWR_EN wire adjacent to Red 5V wire ) using 1K - 10K resistor in series.

        for(int i = 0; i < NUM_SENSORS; i++){ // disable all sensors
          sensors[i].begin();
          sensors[i].enablePowerCtrl( sensorPwrPins[i] );
          sensors[i].power(0);
        }

        for(int i = 0; i < NUM_SENSORS; i++){ // Read from each sensor individually
          sensors[i].power(1); //enable individual sensor

          Serial.print(F( Sensor  ));
          Serial.print(i);
          Serial.print(F(  Distance Reading:  ));
          Serial.print(sensors[i].getDistance());
          Serial.println(F( cm ));

          sensors[i].power(0); //disable individual sensor
        }
      */


  private:
      uint8_t
        _powerPin = 255;

      bool
        _pwrState = 1;

};

class LIDAR_Lite_I2C : public LIDAR_Lite{


  /***********PUBLIC*************************/
public:

  LIDAR_Lite_I2C(),
  LIDAR_Lite_I2C( uint8_t i2cAddr );

  virtual void
    begin( void ) ;

  int16_t
    getDistance( void ) , // return distance in cm.
    getVelocity( void ) ,
    easyDistance( void ) ,
    easyVelocity( void );


  uint8_t
    getHWversion( void ) , // laser units revisions begin with 0x01 (short range ), 0x20 for long range lasers, and Led units begin with 0x40
    getSWversion( void ) ; // laser units revisions begin with 0x01 (short range ), 0x20 for long range lasers, and Led units begin with 0x40

  void
    enableVelocity( LL_V_RESOLUTION_E e) ,
    disableVelocity( void ) ;


  /***********PRIVATE*************************/
private:

  uint8_t
    _readI2C( uint8_t regAddress ); //read single byte

  void
    _readI2C( uint8_t regAddress, int16_t numBytes, uint8_t destAry[] ), //read array of bytes
    _writeI2C( uint8_t regAddress, uint8_t value ),
    _overWriteI2C( uint8_t regAddress, uint8_t value );

  inline void
    _triggerRead( LL_READ_MODE_E e = LL_DC ) ;


  uint8_t const
    _I2CAddress ;



};

class LIDAR_Lite_PWM: public LIDAR_Lite{

public:
  LIDAR_Lite_PWM( uint8_t triggerPin, uint8_t readPin ) ;

  virtual void
    begin();

  unsigned long
    getDistance( void );

private:
  uint8_t const
    _triggerPin ,
    _readPin ;



};
