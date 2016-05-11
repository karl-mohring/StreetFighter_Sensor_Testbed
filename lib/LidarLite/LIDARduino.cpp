/**************************************************************************/
/*!
@file     LIDARduino.cpp
@author   Stuart Feichtinger
@license  MIT (see license.txt)

Arduino Library for the LIDAR-Lite from PulsedLight, Inc.

@section  HISTORY
v0.0.1 - First release

*/
/**************************************************************************/

#include "LIDARduino.h"

void LIDAR_Lite::enablePowerCtrl( uint8_t powerPin ){
  _powerPin = powerPin;
  pinMode(_powerPin, OUTPUT);
  digitalWrite(_powerPin, HIGH);
}

bool LIDAR_Lite::power( bool state ){
  if( _powerPin != 255 ){
    _pwrState = state;
    digitalWrite( _powerPin, _pwrState );
    return 1;
}
  return 0;
}


bool LIDAR_Lite::powerStatus( void ) const{
  return _pwrState;
}

LIDAR_Lite_I2C::LIDAR_Lite_I2C( void ):_I2CAddress( LIDARLite_ADDRESS ){

}

LIDAR_Lite_I2C::LIDAR_Lite_I2C( uint8_t i2cAddr ):_I2CAddress( i2cAddr ){

}

void LIDAR_Lite_I2C::begin( void ){
  I2c.begin();
  delay(100);
  I2c.timeOut(50);

}

int16_t LIDAR_Lite_I2C::getDistance( void ){

  _triggerRead();

  uint8_t readAry[2];
  _readI2C(LIDAR_DIST_READ_HL, 2, readAry);
  int16_t distance = (readAry[0] << 8) | readAry[1];
  return distance;
}

int16_t LIDAR_Lite_I2C::easyDistance( void ){

  return getDistance();
}

/* ==========================================================================================================================================
Get raw velocity readings from sensor and convert to signed int
=============================================================================================================================================*/


int16_t LIDAR_Lite_I2C::getVelocity( void ){
  _triggerRead();
  uint8_t read = _readI2C(LIDAR_VELOCITY_READ);

  return((int)((char)read));
}

int16_t LIDAR_Lite_I2C::easyVelocity( void ){
  enableVelocity( LL_10_MS );
  int velocity = getVelocity();
  disableVelocity();

  return(velocity);
}

void LIDAR_Lite_I2C::enableVelocity( LL_V_RESOLUTION_E e ){
  uint8_t tempReg = ( 1 << e ) | ( 1 << V_ENABLE ); // set Velocity scale factor (e) and enable velocity measurement
  _writeI2C( LIDAR_MODE, tempReg ); //enable velocity mode;

}

void LIDAR_Lite_I2C::disableVelocity( void ){
  uint8_t tempReg = _readI2C( LIDAR_MODE ); //get current Mode settings
  tempReg &= ~( 1 << V_ENABLE ); // clear Velocity enable bit
  _overWriteI2C( LIDAR_MODE, tempReg );


}

uint8_t LIDAR_Lite_I2C::getHWversion( void ) {

  return _readI2C( LIDAR_HW_VER );
}

uint8_t LIDAR_Lite_I2C::getSWversion( void ){

  return _readI2C( LIDAR_SW_VER );
}


uint8_t LIDAR_Lite_I2C::_readI2C( uint8_t regAddress ){
  uint8_t readAry[1];
  uint8_t nackack = 100;
  while(nackack != 0){
    nackack = I2c.read(_I2CAddress, regAddress, 1, readAry ); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return readAry[0]; // Return array for use in other functions
}

void LIDAR_Lite_I2C::_readI2C( uint8_t regAddress, int16_t numBytes, uint8_t destAry[] ){
  uint8_t nackack = 100;
  while(nackack != 0){
    nackack = I2c.read(_I2CAddress, regAddress, numBytes, destAry ); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
}


void LIDAR_Lite_I2C::_writeI2C( uint8_t regAddress, uint8_t value ){

  uint8_t tempReg = _readI2C(regAddress);
  tempReg |= value;
  _overWriteI2C(regAddress, tempReg);

}

void LIDAR_Lite_I2C::_overWriteI2C( uint8_t regAddress, uint8_t value ){
  uint8_t nackack = 100;
  while(nackack != 0){
    nackack = I2c.write(_I2CAddress, regAddress, value ); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

inline void LIDAR_Lite_I2C::_triggerRead( LL_READ_MODE_E e ){
  _writeI2C( LIDAR_COMMAND, (1 << e) );


}

LIDAR_Lite_PWM::LIDAR_Lite_PWM( uint8_t trigPin, uint8_t readPin ):_triggerPin( trigPin ), _readPin( readPin ){
}

void LIDAR_Lite_PWM::begin( void ){
  pinMode( _triggerPin, OUTPUT ) ;
  digitalWrite( _triggerPin, HIGH );
  pinMode( _readPin, INPUT ) ;

}

unsigned long LIDAR_Lite_PWM::getDistance( void ){
  digitalWrite( _triggerPin, LOW );
  unsigned long pulse_width = pulseIn(_readPin, HIGH);
  digitalWrite( _triggerPin, HIGH );

  if(pulse_width != 0){

  return (pulse_width / 10 ); // 10usec = 1 cm of distance for LIDAR-Lite
  }
  return 0;
}
