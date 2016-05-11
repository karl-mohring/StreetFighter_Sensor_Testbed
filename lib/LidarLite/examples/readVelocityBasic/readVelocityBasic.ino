#include "LIDARduino1.h"
#include <I2C.h>

LIDAR_Lite_I2C l;

void setup(){
  Serial.begin(115200);
  while (!Serial) ;

  l.begin();
  Serial.println(F("Starting Basic Velocity Reading..."));

  Serial.print(F("Hardware Version: 0x"));
  Serial.println( l.getHWversion() ); // Mine read "17"
  Serial.print(F("Software Version: 0x")); // Mine read "17"
  Serial.println( l.getHWversion() );


}

void loop(){


Serial.print(l.easyVelocity());
Serial.println(F("m/s"));
delay(500);

}
