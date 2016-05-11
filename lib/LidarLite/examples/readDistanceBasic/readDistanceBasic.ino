#include "LIDARduino.h"
#include <I2C.h>

//LIDAR_Lite_I2C lidar;
LIDAR_Lite_PWM lidar(13, 10);

void setup(){
  Serial.begin(115200);
  while (!Serial) ; //only needed for Arduino Leonardo (Atmega 32u4)

  lidar.begin();
  Serial.println(F("Starting Easy Distance Reading..."));  
}

void loop(){


  Serial.print(lidar.getDistance());
  Serial.println(F("cm"));
  delay(500);

}
