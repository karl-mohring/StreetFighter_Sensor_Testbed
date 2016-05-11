#include <StraightBuffer.h>
#include "Arduino.h"

#define BUFFER_SIZE 20
StraightBuffer buffer(BUFFER_SIZE);

String testString = "test";

//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

void setup(){
	Serial.begin(57600);
	Serial.println("Buffer test");
	
	for (int i = 0; i < 10; i++){

		buffer.write('b');
	}
}

void loop(){
	Serial.println("String: " + testString);

	testString.getBytes(buffer.getBufferAddress(), BUFFER_SIZE , 0);
	buffer.setWritePosition(testString.length());

	Serial.print("Buffer: ");
	Serial.write(buffer.getBufferAddress(), BUFFER_SIZE);
	Serial.println();


	delay(1000);

}