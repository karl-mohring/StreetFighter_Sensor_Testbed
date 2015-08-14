#include <CommandHandler.h>
#include <SimpleTimer.h>
#include <Logging.h>
#include "config.h"


SimpleTimer timer;
int updateTimer = -1;
int stepTimer = -1;

char _commandBuffer[BUFFER_SIZE];
CommandHandler cmd(_commandBuffer, BUFFER_SIZE);

bool transitionsEnabled = false;
byte target;
byte targetPercent;
byte current;
bool ascending = true;

///////////////////////////////////////////////////////////////////////////////
// Main

void setup(){
	Log.Init(LOG_LEVEL_DEBUG, SERIAL_BAUD);

	pinMode(LED_CONTROL_PIN, OUTPUT);

	updateTimer = timer.setInterval(UPDATE_PERIOD, update);

	addCommands();

	Log.Info("LED Test Started\n's' to start the stepping test.\n'T' enables transitions\n'c' to cancel the step test");
}

void loop(){
	readSerial();
	timer.run();
}

///////////////////////////////////////////////////////////////////////////////
// Main

void readSerial(){
	char command;

	while (Serial.available()){
		command = Serial.read();
		cmd.readIn(command);
	}
}

void addCommands(){
	// Levels
	cmd.addCommand('`', lampOff);
	cmd.addCommand('1', lamp10);
	cmd.addCommand('2', lamp20);
	cmd.addCommand('3', lamp30);
	cmd.addCommand('4', lamp40);
	cmd.addCommand('5', lamp50);
	cmd.addCommand('6', lamp60);
	cmd.addCommand('7', lamp70);
	cmd.addCommand('8', lamp80);
	cmd.addCommand('9', lamp90);
	cmd.addCommand('0', lampOn);

	// Transitions
	cmd.addCommand('T', transitionsOn);
	cmd.addCommand('t', transitionsOff);

	// Tests
	cmd.addCommand('s', stepTest);
	cmd.addCommand('c', cancelTest);

	cmd.addCommand('d', disableDebug);
	cmd.addCommand('D', enableDebug);

	cmd.setTerminator('\n');
}

void disableDebug(){
	Log.Init(LOG_LEVEL_INFOS, &Serial);
	Log.Info("Debug disabled");
}

void enableDebug(){
	Log.Init(LOG_LEVEL_DEBUG, &Serial);
	Log.Info("Debug enabled");
}

void cancelTest(){
	if (stepTimer >= 0){
		timer.disable(stepTimer);
		timer.deleteTimer(stepTimer);
		stepTimer = -1;
		Log.Info("Stopping test");
	}
	else{
		Log.Info("Test not running");
	}
}

void stepTest(){
	stepTimer = timer.setInterval(STEP_PERIOD, step);
}

void step(){
	if (ascending){
		if ((100 - targetPercent) > STEP_SIZE){
			setLightLevel(targetPercent + STEP_SIZE);
		}
		else{
			setLightLevel(100);
			ascending = false;
		}
	}

	else{
		if (targetPercent > STEP_SIZE){
			setLightLevel(targetPercent - STEP_SIZE);
		}
		else{
			setLightLevel(0);
			ascending = true;
		}
	}
}

void setLightLevel(int level){
	// Make sure the level is in the percentage range
	level = constrain(level, 0, 100);
	targetPercent = level;

	level = map(level, 0, 100, 255, 0);

	target = level;

	Log.Info("Light level: %d%", targetPercent);

	if (!transitionsEnabled){
		writeLightLevel(target);
	}
}


void update(){
	if (current != target){
		if (target > current){
			writeLightLevel(current + TRANSITION_STEP);
		}
		else{
			writeLightLevel(current - TRANSITION_STEP);
		}
	}
}

void writeLightLevel(int level){
	constrain(level, 0, 255);

	analogWrite(LED_CONTROL_PIN, level);
	current = level;
	Log.Debug("Output PWM level: %d", level);
}

void transitionsOn(){
	transitionsEnabled = true;
	Log.Info("Transitions enabled. Step: %d", TRANSITION_STEP);
}

void transitionsOff(){
	transitionsEnabled = false;
	Log.Info("Transitions disabled.");
}

void lampOff(){
	setLightLevel(0);
}

void lamp10(){
	setLightLevel(10);
}

void lamp20(){
	setLightLevel(20);
}

void lamp30(){
	setLightLevel(30);
}

void lamp40(){
	setLightLevel(40);
}

void lamp50(){
	setLightLevel(50);
}

void lamp60(){
	setLightLevel(60);
}

void lamp70(){
	setLightLevel(70);
}

void lamp80(){
	setLightLevel(80);
}

void lamp90(){
	setLightLevel(90);
}

void lampOn(){
	setLightLevel(100);
}
