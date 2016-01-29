#include <OneWire.h>
#include <DallasTemperature.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include <EmonLib.h>
#include <RTClib.h>
#include <Wire.h>
#include <BH1750FVI.h>
#include <Adafruit_MLX90614.h>
#include <Arduino.h>
#include <LIDARduino.h>
#include <SPI\SPI.h>
#include <ProgmemString.h>
#include <StraightBuffer.h>
#include <Logging.h>
#include <JsonParser.h>
#include <JsonGenerator.h>
#include <Maxbotix.h>
#include <SimpleTimer.h>
#include <CommandHandler.h>


#include "config.h"

using namespace ArduinoJson::Generator;

const int SSL_TESTBED_VERSION = 7;


//////////////////////////////////////////////////////////////////////////
// Config

// XBee
SoftwareSerial xbee_bridge(XBEE_RX, XBEE_TX);
XBee xbee = XBee();

// Traffic Sensors
Maxbotix rangeSensor(RANGEFINDER_AN_PIN, Maxbotix::AN, Maxbotix::XL);
int rangeBaseline;
static int successiveSonarDetections = 0;

LIDAR_Lite_PWM lidar(LIDAR_TRIGGER_PIN, LIDAR_PWM_PIN);
int lidarBaselineRange;
static int successiveLidarDetections = 0;

bool motionDetected;

// Environmental Sensors
Adafruit_MLX90614 roadTemperatureSensor;
BH1750FVI illuminanceSensor;
OneWire onewire(TEMPERATURE_PIN);
DallasTemperature airTemperatureSensor(&onewire);


// Misc
RTC_DS1307 rtc;
EnergyMonitor currentMonitor;
bool lampControlEnabled = false;

// Timers
SimpleTimer timer;
int rangeTimerID = -1;
int motionTimerID = -1;
int flowTimerID = -1;
int lidarTimerID = -1;
int airTemperatureTimerID = -1;
int roadTemperatureTimerID = -1;
int caseTemperatureTimerID = -1;
int humidityTimerID = -1;
int illuminanceTimerID = -1;
int lampStatusTimerID = -1;
int noiseTimerID = -1;
int currentTimerID = -1;
int printTimerID = -1;
int xbeeTimerID = -1;
int lampTimerID = -1;
int lampTransitionTimer = -1;

// Data storage
JsonObject<20> sensorEntry;
StraightBuffer sendBuffer(SEND_BUFFER_SIZE);

// Serial commands
char _commandCache[COMMAND_CACHE_SIZE];
CommandHandler commandHandler(_commandCache, COMMAND_CACHE_SIZE);

long entryNumber;
bool linuxBusy;


//////////////////////////////////////////////////////////////////////////
// Arduino Functions
//////////////////////////////////////////////////////////////////////////

/**
* Startup code - Run once
*/
void setup()
{
	if (DEBUG_ENABLED){
		Log.Init(LOG_LEVEL_DEBUG, 115200);
	}
	else{
		startYunSerial();
	}
	Log.Info(P("Traffic Counter - ver %d"), SSL_TESTBED_VERSION);

	//startLampControl();

	// XBee
	startXBee();

	// Start up sensors
	startSensors();
	addSerialCommands();
}

/**
* Main loop - run forever
*/
void loop()
{
	timer.run();
	//commandHandler.checkSerial();
}


//////////////////////////////////////////////////////////////////////////
// Yun Serial

/**
* Set up the Arduino-Linux serial bridge
* Serial output from the Arduino is disabled until the Yun has finished booting
*/
void startYunSerial(){

	// Set up the handshake pin
	pinMode(YUN_HANDSHAKE_PIN, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Give a delay for the AR9331 to reset, in case we were reset as part of a reboot command.
	// See http://playground.arduino.cc/Hardware/Yun#rebootStability, case 3.
	delay(5000);

	// If the system is booting, wait for UBoot to finish (Shamelessly adapted from Bridge.cpp)
	Serial1.begin(250000);
	do {
		while (Serial1.available() > 0) {
			Serial1.read();
		}
		delay(5000);
	} while (Serial1.available() > 0);
	Serial1.end();

	// Check the initial state of the handshake pin (LOW == Ready)
	_bootStatusChange();

	Serial1.begin(SERIAL_BAUD);

	// Listen on the handshake pin for any changes
	attachInterrupt(4, _bootStatusChange, CHANGE);
}

/**
* Check the boot status of the Yun
*/
void _bootStatusChange(){
	linuxBusy = digitalRead(YUN_HANDSHAKE_PIN);

	// Disable log output until Linux boots
	if (linuxBusy){
		Log.Init(LOG_LEVEL_NOOUTPUT, &Serial1);
	}
	else{
		Log.Init(LOGGER_LEVEL, &Serial1);
	}
}


//////////////////////////////////////////////////////////////////////////
// XBee Comms

void startXBee(){
	xbee_bridge.begin(XBEE_BAUD);
	xbee.begin(xbee_bridge);

	sendBuffer.reset();

	xbeeTimerID = timer.setInterval(XBEE_TRANSMIT_INTERVAL, sendXBeePacket);
}

void prepareXBeePacket(){
	sendBuffer.reset();

	sendBuffer.write(PACKET_START);

	sendBuffer.write(AMBIENT_TEMPERATURE_TAG);
	sendBuffer.writeInt(int(float(sensorEntry[AIR_TEMP]) * 100));

	sendBuffer.write(ROAD_TEMPERATURE_TAG);
	sendBuffer.writeInt(int(float(sensorEntry[ROAD_TEMP]) * 100));

	sendBuffer.write(HUMIDITY_TAG);
	sendBuffer.writeInt(int(float(sensorEntry[HUMIDITY]) * 100));

	sendBuffer.write(ILLUMINANCE_TAG);
	sendBuffer.writeInt(int(sensorEntry[ILLUMINANCE]));

	//sendBuffer.write(LAMP_STATUS_TAG);
	//sendBuffer.write(bool(sensorEntry[LAMP_STATUS]));

	sendBuffer.write(NOISE_TAG);
	sendBuffer.writeInt(int(sensorEntry[NOISE]));

	sendBuffer.write(CURRENT_DRAW_TAG);
	sendBuffer.writeInt(int(float(sensorEntry[CURRENT_DRAW]) * 100));

	sendBuffer.write(TIMESTAMP_TAG);
	sendBuffer.writeLong(sensorEntry[TIME_STAMP]);

	sendBuffer.write(UVD_RANGE_TAG);
	sendBuffer.writeInt(sensorEntry[UVD_RANGE]);

	sendBuffer.write(UVD_COUNT_TAG);
	sendBuffer.writeInt(sensorEntry[COUNT_UVD]);

	sendBuffer.write(MOTION_STATUS_TAG);
	sendBuffer.write(bool(sensorEntry[PIR_STATUS]));

	sendBuffer.write(MOTION_COUNT_TAG);
	sendBuffer.writeInt(sensorEntry[COUNT_PIR]);

	sendBuffer.write(LIDAR_RANGE_TAG);
	sendBuffer.writeInt(sensorEntry[LIDAR_RANGE]);

	sendBuffer.write(LIDAR_COUNT_TAG);
	sendBuffer.writeInt(sensorEntry[COUNT_LIDAR]);
}

void transmitPacket(){
	xbee_bridge.write(sendBuffer.getBufferAddress(), int(sendBuffer.getWritePosition()));
}

void sendXBeePacket(){
	prepareXBeePacket();
	transmitPacket();
}


//////////////////////////////////////////////////////////////////////////
// Sensors

/**
* Enable and configure the sensor suite for reading
*/
void startSensors(){
	sensorEntry[ID] = UNIT_NAME;
	sensorEntry[VERSION] = SSL_TESTBED_VERSION;
	startRTC();

	// Traffic
	startRangefinder();
	startLidar();
	startMotionDetector();

	// Environment
	startAirTemperatureSensor();
	startRoadTemperatureSensor();
	startCaseTemperatureSensor();
	startHumiditySensor();
	//startLightStatus();
	startIlluminanceSensor();
	startMicrophone();
	//startCurrentSensor();

	printTimerID = timer.setInterval(PRINT_INTERVAL, printDataEntry);
}

/**
* Add commands to the response list
*/
void addSerialCommands(){
	commandHandler.setTerminator(COMMAND_TERMINATOR);

	//Add commands here

	commandHandler.setDefaultHandler(defaultHandler);
}

/**
* Bad command received. Make fun of the sender
*/
void defaultHandler(char c){
	Log.Error(P("Command not recognised: %s"), c);
}

/**
* Print a normal data entry
* The traffic event flag is 'off'
*/
void printDataEntry(){
	sensorEntry[EVENT_FLAG] = false;
	printData();
}

/**
* Print a data entry
* The traffic event flag is enabled
*/
void printTrafficEntry(){

	// Activate the lamp (if enabled)
	if (lampControlEnabled){
		activateLamp();
	}

	sensorEntry[EVENT_FLAG] = true;
	printData();

	sendXBeePacket();
}

/**
* Print the current traffic counts and info to Serial
*/
void printData(){
	readDateTime();

	if (DEBUG_ENABLED){
		sensorEntry.printTo(Serial);

	}

	else{
		if (!linuxBusy){
			Serial1.print("#");
			sensorEntry.printTo(Serial1);
			Serial1.println("$");
		}

	}
}

/**
* Read all of the environmental sensors
*/
void readEnvironmentalSensors(){
	readAirTemperature();
	readRoadTemperature();
	readHumidity();
	readIlluminance();
	readCurrentDraw();
}


//////////////////////////////////////////////////////////////////////////
// Rangefinder

/**
* Start the rangefinder sensor
*
* A baseline range (to the ground or static surrounds) is established for
* comparing against new measuremets.
*
*/
void startRangefinder(){
	Log.Debug(P("Ultrasonic - Establishing baseline range..."));
	rangeBaseline = getRangefinderBaseline(BASELINE_VARIANCE_THRESHOLD);

	Log.Debug(P("Ultrasonic Baseline established: %d cm, %d cm variance"), rangeBaseline, BASELINE_VARIANCE_THRESHOLD);

	// Enable the sensor if it passes the baseline check
	if (rangeBaseline > RANGE_DETECT_THRESHOLD) {
		rangeTimerID = timer.setInterval(CHECK_RANGE_INTERVAL, checkRange);
		sensorEntry[COUNT_UVD] = 0;
		sensorEntry[UVD_RANGE] = rangeBaseline;
	}
}

/**
* Establish the baseline range from the sensor to the ground
* The sensor will take samples until the readings are consistent
*/
int getRangefinderBaseline(int variance){
	// Establish baseline (fixed height) of the range sensor
	int averageRange = getRange();
	int baselineReads = 1;
	int averageVariance = 500;

	// Keep reading in the baseline until it stablises
	while ((baselineReads < MIN_BASELINE_READS || averageVariance > variance) && baselineReads < MAX_BASELINE_READS){
		int newRange = getRange();
		int newVariance = (averageRange - newRange);
		if (newVariance < 0){ newVariance *= -1; }

		averageVariance = ((averageVariance + newVariance) / 2);
		averageRange = ((averageRange + newRange) / 2);

		Log.Debug(P("Ultrasonic Calibration: Range - %d, Variance - %d"), int(averageRange), averageVariance);

		baselineReads++;
		delay(BASELINE_READ_INTERVAL);
	}

	if (averageVariance > variance) {
		averageRange = -1;
	}

	return averageRange;
}

/**
* Get the range in cm from the ultrasonic rangefinder
*
* @return Target distance from sensor in cm
*/
int getRange(){
	int targetDistance = rangeSensor.getRange();
	Log.Debug(P("Ultrasonic Range: %d cm"), targetDistance);
	return targetDistance;
}

/**
* Check the ultrasonic range sensor to see if a traffic event has occurred.
*
* Traffic events are counted as a break in the sensor's 'view' of the ground.
* Any object between the sensor and the ground baseline will cause the sensor
* to register a shorter range than usual.
*/
void checkRange(){
	int newRange = getRange();
	sensorEntry[UVD_RANGE] = newRange;

	// Detection occurs when target breaks the LoS to the baseline
	if ((rangeBaseline - newRange) > RANGE_DETECT_THRESHOLD){

		// If two in a row
		if (successiveSonarDetections == MIN_SUCCESSIVE_SONAR_READS){
			successiveSonarDetections += 1;

			// Increase traffic count
			int trafficCount = sensorEntry[COUNT_UVD];
			trafficCount++;
			sensorEntry[COUNT_UVD] = trafficCount;
			Log.Info(P("Traffic count - UVD: %d counts"), trafficCount);

			// Also send an XBee alert
			printTrafficEntry();
		}

		else if (successiveSonarDetections < MIN_SUCCESSIVE_SONAR_READS){
			successiveSonarDetections += 1;
		}
	}

	else{
		successiveSonarDetections = 0;
	}
}

/**
* Reset the traffic count for the ultrasonic rangefinder
*/
void resetRangeCount(){
	sensorEntry[COUNT_UVD] = 0;
	Log.Info(P("Traffic count reset - UVD"));
}


//////////////////////////////////////////////////////////////////////////
// Lidar

/**
* Start the rangefinder sensor
*
* A baseline range (to the ground or static surrounds) is established for
* comparing against new measuremets.
*
*/
void startLidar(){
	pinMode(LIDAR_TRIGGER_PIN, INPUT);
	pinMode(LIDAR_PWM_PIN, INPUT);

	lidar.begin();

	Log.Debug(P("Lidar - Establishing baseline range..."));
	lidarBaselineRange = getLidarBaselineRange(BASELINE_VARIANCE_THRESHOLD);
	Log.Debug(P("Lidar Baseline established: %d cm, %d cm variance"), lidarBaselineRange, BASELINE_VARIANCE_THRESHOLD);

	if (lidarBaselineRange > LIDAR_DETECT_THRESHOLD) {
		lidarTimerID = timer.setInterval(LIDAR_CHECK_RANGE_INTERVAL, checkLidarRange);
		sensorEntry[COUNT_LIDAR] = 0;
		sensorEntry[LIDAR_RANGE] = 0;
	}
}

/**
* Establish the baseline range from the lidar to the ground
* The sensor will take samples until the readings are consistent
*/
int getLidarBaselineRange(int variance){
	// Establish baseline (fixed height) of the range sensor
	int averageRange = getLidarRange();
	int baselineReads = 1;
	int averageVariance = 500;

	// Keep reading in the baseline until it stablises
	while ((baselineReads < MIN_BASELINE_READS || averageVariance > variance) && baselineReads < MAX_BASELINE_READS){
		int newRange = getLidarRange();
		int newVariance = (averageRange - newRange);
		if (newVariance < 0){ newVariance *= -1; }

		averageVariance = ((averageVariance + newVariance) / 2);
		averageRange = ((averageRange + newRange) / 2);

		Log.Debug(P("Lidar Calibration: Range - %d, Variance - %d"), int(averageRange), averageVariance);

		baselineReads++;
		delay(BASELINE_READ_INTERVAL);
	}

	// Calibration fails if the range is varying too much
	if (averageVariance > variance) {
		averageRange = -1;
	}

	return averageRange;
}

/**
* Get the range in cm from the lidar
*
* @return Target distance from sensor in cm
*/
int getLidarRange(){
	int targetDistance = lidar.getDistance();
	Log.Debug(P("Lidar Range: %d cm"), targetDistance);
	return targetDistance;
}

/**
* Check the lidar to see if a traffic event has occurred.
*
* Traffic events are counted as a break in the sensor's 'view' of the ground.
* Any object between the sensor and the ground baseline will cause the sensor
* to register a shorter range than usual.
*/
void checkLidarRange(){
	int newRange = getLidarRange();
	sensorEntry[LIDAR_RANGE] = newRange;

	// Detection occurs when target breaks the LoS to the baseline
	if ((lidarBaselineRange - newRange) > RANGE_DETECT_THRESHOLD){

		// If x in a row
		if (successiveLidarDetections == MIN_SUCCESSIVE_LIDAR_READS){
			successiveLidarDetections += 1;

			// Increase traffic count
			int trafficCount = sensorEntry[COUNT_LIDAR];
			trafficCount++;
			sensorEntry[COUNT_LIDAR] = trafficCount;
			Log.Info(P("Traffic count - Lidar: %d counts"), trafficCount);

			// Also send an XBee alert
			printTrafficEntry();
		}

		else if (successiveLidarDetections < MIN_SUCCESSIVE_LIDAR_READS){
			successiveLidarDetections += 1;
		}
	}

	else{
		successiveLidarDetections = 0;
	}
}

/**
* Reset the traffic count for the lidar
*/
void resetLidarCount(){
	sensorEntry[COUNT_LIDAR] = 0;
}


//////////////////////////////////////////////////////////////////////////
// PIR

/**
* Start the PIR motion sensor for motion detections
*/
void startMotionDetector(){
	pinMode(PIR_MOTION_PIN, INPUT);

	Log.Info(P("Calibrating motion sensor - wait %d ms"), MOTION_INITIALISATION_TIME);


	for (long i = 0; i < MOTION_INITIALISATION_TIME; i += (MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS)){
		Log.Debug(P("Motion sensor calibration: %d ms remaining..."), (MOTION_INITIALISATION_TIME - i));
		Serial.flush();
		delay(MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS);
	}

	motionDetected = false;

	motionTimerID = timer.setInterval(CHECK_MOTION_INTERVAL, checkPirMotion);
	Log.Info(P("Motion started"));
	sensorEntry[COUNT_PIR] = 0;
	sensorEntry[PIR_STATUS] = 0;
}

/**
* Stop checking the motion detector
*/
void stopMotionDetector(){
	timer.deleteTimer(motionTimerID);


	Log.Info(P("Motion sensor stopped"));
}

/**
* Check the PIR sensor for detected movement
* The sensor will output HIGH when motion is detected.
* Detections will hold the detection status HIGH until the cool-down has lapsed (default: 60s)
*
* Returns:
*	True if motion has been detected recently
*/
void checkPirMotion(){

	// Check the sensor
	if (digitalRead(PIR_MOTION_PIN) == MOTION_DETECTED){

		motionDetected = true;
		Log.Debug(P("Motion detected"));

		// Increment PIR count
		long motionCount = sensorEntry[COUNT_PIR];
		motionCount++;
		sensorEntry[COUNT_PIR] = motionCount;

		Log.Info(P("Traffic count - PIR: %l counts"), motionCount);

		printTrafficEntry();

		// Enter the cooldown phase
		motionCoolDown();

	}
	else{
		// No alarm; is fine
		motionDetected = false;
	}


	sensorEntry[PIR_STATUS] = motionDetected;
}

/**
* Temporarily pause checking the PIR motion sensor
*/
void motionCoolDown(){
	timer.deleteTimer(motionTimerID);
	motionTimerID = -1;
	timer.setTimeout(MOTION_COOLDOWN, resumeMotionDetection);
}

/**
* Start polling the PIR motion sensor
*/
void resumeMotionDetection(){
	motionTimerID = timer.setInterval(CHECK_MOTION_INTERVAL, checkPirMotion);
}

/**
* Reset the number of motion detections
*/
void resetMotionCount(){
	sensorEntry[COUNT_PIR] = 0;
}


//////////////////////////////////////////////////////////////////////////
// Air Temperature

/**
* Start the air temperature sensor
*/
void startAirTemperatureSensor(){

	// TMP36
	pinMode(TEMPERATURE_PIN, INPUT);

	// Uncomment in case of DS18B20
	/*airTemperatureSensor.begin();
	airTemperatureSensor.setResolution(TEMPERATURE_RESOLUTION);*/

	airTemperatureTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readAirTemperature);
}

/**
* Read the air temperature sensor
*/
float getAirTemperature(){
	float air_temp;

	// TMP36
	float temp_voltage = (analogRead(TEMPERATURE_PIN) * AREF_VOLTAGE) / 1024;
	air_temp = (temp_voltage - 0.5) * 100;

	// DS18B20
	/*airTemperatureSensor.requestTemperatures();
	air_temp = airTemperatureSensor.getTempCByIndex(0);*/

	return air_temp;
}

/**
* Read the air temperature into the current entry
*/
void readAirTemperature(){
	sensorEntry[AIR_TEMP] = getAirTemperature();
}


//////////////////////////////////////////////////////////////////////////
// Road Temperature

/**
* Start the road temperature sensor
*/
void startRoadTemperatureSensor(){
	roadTemperatureSensor.begin();
	roadTemperatureTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readRoadTemperature);
}

/**
* Read the road temperature from the non-contact thermometer
*/
float getRoadTemperature(){
	return roadTemperatureSensor.readObjectTempC();
}

/**
* Read the road temperature into the entry
*/
void readRoadTemperature(){
	sensorEntry[ROAD_TEMP] = getRoadTemperature();
}


//////////////////////////////////////////////////////////////////////////
// Case Temperature

/**
* Start the road temperature sensor
*/
void startCaseTemperatureSensor(){
	roadTemperatureSensor.begin();
	caseTemperatureTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readCaseTemperature);
}

/**
* Read the road temperature from the non-contact thermometer
*/
float getCaseTemperature(){
	return roadTemperatureSensor.readAmbientTempC();
}

/**
* Read the road temperature into the entry
*/
void readCaseTemperature(){
	sensorEntry[CASE_TEMP] = getCaseTemperature();
}


//////////////////////////////////////////////////////////////////////////
// Humidity 

/**
* Start the HIH4030 humidity sensor
*/
void startHumiditySensor(){
	pinMode(HUMIDITY_PIN, INPUT);
	humidityTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readHumidity);
}

/**
* Calculate the humidity from the sensor
*/
float getHumidity(float air_temperature){
	int raw_humidity = analogRead(HUMIDITY_PIN);
	float humidity_voltage = (raw_humidity * AREF_VOLTAGE) / 1024.0;

	// See HIH4030 for conversion formula
	float uncalibrated_humidity = 161.0 * humidity_voltage / AREF_VOLTAGE - 25.8;
	float real_humidity = uncalibrated_humidity / (1.0546 - 0.0026 * air_temperature);

	return real_humidity;
}

/**
* Read the humidity into the current entry
*/
void readHumidity(){
	sensorEntry[HUMIDITY] = getHumidity(getAirTemperature());
}


//////////////////////////////////////////////////////////////////////////
// Illuminance

/**
* Start the BH1750FVI illuminance sensor
*/
void startIlluminanceSensor(){
	illuminanceSensor.begin();
	illuminanceSensor.SetAddress(Device_Address_L);
	illuminanceSensor.SetMode(Continuous_H_resolution_Mode);

	illuminanceTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readIlluminance);
}

/**
* Read the illuminance from the sensor
*
* @return: illuminance value in lux
*/
int getIlluminance(){
	return illuminanceSensor.GetLightIntensity();
}

/**
* Read the illuminance into the current entry
*/
void readIlluminance(){
	sensorEntry[ILLUMINANCE] = getIlluminance();
}


//////////////////////////////////////////////////////////////////////////
// Lamp Status 

/**
* Start the lamp status sensor
*/
void startLightStatus(){
	pinMode(LAMP_STATUS_PIN, INPUT);

	lampStatusTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readLightStatus);
}

/**
* Get the status of the lamp status sensor
*
* @returns: True if the lamp is on
*/
bool getLightStatus(){
	bool lamp_status = false;
	int lamp_level = analogRead(LAMP_STATUS_PIN);

	if (lamp_level < LIGHT_HEALTHY_THRESHOLD){
		lamp_status = true;
	}

	return lamp_status;
}

/**
* Read the lamp status into the current entry
*/
void readLightStatus(){
	sensorEntry[LAMP_STATUS] = getLightStatus();
}


//////////////////////////////////////////////////////////////////////////
// Noise 

/**
* Start the microphone for sound pressure monitoring
*/
void startMicrophone(){
	pinMode(MICROPHONE_PIN, INPUT);

	noiseTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readSoundLevel);
}

/**
* Get the average sound level over the specified sampling period
*
* @param samplePeriod Listening period for the sampling in ms.
* @return Average sound level in 10-bit counts
*/
int getSoundLevel(int sample_period){
	unsigned long startTime = millis();
	long total = 0;
	long count = 0;

	while (millis() < (startTime + sample_period)){
		int soundLevel = analogRead(MICROPHONE_PIN);
		total += soundLevel;
		count += 1;
	}

	int average = int(total / count);
	return average;
}

/**
* Read the sound level into the current entry
*/
void readSoundLevel(){
	sensorEntry[NOISE] = getSoundLevel(SOUND_SAMPLE_TIME);
}


//////////////////////////////////////////////////////////////////////////
// Current 

/**
* Start the split-core current sensor
*/
void startCurrentSensor(){
	currentMonitor.current(CURRENT_DETECT_PIN, CURRENT_CALIBRATION_FACTOR);

	currentTimerID = timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, readCurrentDraw);
}

/**
* Get the current draw from the clamp
*/
float getCurrentDraw(){
	return currentMonitor.calcIrms(CURRENT_SAMPLES);
}

/**
* Read the current draw into the current entry
*/
void readCurrentDraw(){
	sensorEntry[CURRENT_DRAW] = getCurrentDraw();
}


//////////////////////////////////////////////////////////////////////////
// RTC

/**
* Start up the real-time clock
*/
void startRTC(){
	rtc.begin();
}

/**
* Get the current timestamp.
*
* @return: seconds since 1/1/2000
*/
long getDateTime(){
	DateTime now = rtc.now();
	return now.secondstime();
}

/**
* Read the timestamp into the current entry
*/
void readDateTime(){
	sensorEntry[TIME_STAMP] = getDateTime();
}


//////////////////////////////////////////////////////////////////////////
// Dimming Control

/**
* Start the lamp
*/
void startLampControl(){
	//Start lamp in active mode
	activateLamp();
	lampControlEnabled = true;
};

/**
* Switch the lamp to the active state
* The lamp will go back to the inactive state after the cooldown has elapsed.
* Successive activation reset the cooldown timer.
*/
void activateLamp(){
	setLampTarget(ACTIVE_BRIGHTNESS);

	// Start the lamp cooldown timer; reset if lamp is already active
	if (lampTimerID < 0){
		lampTimerID = timer.setTimeout(LAMP_ACTIVE_TIME, deactivateLamp);
	}
	else{
		timer.restartTimer(lampTimerID);
	}
}

/**
* Set the lamp to the inactive state
*/
void deactivateLamp(){
	setLampTarget(INACTIVE_BRIGHTNESS);

	// Trash the timer
	timer.deleteTimer(lampTimerID);
}

/**
* Set the brightness target for the lamp (in percentage; not binary level)
* Lamp brightness changes with a transition; changes are not instantaneous
*
* @level: brighteness as a percentage of the maximum output
*
*/
void setLampTarget(int level){
	// Make sure the level is in the percentage range
	level = constrain(level, 0, 100);
	level = map(level, 0, 100, 255, 0);

	sensorEntry[LAMP_TARGET] = level;
	transitionLamp();
}

/**
* Write the lamp level; instantaneous change
* @level: Lamp level in binary (0-255; 255 == maximum)
*/
void writeLampLevel(int level){
	constrain(level, 0, 255);

	analogWrite(LAMP_CONTROL_PIN, level);
	sensorEntry[LAMP_OUTPUT] = level;
}

/**
* Step the lamp output towards the target output.
* Time between transition steps and step size changes with direction.
* e.g. Transitions from low to high can be set fast (high step size; low transition period)
* Where transitions from high to low can be slower and softer (low step size w/ long period between steps)
*/
void transitionLamp(){
	byte target = int(sensorEntry[LAMP_TARGET]);
	byte output = int(sensorEntry[LAMP_OUTPUT]);
	byte new_output;
	long transitionTime;

	// If the output is already close to the target, instantly change the output to match
	if ((target >= output && (target - output) <= LAMP_TRANSITION_STEP_UP) || (target >= output && (output - target) <= LAMP_TRANSITION_STEP_DOWN)){
		writeLampLevel(target);
	}

	// Else, a transition is needed; find out what the new output and period needs to be
	else{
		if (target > output){
			new_output = output + LAMP_TRANSITION_STEP_UP;
			transitionTime = LAMP_TRANSITION_UP_PERIOD;
		}
		else{
			new_output = output - LAMP_TRANSITION_STEP_DOWN;
			transitionTime = LAMP_TRANSISTION_DOWN_PERIOD;
		}

		// Apply next transition output and reschedule a transition (period depending on direction)
		writeLampLevel(new_output);
		lampTransitionTimer = timer.setTimeout(transitionTime, transitionLamp);
	}
}

