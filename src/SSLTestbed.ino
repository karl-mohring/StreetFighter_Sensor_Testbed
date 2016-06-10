#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <EmonLib.h>
#include <RTClib.h>
#include <Wire.h>
#include <ArduinoJson.h>

#include "BH1750FVI.h"
#include "Adafruit_MLX90614.h"
#include "LIDARduino.h"
#include "ProgmemString.h"
#include "Logging.h"
#include "SimpleTimer.h"

#include "config.h"

// XBee - in Transparent (Serial bridge) mode
SoftwareSerial xbee_bridge(COMM_1_RX, COMM_1_TX);

// Bluetooth module
SoftwareSerial bluetooth(COMM_2_RX, COMM_2_TX);

// Traffic Sensors
static int successive_sonar_detections = 0;
LIDAR_Lite_PWM lidar(LIDAR_TRIGGER_PIN, LIDAR_PWM_PIN);
static int successive_lidar_detections = 0;

// Environmental Sensors
Adafruit_MLX90614 road_temperature_sensor;
BH1750FVI illuminance_sensor;
OneWire onewire(TEMPERATURE_PIN);
DallasTemperature air_temperature_sensor(&onewire);

// Misc
RTC_DS1307 rtc;
EnergyMonitor current_monitor;
bool lamp_control_enabled = false;

// JSON serialiser - should be enclosed in function scope, but dynamic memory allocation scares me
StaticJsonBuffer<COMM_BUFFER_SIZE> print_buffer;
JsonObject& entry = print_buffer.createObject();

// Timers
SimpleTimer timer;
int pir_timer = -1;

SensorEntry data;
LampControl lamp;

/* Arduino main functions */
void setup(){
	/**
	* Startup code - Run once
	*/
	data.id = UNIT_NAME;
	data.version = SSL_TESTBED_VERSION;
	lamp.control_enabled = false;

	//start_yun_serial();
	Log.Init(LOGGER_LEVEL, SERIAL_BAUD);
	Log.Info(P("Traffic Counter - ver %d"), SSL_TESTBED_VERSION);

	start_rtc();

	if (LAMP_CONTROL_ENABLED) {
		enable_lamp_control();
	}

	if (XBEE_ENABLED) {
		start_xbee();
	}

	if (BLUETOOTH_ENABLED) {
		start_bluetooth_scanner();
	}

	start_sensors();
}


void loop(){
	/**
	* Main loop - run forever
	*/
	timer.run();
}


/* Yun Serial */
void start_yun_serial(){
	/**
	* Set up the Arduino-Linux serial bridge
	* Serial output from the Arduino is disabled until the Yun has finished booting
	* The serial can be accessed safely after the boot debug finishes.
	* Any serial transmission during this time would halt start-up, which would suck.
	*/

	// Set up the handshake pin
	pinMode(YUN_HANDSHAKE_PIN, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Give a delay for the AR9331 to reset, in case we were reset as part of a reboot command.
	// See http://playground.arduino.cc/Hardware/Yun#rebootStability, case 3.
	delay(YUN_BOOT_DELAY);

	// If the system is booting, wait for UBoot to finish (Shamelessly adapted from Bridge.cpp)
	USE_SERIAL.begin(YUN_LINUX_BAUD_RATE);
	do {
		while (USE_SERIAL.available() > 0) {
			USE_SERIAL.read();
		}
		delay(YUN_BOOT_DELAY);
	} while (USE_SERIAL.available() > 0);
	USE_SERIAL.end();

	// Check the initial state of the handshake pin (LOW == Ready)
	boot_status_change_ISR();

	USE_SERIAL.begin(SERIAL_BAUD);

	// Listen on the handshake pin for any changes
	attachInterrupt(4, boot_status_change_ISR, CHANGE);
}


void boot_status_change_ISR(){
	/**
	* ISR - Called when the handshake pin (D7) is changed by the AR9331
	* Toggle serial output depending on the boot state of the Yun.
	* If the handshake pin changes, the boot status of the Yun has changed.
	* Handshake == LOW means the linux environment is ready for serial
	*/
	bool linux_busy = digitalRead(YUN_HANDSHAKE_PIN);

	if (linux_busy){
		// Disable log output until Linux boots
		Log.Init(LOG_LEVEL_NOOUTPUT, &USE_SERIAL);
	}
	else{
		Log.Init(LOGGER_LEVEL, &USE_SERIAL);
	}
}


/* XBee Comms */
void start_xbee(){
	/**
	* Start up xbee communication
	* XBee is connected to software serial
	* XBee communication uses AT mode - which just uses the XBee as a serial bridge.
	* There is no packet handling and all data travels one way: out
	*/
	xbee_bridge.begin(XBEE_BAUD);
}


/* Printing */
void start_sensors(){
	/**
	* Enable and configure the sensor suite for reading.
	* A timer is started to regularly print sensor data
	*/

	// Traffic
	if (SONAR_ENABLED){
		start_sonar();
	}

	if (LIDAR_ENABLED) {
		start_lidar();
	}

	if (PIR_ENABLED) {
		start_pir();
	}


	// Environment
	if (AIR_TEMPERATURE_ENABLED) {
		start_air_temperature();
	}

	if (ROAD_TEMPERATURE_ENABLED) {
		start_road_temperature();
		start_case_temperature();
	}

	if (HUMIDITY_ENABLED) {
		start_humidity();
	}

	if (ILLUMINANCE_ENABLED) {
		start_illuminance();
	}

	if (MICROPHONE_ENABLED) {
		start_noise_monitoring();
	}


	// Start timer for regular entry transmission
	timer.setInterval(PRINT_INTERVAL, print_regular_entry);
}


void print_regular_entry(){
	/**
	* Print a normal data entry
	* Regular entries are triggered by timer, rather than sensor
	* The traffic event flag is 'off'
	*/
	data.event_flag = false;
	print_data();
}


void trigger_traffic_event(){
	/**
	* Print a data entry
	* The traffic event flag is enabled
	*/

	// Activate the lamp (if enabled)
	if (lamp.control_enabled){
		activate_lamp();
	}

	data.event_flag = true;
	print_data();
}


void print_data(){
	/**
	* Print the current traffic counts and info to Serial
	*/
	update_timestamp();
	print_json_string();
}


void print_json_string(){
	/**
	* Format the data packet into a JSON string
	* @return JSON-formatted string containing selected data
	*/

	entry[P("version")] = data.version;
	entry[P("id")] = data.id;
	entry[P("event_flag")] = data.event_flag;
	entry[P("count_pir")] = data.pir_count;
	entry[P("pir_status")] = data.last_pir_status;
	entry[P("lidar_count")] = data.lidar_count;
	entry[P("lidar_range")] = data.lidar_range;
	entry[P("uvd_count")] = data.sonar_count;
	entry[P("uvd_range")] = data.sonar_range;
	entry[P("air_temp")] = data.air_temperature;
	entry[P("case_temp")] = data.case_temperature;
	entry[P("road_temp")] = data.road_temperature;
	entry[P("humidity")] = data.humidity;
	entry[P("illuminance")] = data.illuminance;
	entry[P("current_draw")] = data.current_draw;
	entry[P("noise")] = data.noise_level;
	entry[P("timestamp")] = data.timestamp;

	if (Log.getLevel() > LOG_LEVEL_NOOUTPUT){
		USE_SERIAL.print(PACKET_START);
		entry.printTo(USE_SERIAL);
		USE_SERIAL.println(PACKET_END);
		USE_SERIAL.flush();
	}

	if (XBEE_ENABLED) {
		// Push the JSON string to XBee as well
		xbee_bridge.print(PACKET_START);
		entry.printTo(xbee_bridge);
		xbee_bridge.print(PACKET_END);
	}
}


/* Sonar */
void start_sonar(){
	/**
	* Start the sonar/ultrasonic rangefinder
	* A baseline range (to the ground or static surrounds) is established for comparing against new measurements.
	* The sonar is disabled if the sensor cannot establish a baseline measurement
	*/

	Log.Debug(P("Ultrasonic - Establishing baseline range..."));
	data.sonar_baseline = get_sonar_baseline(BASELINE_VARIANCE_THRESHOLD);

	data.sonar_count = 0;
	data.sonar_range = 0;
	timer.setInterval(CHECK_RANGE_INTERVAL, update_sonar);

	// Enable the sensor if it passes the baseline check
	if (data.sonar_baseline > RANGE_DETECT_THRESHOLD) {
		Log.Info(P("Sonar started - Baseline: %dcm"), data.sonar_baseline);
	}else{
		data.sonar_baseline = 0;
		Log.Error(P("Sonar initialisation failed - sensor disabled"));
	}

	update_sonar();
}


int get_sonar_baseline(int variance){
	/**
	* Establish the baseline range from the sensor to the ground
	* The sensor will take samples until the readings are consistent or until timeout
	* :variance: Maximum amount of variance allowed between reads for valid baseline (in cm)
	* :return: Average variance of the baseline in cm
	*/
	int average_range = get_sonar_range();
	int baseline_reads = 1;
	int average_variance = 500;

	// Keep reading in the baseline until it stablises
	while ((baseline_reads < MIN_BASELINE_READS || average_variance > variance) && baseline_reads < MAX_BASELINE_READS){
		int new_range = get_sonar_range();
		int new_variance = abs(average_range - new_range);

		average_variance = ((average_variance + new_variance) / 2);
		average_range = ((average_range + new_range) / 2);

		Log.Debug(P("Ultrasonic Calibration: Range - %d, Variance - %d"), int(average_range), average_variance);

		baseline_reads++;
		delay(BASELINE_READ_INTERVAL);
	}

	// Baseline could not be established; send invalid measurement
	if (average_variance > variance) {
		average_range = -1;
	}

	return average_range;
}


int get_sonar_range(){
	/**
	* Get the range in cm from the ultrasonic rangefinder
	* :return: Distance to object in cm
	*/

	// The time of flight for the entire pulse, including the transmitted and reflected wave.
	long raw_time_of_flight = pulseIn(SONAR_PIN, HIGH, long(CHECK_RANGE_INTERVAL)*1000);

	// Ref: https://en.wikipedia.org/wiki/Speed_of_sound#Practical_formula_for_dry_air
	// Humidity does affect the speed of sound in air, but only slightly
	long compensated_distance = (raw_time_of_flight * (331.3 + 0.606 * data.air_temperature)) / 20000;

	Log.Verbose(P("Ultrasonic Range: %d cm"), compensated_distance);

	return int(compensated_distance);
}


void update_sonar(){
	/**
	* Check the ultrasonic range sensor to see if a traffic event has occurred.
	* Traffic events are counted as a break in the sensor's 'view' of the ground.
	* Any object between the sensor and the ground baseline will cause the sensor to register a shorter range than usual.
	*/
	data.sonar_range = get_sonar_range();

	// Detection occurs when target breaks the LoS to the baseline
	if ((data.sonar_baseline - data.sonar_range) > RANGE_DETECT_THRESHOLD){

		// If n in a row
		if (successive_sonar_detections == MIN_SUCCESSIVE_SONAR_READS){
			successive_sonar_detections += 1;

			// Increase traffic count
			data.sonar_count++;
			Log.Info(P("Traffic count - Sonar: %d counts"), data.sonar_count);

			// Also send an XBee alert
			trigger_traffic_event();
		}

		else if (successive_sonar_detections < MIN_SUCCESSIVE_SONAR_READS){
			successive_sonar_detections += 1;
		}
	}
	else{
		successive_sonar_detections = 0;
	}
}


/* Lidar */
void start_lidar(){
	/**
	* Start the rangefinder sensor
	* A baseline range (to the ground or static surrounds) is established for comparing against new measuremets.
	*/
	pinMode(LIDAR_TRIGGER_PIN, INPUT);
	pinMode(LIDAR_PWM_PIN, INPUT);
	lidar.begin();

	Log.Debug(P("Lidar - Establishing baseline range..."));
	data.lidar_baseline = get_lidar_baseline(BASELINE_VARIANCE_THRESHOLD);

	timer.setInterval(LIDAR_CHECK_RANGE_INTERVAL, update_lidar);
	data.lidar_count = 0;
	data.lidar_range = 0;

	if (data.lidar_baseline > LIDAR_DETECT_THRESHOLD) {

		Log.Info(P("Lidar started - Baseline: %dcm"), data.lidar_baseline);
	}else{
		data.lidar_baseline = 0;
		Log.Error(P("Lidar initialisation failed - sensor disabled"));
	}

	update_lidar();
}


int get_lidar_baseline(int variance){
	/**
	* Establish the baseline range from the lidar to the ground
	* The sensor will take samples until the readings are consistent
	* :variance: Max allowed variance of the baseline in cm
	* :return: Average variance of the baseline in cm
	*/
	int average_range = get_lidar_range();
	int baseline_reads = 1;
	int average_variance = 500;

	// Keep reading in the baseline until it stablises
	while ((baseline_reads < MIN_BASELINE_READS || average_variance > variance) && baseline_reads < MAX_BASELINE_READS){
		int new_range = get_lidar_range();
		int new_variance = abs(average_range - new_range);

		average_variance = ((average_variance + new_variance) / 2);
		average_range = ((average_range + new_range) / 2);

		Log.Debug(P("Lidar Calibration: Range - %d, Variance - %d"), int(average_range), average_variance);
		baseline_reads++;
		delay(BASELINE_READ_INTERVAL);
	}

	// Calibration fails if the range is varying too much
	if (average_variance > variance) {
		average_range = -1;
	}

	return average_range;
}


int get_lidar_range(){
	/**
	* Get the range in cm from the lidar
	* :return: Target distance from sensor in cm
	*/
	int target_distance = lidar.getDistance();
	Log.Verbose(P("Lidar Range: %d cm"), target_distance);
	return target_distance;
}


void update_lidar(){
	/**
	* Check the lidar to see if a traffic event has occurred.
	* Traffic events are counted as a break in the sensor's 'view' of the ground.
	* Any object between the sensor and the ground baseline will cause the sensor to register a shorter range than usual.
	*/
	data.lidar_range = get_lidar_range();

	// Detection occurs when target breaks the LoS to the baseline
	if ((data.lidar_baseline - data.lidar_range) > RANGE_DETECT_THRESHOLD){

		// If x in a row
		if (successive_lidar_detections == MIN_SUCCESSIVE_LIDAR_READS){
			successive_lidar_detections += 1;

			// Increase traffic count
			data.lidar_count++;
			Log.Info(P("Traffic count - Lidar: %d counts"), data.lidar_count);

			// Also send an XBee alert
			trigger_traffic_event();
		}

		else if (successive_lidar_detections < MIN_SUCCESSIVE_LIDAR_READS){
			successive_lidar_detections += 1;
		}
	}

	else{
		successive_lidar_detections = 0;
	}
}


/* PIR */
void start_pir(){
	/**
	* Start the PIR motion sensor for motion detections
	*/
	pinMode(PIR_MOTION_PIN, INPUT);

	Log.Info(P("Calibrating motion sensor - wait %d ms"), MOTION_INITIALISATION_TIME);

	// Stop everything until the PIR sensor has had a chance to settle
	for (long i = 0; i < MOTION_INITIALISATION_TIME; i += (MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS)){
		Log.Debug(P("Motion sensor calibration: %d ms remaining..."), (MOTION_INITIALISATION_TIME - i));
		Serial.flush();
		delay(MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS);
	}

	pir_timer = timer.setInterval(CHECK_MOTION_INTERVAL, update_pir);
	Log.Info(P("Motion started"));
	data.pir_count = 0;
	data.last_pir_status = false;

	update_pir();
}


void update_pir(){
	/**
	* Check the PIR sensor for detected movement
	* The sensor will output HIGH when motion is detected.
	* Detections will hold the detection status HIGH until the cool-down has lapsed (default: 2s)
	*/
	bool motion_detected = false;
	if (digitalRead(PIR_MOTION_PIN) == MOTION_DETECTED){
		motion_detected = true;

		Log.Debug(P("Motion detected"));
		data.pir_count++;
		Log.Info(P("Traffic count - PIR: %l counts"), data.pir_count);

		trigger_traffic_event();
		enter_pir_cooldown();

	}

	data.last_pir_status = motion_detected;
}


void enter_pir_cooldown(){
	/**
	* Temporarily pause checking the PIR motion sensor
	* The resume_pir_detection callback will trigger at the end of the timeout
	*/
	timer.disable(pir_timer);
	timer.setTimeout(MOTION_COOLDOWN, resume_pir_detection);
}


void resume_pir_detection(){
	/**
	* Start polling the PIR motion sensor
	*/
	timer.enable(pir_timer);
}


/* Air Temperature */
void start_air_temperature(){
	/**
	* Start the air temperature sensor
	*/

	if (TEMPERATURE_SENSOR == TMP36){
		pinMode(TEMPERATURE_PIN, INPUT);
	}
	else if (TEMPERATURE_SENSOR == DS18B20){ // DS18B20
		air_temperature_sensor.begin();
		air_temperature_sensor.setResolution(TEMPERATURE_RESOLUTION);
	}
	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_air_temperature);

	update_air_temperature();
}


void update_air_temperature(){
	/**
	* Read the air temperature into the current entry
	*/
	data.air_temperature = get_air_temperature();
}


float get_air_temperature(){
	/**
	* Read the air temperature sensor
	* :return: Air temperature in degC
	*/
	float air_temp;

	if (TEMPERATURE_SENSOR == TMP36){
		float temp_voltage = (analogRead(TEMPERATURE_PIN) * AREF_VOLTAGE) / 1024;
		air_temp = (temp_voltage - 0.5) * 100;
	}
	else{
		air_temperature_sensor.requestTemperatures();
		air_temp = air_temperature_sensor.getTempCByIndex(0);
	}

	return air_temp;
}


/* Road Temperature */
void start_road_temperature(){
	/**
	* Start the road temperature sensor
	*/
	road_temperature_sensor.begin();
	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_road_temperature);

	update_road_temperature();
}


void update_road_temperature(){
	/**
	* Read the road temperature into the entry
	*/
	data.road_temperature = get_road_temperature();
}


float get_road_temperature(){
	/**
	* Read the road temperature from the non-contact thermometer
	 :return: Object temperature in deg C
	*/
	return road_temperature_sensor.readObjectTempC();
}


/* Case Temperature */
void start_case_temperature(){
	/**
	* Start the road temperature sensor
	*/
	road_temperature_sensor.begin();
	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_case_temperature);

	update_case_temperature();
}


void update_case_temperature(){
	/**
	* Read the road temperature into the entry
	*/
	data.case_temperature = getCaseTemperature();
}


float getCaseTemperature(){
	/**
	* Read the internal case temperature from the non-contact thermometer
	* :return: Case temperature in deg C/
	*/
	return road_temperature_sensor.readAmbientTempC();
}


/* Humidity */
void start_humidity(){
	/**
	* Start the HIH4030 humidity sensor
	*/
	pinMode(HUMIDITY_PIN, INPUT);
	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_humidity);

	update_humidity();
}


void update_humidity(){
	/**
	* Timer callback function
	* Read the humidity into the current entry
	*/
	data.humidity = get_humidity(get_air_temperature());
}


float get_humidity(float air_temperature){
	/**
	* Calculate the humidity from the sensor
	* :air_temperature: The current air temperature in deg C - used to calculate humidity
	* :return: Relative humidity in %
	*/
	int raw_humidity = analogRead(HUMIDITY_PIN);
	float humidity_voltage = (raw_humidity * AREF_VOLTAGE) / 1024.0;

	// See HIH4030 for conversion formula
	float uncalibrated_humidity = 161.0 * humidity_voltage / AREF_VOLTAGE - 25.8;
	float real_humidity = uncalibrated_humidity / (1.0546 - 0.0026 * air_temperature);

	return real_humidity;
}


/* Illuminance */
void start_illuminance(){
	/**
	* Start the BH1750FVI illuminance sensor
	*/
	illuminance_sensor.begin();
	illuminance_sensor.SetAddress(Device_Address_L);
	illuminance_sensor.SetMode(Continuous_H_resolution_Mode);

	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_illuminance);

	update_illuminance();
}


void update_illuminance(){
	/**
	* Timer callback function
	* Read the illuminance into the current entry
	*/
	data.illuminance = get_illuminance();
}


int get_illuminance(){
	/**
	* Read the illuminance from the sensor
	* :return: illuminance value in lux
	*/
	return illuminance_sensor.GetLightIntensity();
}


/* Noise */
void start_noise_monitoring(){
	/**
	* Start the microphone for sound pressure monitoring
	*/
	pinMode(MICROPHONE_PIN, INPUT);

	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_noise_level);
}


void update_noise_level(){
	/**
	* Read the sound level into the current entry
	*/
	data.noise_level = get_noise_level(SOUND_SAMPLE_TIME);
}


int get_noise_level(int sample_period){
	/**
	* Get the average sound level over the specified sampling period
	* :samplePeriod: Listening period for the sampling in ms.
	* :return: Average sound level in 10-bit counts
	*/
	unsigned long start_time = millis();
	long total = 0;
	long count = 0;

	while (millis() < (start_time + sample_period)){
		int soundLevel = analogRead(MICROPHONE_PIN);
		total += soundLevel;
		count += 1;
	}

	int average = int(total / count);
	return average;
}


/* Current */
void start_current_monitor(){
	/**
	* Start the split-core current sensor
	*/
	current_monitor.current(CURRENT_DETECT_PIN, CURRENT_CALIBRATION_FACTOR);

	timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_current_draw);
}


void update_current_draw(){
	/**
	* Read the current draw into the current entry
	*/
	data.current_draw = get_current_draw();
}


float get_current_draw(){
	/**
	* Get the current draw from the clamp
	* :return: Current draw in amperes
	*/
	return current_monitor.calcIrms(CURRENT_SAMPLES);
}


/* RTC */
void start_rtc(){
	/**
	* Start up the real-time clock
	*/
	rtc.begin();
}


void update_timestamp(){
	/**
	* Read the timestamp into the current entry
	*/
	data.timestamp = get_timestamp();
}


long get_timestamp(){
	/**
	* Get the current timestamp.
	* :return: Long timestamp format (seconds since 2000-01-01)
	*/
	DateTime now = rtc.now();
	return now.secondstime();
}


/* Lamp control */
void enable_lamp_control(){
	/**
	* Start lamp control
	* The lamp will start off in the active state for debugging and quick testing
	*/
	lamp.control_enabled = true;
	lamp.current_level = 0;
	activate_lamp();
};


void activate_lamp(){
	/**
	* Switch the lamp to the active state
	* The lamp will go back to the inactive state after the cooldown has elapsed.
	* Successive activation reset the cooldown timer.
	*/
	set_lamp_target(ACTIVE_BRIGHTNESS);
	lamp.timeout = 15;

	// Start the lamp cooldown timer; reset if lamp is already active
	if (lamp.timer_id > -1){
		timer.deleteTimer(lamp.timer_id);
	}
	lamp.timeout = LAMP_ACTIVE_TIME + 1;
	update_lamp_timeout();
}


void update_lamp_timeout(){
	/**
	* Timer callback function - every second
	* Decrement the lamp timeout
	* The lamp will deactivate once the timeout reaches 0
	*/
	lamp.timeout--;

	if (lamp.timeout > 0){
		lamp.timer_id = timer.setTimeout(1000, update_lamp_timeout);
	}
	else{
		deactivate_lamp();
	}
}


void deactivate_lamp(){
	/**
	* Set the lamp to the inactive state
	*/
	set_lamp_target(INACTIVE_BRIGHTNESS);

	// Trash the timer
	timer.deleteTimer(lamp.timer_id);
	lamp.timer_id = -1;
}


void set_lamp_target(int level){
	/**
	* Set the brightness target for the lamp
	* Lamp brightness changes with a transition; changes are not instantaneous
	* :level: brightness in 8-bit levels
	*/

	// Make sure the level is in the percentage range
	lamp.target = constrain(level, 0, 255);
	transition_lamp();
}


void write_lamp_level(int level){
	/**
	* Write the lamp level; instantaneous change
	* :level: Lamp level in 8-bit (0-255; 255 == maximum)
	*/
	lamp.current_level = constrain(level, 0, 255);
	analogWrite(LAMP_CONTROL_PIN, lamp.current_level);
}


void transition_lamp(){
	/**
	* Step the lamp output towards the target output.
	* Time between transition steps and step size changes with direction.
	* e.g. Transitions from low to high can be set fast (high step size; low transition period)
	* Where transitions from high to low can be slower and softer (low step size w/ long period between steps)
	*/
	byte target = lamp.target;
	byte output = lamp.current_level;
	byte new_output;
	long transition_time;

	// If the output is already close to the target, instantly change the output to match
	if ((target >= output && (target - output) <= LAMP_TRANSITION_STEP_UP) ||
	(target <= output && (output - target) <= LAMP_TRANSITION_STEP_DOWN)){
		write_lamp_level(target);
	}

	// Else, a transition is needed; find out what the new output and period needs to be
	else{
		if (target > output){
			new_output = output + LAMP_TRANSITION_STEP_UP;
			transition_time = LAMP_TRANSITION_UP_PERIOD;
		}
		else{
			new_output = output - LAMP_TRANSITION_STEP_DOWN;
			transition_time = LAMP_TRANSISTION_DOWN_PERIOD;
		}

		// Apply next transition output and reschedule a transition (period depending on direction)
		write_lamp_level(new_output);
		timer.setTimeout(transition_time, transition_lamp);
	}
}


/* Bluetooth Scanner */
void start_bluetooth_scanner(){
	/** Start the bluetooth scanner
	* Initialise the bluetooth module in AP -> Master mode to scan for passing devices.
	* Inquiry time is configurable and the results can be filtered by BT class.
	*/

    Log.Debug(P("Starting Bluetooth at %d baud"), BLUETOOTH_BAUD);
    bluetooth.begin(BLUETOOTH_BAUD);
    bluetooth.listen();

	// Put BT module into master mode for scanning
    Log.Debug(P("Entering config..."));
    bluetooth.write(P("$$$"));
    bluetooth.flush();
    Log.Debug(P("Response: %s\n"), bluetooth.readString().c_str());

    Log.Debug(P("Setting master mode..."));
    bluetooth.write(P("SM,1\n"));
    bluetooth.flush();
    Log.Debug(P("Response: %s\n"), bluetooth.readString().c_str());

	// Set up regular scanning intervals
    timer.setInterval(BLUETOOTH_SCAN_INTERVAL, start_bluetooth_scan);
}


void start_bluetooth_scan(){
	/** Initiliase a bluetooth device scan
	* Scan for nearby bluetooth networks.
	* The module will produce a list of devices when the specified scan duration has elapsed.
	*/

	// Start the scan
    Log.Debug(P("Starting scan"));
    bluetooth.write(P("I,5\n"));
    bluetooth.flush();
    Log.Debug(P("Response: %s"), bluetooth.readString().c_str());

	// Create a callback to grab the list when the module is finished scanning
    timer.setTimeout(BLUETOOTH_SCAN_TIME_MS + 1000, check_bluetooth_scan);

}


void check_bluetooth_scan(){
	/** Grab the deteced device list from the bluetooth module
	* The scan yields the BT address and network name.
	* @param device_list Buffer to hold list of detected devices
	*/
	if (Log.getLevel() > LOG_LEVEL_NOOUTPUT){
		USE_SERIAL.print(PACKET_START);
		USE_SERIAL.print(bluetooth.readString().c_str());
		USE_SERIAL.println(PACKET_END);
		USE_SERIAL.flush();
	}
}
