#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

#include <ArduinoJson.h>
#include <LIDARduino.h>
#include "MLX90621.h"
#include "ProgmemString.h"
#include "Logging.h"
#include "SimpleTimer.h"

#include "config.h"

// XBee - in Transparent (Serial bridge) mode
SoftwareSerial xbee_bridge(COMM_1_RX, COMM_1_TX);

// Bluetooth module
SoftwareSerial bluetooth(COMM_2_RX, COMM_2_TX);

// Traffic Sensors
LIDAR_Lite_PWM lidar(LIDAR_TRIGGER_PIN, LIDAR_PWM_PIN);
static int successive_lidar_detections = 0;
MLX90621 thermal_flow_sensor;

// Misc
RTC_DS3231 rtc;

char bluetooth_buffer[BLUETOOTH_BUFFER_SIZE];
int bluetooth_index = 0;
bool bluetooth_recording = false;

// Timers
SimpleTimer timer;

// PIR variables
bool cooldown_active[NUM_PIR_SENSORS];
bool motion_status[NUM_PIR_SENSORS];
unsigned long motion_start_time[NUM_PIR_SENSORS];

// Objects
SensorEntry data;

////////////////////////////////////////////////////////////////////////////////
/* Arduino main functions */
void setup() {
  /**
  * Startup code - Run once
  */
  data.id = UNIT_NAME;
  data.version = SSL_TESTBED_VERSION;

   start_yun_serial();
  Log.Init(LOG_LEVEL_DEBUG, 57600);
  Log.Info(P("Traffic Counter - ver %d"), SSL_TESTBED_VERSION);

  if (REAL_TIME_CLOCK_ENABLED) {
    start_rtc();
  }

  if (XBEE_ENABLED) {
    start_xbee();
  }

  start_sensors();

  if (BLUETOOTH_ENABLED) {
    start_bluetooth_scanner();
  }
}

void loop() {
  /**
  * Main loop - run forever
  */
  timer.run();
}

////////////////////////////////////////////////////////////////////////////////
/* Yun Serial */
void start_yun_serial() {
  /**
  * Set up the Arduino-Linux serial bridge
  * Serial output from the Arduino is disabled until the Yun has finished
  * booting
  * The serial can be accessed safely after the boot debug finishes.
  * Any serial transmission during this time would halt start-up, which would
  * suck.
  */

  // Set up the handshake pin
  pinMode(YUN_HANDSHAKE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Give a delay for the AR9331 to reset, in case we were reset as part of a
  // reboot command.
  // See http://playground.arduino.cc/Hardware/Yun#rebootStability, case 3.
  delay(YUN_BOOT_DELAY);

  // If the system is booting, wait for UBoot to finish (Shamelessly adapted
  // from Bridge.cpp)
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

void boot_status_change_ISR() {
  /**
  * ISR - Called when the handshake pin (D7) is changed by the AR9331
  * Toggle serial output depending on the boot state of the Yun.
  * If the handshake pin changes, the boot status of the Yun has changed.
  * Handshake == LOW means the linux environment is ready for serial
  */
  bool linux_busy = digitalRead(YUN_HANDSHAKE_PIN);

  if (linux_busy) {
    // Disable log output until Linux boots
    Log.Init(LOG_LEVEL_NOOUTPUT, &USE_SERIAL);
  } else {
    Log.Init(LOGGER_LEVEL, &USE_SERIAL);
  }
}

////////////////////////////////////////////////////////////////////////////////
/* XBee Comms */
void start_xbee() {
  /**
  * Start up xbee communication
  * XBee is connected to software serial
  * XBee communication uses AT mode - which just uses the XBee as a serial
  * bridge.
  * There is no packet handling and all data travels one way: out
  */
  xbee_bridge.begin(XBEE_BAUD);
}

////////////////////////////////////////////////////////////////////////////////
/* Printing */
void start_sensors() {
  /**
  * Enable and configure the sensor suite for reading.
  * A timer is started to regularly print sensor data
  */

  // Environment
  if (THERMO_FLOW_ENABLED) {
    start_thermal_flow();
    start_case_temperature();
  }

  if (LIDAR_ENABLED) {
    start_lidar();
  }

  if (PIR_ENABLED) {
    start_pir();
  }

  // Start timer for regular entry transmission
  timer.setInterval(PRINT_INTERVAL, print_regular_entry);
}

void print_regular_entry() {
  /**
  * Print a normal data entry
  * Regular entries are triggered by timer, rather than sensor
  * The traffic event flag is 'off'
  */
  data.event_flag = false;
  print_data();
}

void trigger_traffic_event() {
  /**
  * Print a data entry
  * The traffic event flag is enabled
  */
  data.event_flag = true;
  print_data();
}

void print_data() {
  /**
  * Print the current traffic counts and info to Serial
  */
  if (REAL_TIME_CLOCK_ENABLED) {
    update_timestamp();
  }

  print_json_string();
}

void print_json_string() {
  /**
  * Format the data packet into a JSON string
  * @return JSON-formatted string containing selected data
  */
  StaticJsonBuffer<COMM_BUFFER_SIZE> print_buffer;
  JsonObject &entry = print_buffer.createObject();

  entry["ver"] = data.version;
  entry["id"] = data.id;
  entry["npir"] = data.pir_count[PIR_NARROW];
  entry["wpir_l"] = data.pir_count[PIR_WIDE_LEFT];
  entry["wpir_r"] = data.pir_count[PIR_WIDE_RIGHT];
  entry["cool_n"] = data.pir_in_cooldown[PIR_NARROW];
  entry["cool_l"] = data.pir_in_cooldown[PIR_WIDE_LEFT];
  entry["cool_r"] = data.pir_in_cooldown[PIR_WIDE_RIGHT];
  entry["lidar"] = data.lidar_count;
  entry["l_range"] = data.lidar_range;
  entry["t_case"] = data.case_temperature;
  entry["timestamp"] = data.timestamp;

  if (Log.getLevel() > LOG_LEVEL_NOOUTPUT) {
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

////////////////////////////////////////////////////////////////////////////////
/* PIR */
void start_pir() {
  /**
  * Start the PIR motion sensor for motion detections
  */

  // Set up input pins for PIR sensors
  for (int i = 0; i < NUM_PIR_SENSORS; i++) {
    pinMode(PIR_PINS[i], INPUT);
    motion_status[i] = false;
    motion_start_time[i] = 0;
    data.pir_count[i] = 0;
  }

  // Stop everything until the PIR sensor has had a chance to settle
  warm_up_pir_sensors();

  // Start up regular reads
  timer.setInterval(CHECK_MOTION_INTERVAL, update_pir);
  Log.Info(P("Motion started"));

  update_pir();
}

void warm_up_pir_sensors(){
    /**
    * Halt the system until the PIR sensors have had a chance to start.
    * The downtime is goverened by the MOTION_INITIALISATION_TIME variable
    * Default: 10 seconds
    */
    Log.Info(P("Calibrating motion sensor - wait %d ms"), MOTION_INITIALISATION_TIME);

    for (long i = 0; i < MOTION_INITIALISATION_TIME;
         i += (MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS)) {
      Log.Debug(P("Motion sensor calibration: %d ms remaining..."), (MOTION_INITIALISATION_TIME - i));
      USE_SERIAL.flush();
      delay(MOTION_INITIALISATION_TIME / MOTION_INITIALISATION_INTERVALS);
    }
}

void update_pir() {
  /**
  * Check the PIR sensor for detected movement
  * The sensor will output HIGH when motion is detected.
  * Detections will hold the detection status HIGH until the cool-down has
  * lapsed (default: 2s)
  */

  // Check all PIR sensors (both narrow and wide)
  for (int i = 0; i < NUM_PIR_SENSORS; i++) {

    // Check cooldown status
    if (data.pir_in_cooldown[i]) {
      if (millis() - motion_start_time[i] > PIR_COOLDOWNS[i]) {
        data.pir_in_cooldown[i] = false;
      }
    }

    // Check for motion if not in cooldown
    if (!data.pir_in_cooldown[i]) {
      if (digitalRead(PIR_PINS[i]) == PIR_ACTIVE_STATES[i]) {
        if (!PIR_MUST_GO_LOW_BETWEEN_TRIGGERS || !motion_status[i]) {

          // If motion detected, increment the count (and do all the extra bits)
          motion_status[i] = true;
          motion_start_time[i] = millis();
          data.pir_in_cooldown[i] = true;
          increment_pir_count(i);
          Log.Info("Sensor %d triggered. Count: %l", i, data.pir_count[i]);
        }

        else {
          if (motion_status[i]) {
            motion_status[i] = false;
          }
        }
      }
    }
  }
}

void increment_pir_count(int sensor_num) {

    // Narrow sensor detection
    if (PIR_TYPES[sensor_num] == PIR_NARROW) {
        increment_narrow_pir(sensor_num);
    }

    // Wide sensor detection
    else{
        increment_wide_pir(sensor_num);
    }
}

void increment_narrow_pir(int sensor_num){
    // If we're dealing with the narrow sensor, the count should only increment if a wide is already triggered
    bool wide_triggered = false;
    for (int i = 0; i < NUM_PIR_SENSORS; i++) {
        if (PIR_TYPES[i] == PIR_WIDE && data.pir_in_cooldown[i]){
            wide_triggered = true;
        }
    }
    if (wide_triggered){
        data.pir_count[sensor_num]++;
    }
}

void increment_wide_pir(int sensor_num){
    // The wide sensors don't have any self-validation
    data.pir_count[sensor_num]++;

    // However, we still want to check if there's been some flow between the sensors
    unsigned long other_start = 2 * PIR_MAX_FLOW_DELAY;
    unsigned long now = millis();

    // Find out if/when another wide sensor was active
    for (int i = 0; i < NUM_PIR_SENSORS; i++) {
        Log.Debug(P("I: %d %T; Wide? %T; Cooldown? %T"), i, i == sensor_num, PIR_TYPES[i] == PIR_WIDE, data.pir_in_cooldown[i]);
        if (i != sensor_num && PIR_TYPES[i] == PIR_WIDE && data.pir_in_cooldown[i]) {
            Log.Debug(P("Checking sensor %d"), i);

            unsigned long time_since_trigger = now - motion_start_time[i];
            if (other_start > time_since_trigger){
                other_start = time_since_trigger;
            }
        }
    }

    if (PIR_MIN_FLOW_DELAY < other_start && other_start < PIR_MAX_FLOW_DELAY) {
        if (sensor_num == PIR_WIDE_LEFT) {
            // Object travelled from right to left
            Log.Info(P("Leftward travel"));
        }
        else{
            // Object travelled from left to right
            Log.Info(P("Rightward travel"));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/* Lidar */
void start_lidar() {
  /**
  * Start the rangefinder sensor
  * A baseline range (to the ground or static surrounds) is established for
  * comparing against new measuremets.
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
  } else {
    data.lidar_baseline = 0;
    Log.Error(P("Lidar initialisation failed - sensor disabled"));
  }

  update_lidar();
}

int get_lidar_baseline(int variance) {
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
  while ((baseline_reads < MIN_BASELINE_READS || average_variance > variance) &&
         baseline_reads < MAX_BASELINE_READS) {
    int new_range = get_lidar_range();
    int new_variance = abs(average_range - new_range);

    average_variance = ((average_variance + new_variance) / 2);
    average_range = ((average_range + new_range) / 2);

    Log.Debug(P("Lidar Calibration: Range - %d, Variance - %d"),
              int(average_range), average_variance);
    baseline_reads++;
    delay(BASELINE_READ_INTERVAL);
  }

  // Calibration fails if the range is varying too much
  if (average_variance > variance) {
    average_range = -1;
  }

  return average_range;
}

int get_lidar_range() {
  /**
  * Get the range in cm from the lidar
  * :return: Target distance from sensor in cm
  */
  int target_distance = lidar.getDistance();
  Log.Verbose(P("Lidar Range: %d cm"), target_distance);
  return target_distance;
}

void update_lidar() {
  /**
  * Check the lidar to see if a traffic event has occurred.
  * Traffic events are counted as a break in the sensor's 'view' of the ground.
  * Any object between the sensor and the ground baseline will cause the sensor
  * to register a shorter range than usual.
  */
  data.lidar_range = get_lidar_range();

  // Detection occurs when target breaks the LoS to the baseline
  if ((data.lidar_baseline - data.lidar_range) > RANGE_DETECT_THRESHOLD) {

    // If x in a row
    if (successive_lidar_detections == MIN_SUCCESSIVE_LIDAR_READS) {
      successive_lidar_detections += 1;

      // Increase traffic count
      data.lidar_count++;
      Log.Info(P("Traffic count - Lidar: %d counts"), data.lidar_count);

      // Also send an XBee alert
      trigger_traffic_event();
    }

    else if (successive_lidar_detections < MIN_SUCCESSIVE_LIDAR_READS) {
      successive_lidar_detections += 1;
    }
  }

  else {
    successive_lidar_detections = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
/* Thermal Flow */
void start_thermal_flow(){
    Wire.begin();
    thermal_flow_sensor.initialise(THERM_FRAMERATE);

    timer.setInterval(THERM_FRAME_UPDATE_INTERVAL, get_frame);
}

void get_frame(){
    // Prepend the frame packet with it's tags
    USE_SERIAL.print(PACKET_START);
    USE_SERIAL.print(P("{id:thermal, data:"));
    // for (int i = 0; i < 64; i++) {
    //     USE_SERIAL.print(thermal_flow_sensor.irData[i]);
    //     if (i < 63) {
    //         USE_SERIAL.print(",");
    //     }
    // }
    thermal_flow_sensor.print_temperatures(USE_SERIAL);
    USE_SERIAL.print("}\n\n");
}

void start_case_temperature() {
  /**
  * Start the road temperature sensor
  */
  timer.setInterval(CHECK_ENVIRONMENTAL_SENSOR_INTERVAL, update_case_temperature);
  update_case_temperature();
}

void update_case_temperature() {
  /**
  * Read the road temperature into the entry
  */
  // data.case_temperature = get_case_temperature();
  // Log.Info("Ambient temperature: %d.%d degC", int(data.case_temperature), int((data.case_temperature - int(data.case_temperature))*100));
}

float get_case_temperature(){
    return thermal_flow_sensor.get_ambient_temperature();
}

////////////////////////////////////////////////////////////////////////////////
/* RTC */
void start_rtc() {
  /**
  * Start up the real-time clock
  */
  rtc.begin();
  update_timestamp();

  timer.setInterval(SYSTEM_CLOCK_UPDATE_INTERVAL, update_system_clock);
  update_system_clock();
}

void update_timestamp() {
  /**
  * Read the timestamp into the current entry
  */

  get_datetime(data.timestamp);
  Log.Debug(P("Time: %s"), data.timestamp);
}

long get_timestamp() {
  /**
  * Get the current timestamp.
  * :return: Long timestamp format (seconds since 2000-01-01)
  */
  DateTime now = rtc.now();
  return now.secondstime();
}

void get_datetime(char *buffer) {
  /** Get the datetime in string format
  * DateTime follows the standard ISO format ("YYYY-MM-DD hh:mm:ss")
  * @param buffer Character buffer to write datetime to; Must be at least 20
  * char wide.
  */
  DateTime now = rtc.now();

  // Get the DateTime into the standard, readable format
  sprintf(buffer, P("%04d-%02d-%02d %02d:%02d:%02d"), now.year(), now.month(),
          now.day(), now.hour(), now.minute(), now.second());
}

void update_system_clock() {
  /**
  * Update the system clock of the Yun to match the RTC
  */
  DateTime now = rtc.now();
  uint32_t unix_time = now.unixtime();

  const char TIMESTAMP_HEADER[] = "\"id\":\"Time\",\"data\":";
  Log.Info("%c{%s%l}%c", PACKET_START, TIMESTAMP_HEADER, unix_time, PACKET_END);
}

////////////////////////////////////////////////////////////////////////////////
/* Bluetooth Scanner */
void start_bluetooth_scanner() {
  /** Start the bluetooth scanner
  * Initialise the bluetooth module in AP -> Master mode to scan for passing
  * devices.
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

void start_bluetooth_scan() {
  /** Initiliase a bluetooth device scan
  * Scan for nearby bluetooth networks.
  * The module will produce a list of devices when the specified scan duration
  * has elapsed.
  */

  read_bluetooth_buffer();
  stop_bluetooth_read();

  // Start the scan
  Log.Debug(P("Starting scan"));
  bluetooth.write(P("I,"));
  bluetooth.println(BLUETOOTH_SCAN_TIME);
  bluetooth.flush();

  Log.Debug(P("Response: %s"), bluetooth.readString().c_str());

  // Create a callback to grab the list when the module is finished scanning
  timer.setTimeout(BLUETOOTH_SCAN_TIME_MS, start_bluetooth_read);
}

void start_bluetooth_read() {
  bluetooth_recording = true;
  bluetooth_index = 0;
  bluetooth.listen();
  read_bluetooth_buffer();
}

void read_bluetooth_buffer() {
  if (bluetooth_recording) {

    Log.Verbose("Buffer: %d", bluetooth_index);

    while (bluetooth.available()) {

      String string = bluetooth.readString().c_str();
      for (size_t i = 0; i < string.length(); i++) {
        if (bluetooth_index < BLUETOOTH_BUFFER_SIZE) {
          bluetooth_buffer[bluetooth_index] = string[i];
          bluetooth_index++;

        } else {
          bluetooth_index = BLUETOOTH_BUFFER_SIZE - 1;
        }
      }
    }

    timer.setTimeout(BLUETOOTH_CHECK_INTERVAL, read_bluetooth_buffer);
  }
}

void stop_bluetooth_read() {
  bluetooth_recording = false;
  bluetooth_buffer[bluetooth_index] = '\0';

  const char BLUETOOTH_HEADER[] = "\"id\":\"Blue\",\"data\":";
  Log.Info(P("%c{%s\"%s\"}%c"), PACKET_START, BLUETOOTH_HEADER, bluetooth_buffer,
           PACKET_END);
}
