#include <Arduino.h>
#include <ArduinoJson.h>
#include <LIDARduino.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Logging.h"
#include "PIR.h"
#include "ProgmemString.h"
#include "SimpleTimer.h"
#include "Tripwire.h"

#include "config.h"

// XBee - in Transparent (Serial bridge) mode
SoftwareSerial xbee_bridge(COMM_1_RX, COMM_1_TX);

// Bluetooth module
SoftwareSerial bluetooth(COMM_2_RX, COMM_2_TX);

// Traffic Sensors
LIDAR_Lite_PWM lidar(LIDAR_TRIGGER_PIN, LIDAR_PWM_PIN);
Tripwire lidar_tripwire(get_lidar_range);
static int successive_lidar_detections = 0;

// Misc
RTC_DS3231 rtc;

char bluetooth_buffer[BLUETOOTH_BUFFER_SIZE];
int bluetooth_index = 0;
bool bluetooth_recording = false;

// Timers
SimpleTimer timer;

// PIR variables
PIR narrow(NARROW_PIR_PIN, NARROW_PIR_COOLDOWN, MOUNTING_HEIGHT, NARROW_PIR_FOV);
PIR wide_left(WIDE_PIR_PIN_L, WIDE_PIR_COOLDOWN, MOUNTING_HEIGHT, WIDE_PIR_FOV);
PIR wide_right(WIDE_PIR_PIN_R, WIDE_PIR_COOLDOWN, MOUNTING_HEIGHT, WIDE_PIR_FOV);

bool pir_recording_event = false;
bool pir_clear = true;
bool pedestrian_timeout_active = false;

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
    entry["npir"] = narrow.num_detections;
    entry["wpir_l"] = wide_left.num_detections;
    entry["wpir_r"] = wide_right.num_detections;
    entry["cool_n"] = narrow.is_in_cooldown;
    entry["cool_l"] = wide_left.is_in_cooldown;
    entry["cool_r"] = wide_right.is_in_cooldown;
    entry["lidar"] = lidar_tripwire.num_detections;
    entry["l_range"] = lidar_tripwire.distance;
    entry["ped"] = data.pir_pedestrian;
    entry["veh"] = data.pir_vehicle;
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
    narrow.set_trigger_state(HIGH);
    narrow.start();

    wide_left.set_trigger_state(HIGH);
    wide_left.start();

    wide_right.set_trigger_state(HIGH);
    wide_right.start();

    narrow.set_event_start_callback(narrow_event_start);
    narrow.set_event_end_callback(narrow_event_end);

    wide_left.set_event_start_callback(wide_event_start);
    wide_right.set_event_start_callback(wide_event_start);
    wide_left.set_cooldown_over_callback(wide_cooldown_finished);
    wide_right.set_cooldown_over_callback(wide_cooldown_finished);

    Log.Info(P("Calibrating motion sensors..."));
    narrow.calibrate(MOTION_INITIALISATION_TIME);

    // Start up regular reads
    timer.setInterval(CHECK_MOTION_INTERVAL, update_pir);
    Log.Info(P("Motion started"));

    update_pir();
}

void narrow_event_start() {
    /**
    * Callback - Determine what traffic has caused the event based on timing
    */

    Log.Debug("Narrow PIR event started");
    if (pir_recording_event) {
        // Calculte the speed if motion has been triggered from outside already
        if (!pedestrian_timeout_active) {
            float distance = wide_left.radius - narrow.radius;
            unsigned long time = get_most_recent_wide_event() - narrow.detection_start_time;
            float speed = distance / (time / 1000);  // m/s
            speed *= 3.6;

            if (speed > (2 * WALKING_SPEED)) {
                data.pir_vehicle++;
                data.pir_pedestrian--;

            } else {
                pedestrian_timeout_active = true;
                unsigned long walking_timeout = (narrow.radius / (WALKING_SPEED / 3.6) * 1000);
                timer.setTimeout(walking_timeout, cancel_pedestrian_timeout);
            }
        }
    } else {
        data.pir_error++;
    }

    trigger_traffic_event();
}

void cancel_pedestrian_timeout() {
    /**
    * Cancel the pedestrian timeout - stops the narrow sensor from overcounting
    */
    pedestrian_timeout_active = false;
}

unsigned long get_most_recent_wide_event() {
    /**
    * Get the time of the most recent detection from the wide sensors
    */
    unsigned long event_time = 0;

    if (wide_left.detection_start_time > event_time) {
        event_time = wide_left.last_untriggered_time;
    }
    if (wide_right.detection_start_time > event_time) {
        event_time = wide_right.last_untriggered_time;
    }

    return event_time;
}

void narrow_event_end() {
    /**
    * Callback - trigger a traffic notification at the end of a narrow event (event, not detection)
    * The callback at the end makes sure the event width is recorded
    */
    trigger_traffic_event();
}

void wide_event_start() {
    /**
    * Callback - Start recording a timed event when the wide sensors are triggered
    */

    Log.Debug("Wide PIR event started");

    if (pir_recording_event) {
        pir_recording_event = false;
        long time_between_wide_detections = (abs(wide_left.last_untriggered_time - wide_right.last_untriggered_time));

        long maximum_vehicle_time = long(wide_left.radius * 1000) / float((2 * WALKING_SPEED) / 3.6);
        if (time_between_wide_detections < maximum_vehicle_time) {
            data.pir_vehicle++;
        }
    }

    else {
        pir_recording_event = true;
        data.pir_pedestrian++;
    }

    trigger_traffic_event();
}

void wide_cooldown_finished() {
    /**
    * Callback - Stop recording events when the cooldown period is over
    */
    pir_recording_event = false;
}

void update_pir() {
    /**
    * Check the motion sensors and update detection data
    */
    pir_clear = is_pir_clear();

    narrow.update();
    wide_left.update();
    wide_right.update();
}

bool is_pir_clear() {
    /**
    * Check if motion sensors are currently detecting events
    * @return True if no events are currently in progress
    */
    return !(narrow.state && wide_left.state && wide_right.state);
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

    lidar_tripwire.distance_threshold = LIDAR_DETECT_THRESHOLD;
    lidar_tripwire.max_baseline_variance = BASELINE_VARIANCE_THRESHOLD;
    lidar_tripwire.min_successive_detections = MIN_SUCCESSIVE_LIDAR_READS;
    lidar_tripwire.min_baseline_reads = MIN_BASELINE_READS;
    lidar_tripwire.max_baseline_reads = MAX_BASELINE_READS;
    lidar_tripwire.baseline_read_interval = BASELINE_READ_INTERVAL;

    lidar_tripwire.start();
    Log.Debug(P("Lidar - Establishing baseline range..."));
    lidar_tripwire.calibrate();
    Log.Debug(P("Lidar - Baseline range: %d\tvariance: %d"), lidar_tripwire.baseline_distance,
              lidar_tripwire.baseline_variance);

    if (lidar_tripwire.is_calibrated) {
        lidar_tripwire.set_event_start_callback(narrow_event_start);
        timer.setInterval(LIDAR_CHECK_RANGE_INTERVAL, update_lidar);
    }

    lidar_tripwire.update();
}

long get_lidar_range() {
    /**
    * Get the range in cm from the lidar
    * :return: Target distance from sensor in cm
    */
    int target_distance = lidar.getDistance();
    Log.Verbose(P("Lidar Range: %d cm"), target_distance);
    return long(target_distance);
}

void update_lidar() { lidar_tripwire.update(); }

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
    sprintf(buffer, P("%04d-%02d-%02d %02d:%02d:%02d"), now.year(), now.month(), now.day(), now.hour(), now.minute(),
            now.second());
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
    Log.Info(P("%c{%s\"%s\"}%c"), PACKET_START, BLUETOOTH_HEADER, bluetooth_buffer, PACKET_END);
}
