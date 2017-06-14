#include <Arduino.h>
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
PIR narrow(NARROW_PIR_PIN, NARROW_PIR_COOLDOWN, MOUNTING_HEIGHT, NARROW_PIR_FOV, "narrow");
PIR wide_left(WIDE_PIR_PIN_L, WIDE_PIR_COOLDOWN, MOUNTING_HEIGHT, WIDE_PIR_FOV, "left");
PIR wide_right(WIDE_PIR_PIN_R, WIDE_PIR_COOLDOWN, MOUNTING_HEIGHT, WIDE_PIR_FOV, "right");

// Objects
SensorEntry data;

////////////////////////////////////////////////////////////////////////////////
/* Arduino main functions */

void setup() {
    /**
    * Startup code - Run once
    */

    start_yun_serial();
    Log.Info(P("Traffic Counter - ver %s"), SSL_TESTBED_VERSION);

    if (REAL_TIME_CLOCK_ENABLED) {
        start_rtc();
    }

    if (XBEE_ENABLED) {
        start_xbee();
    }

    if (LIDAR_ENABLED) {
        start_lidar();
    }

    if (PIR_ENABLED) {
        start_pir();
    }

    if (BLUETOOTH_ENABLED) {
        start_bluetooth_scanner();
    }

    timer.setInterval(PRINT_INTERVAL, print_heartbeat_message);
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

void print_heartbeat_message() {
    Log.Debug(P("%c{\"id\":\"%s\",\"height\":%l,\"ver\":%s,\"trip_ver\":%s,\"pir_ver\":%s}%c"), PACKET_START, UNIT_NAME,
              lidar_tripwire.baseline_distance, SSL_TESTBED_VERSION, TRIPWIRE_VERSION, PIR_VERSION, PACKET_END);
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
    narrow.set_event_start_callback(pir_event_start);
    narrow.set_event_end_callback(pir_event_end);
    narrow.set_cooldown_over_callback(pir_cooldown_end);

    wide_left.set_trigger_state(HIGH);
    wide_left.set_event_start_callback(pir_event_start);
    wide_left.set_event_end_callback(pir_event_end);
    wide_left.set_cooldown_over_callback(pir_cooldown_end);
    wide_left.start();

    wide_right.set_trigger_state(HIGH);
    wide_right.set_event_start_callback(pir_event_start);
    wide_right.set_event_end_callback(pir_event_end);
    wide_right.set_cooldown_over_callback(pir_cooldown_end);
    wide_right.start();

    Log.Info(P("Calibrating motion sensors..."));
    narrow.calibrate(MOTION_INITIALISATION_TIME);

    // Start up regular reads
    timer.setInterval(CHECK_MOTION_INTERVAL, update_pir);
    Log.Info(P("Motion started"));

    update_pir();
}

void pir_event_start(PIR sensor) {
    /**
    * Callback - Determine what traffic has caused the event based on timing
    */
    Log.Debug(P("%c{\"id\":\"%s\",\"type\":\"start\",\"count\":%l}%c"), PACKET_START, sensor.name,
              sensor.num_detections, PACKET_END);
}

void pir_event_end(PIR sensor) {
    /**
    * Callback - trigger a traffic notification at the end of a narrow event (event, not detection)
    * The callback at the end makes sure the event width is recorded
    */

    Log.Info(P("%c{\"id\":\"%s\",\"type\":\"end\",\"count\":%l, \"time\":%l}%c"), PACKET_START, sensor.name,
             sensor.num_detections, sensor.event_width, PACKET_END);
}

void pir_cooldown_end(PIR sensor) {
    Log.Debug(P("%c{\"id\":\"%s\",\"type\":\"cool\",\"count\":%l}%c"), PACKET_START, sensor.name, sensor.num_detections,
              PACKET_END);
}

void update_pir() {
    /**
    * Check the motion sensors and update detection data
    */
    narrow.update();
    wide_left.update();
    wide_right.update();
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

    lidar_tripwire.cooldown_time = LIDAR_COOLDOWN;
    lidar_tripwire.distance_threshold = LIDAR_DETECT_THRESHOLD;
    lidar_tripwire.max_baseline_variance = BASELINE_VARIANCE_THRESHOLD;
    lidar_tripwire.min_successive_detections = MIN_SUCCESSIVE_LIDAR_READS;
    lidar_tripwire.min_baseline_reads = MIN_BASELINE_READS;
    lidar_tripwire.max_baseline_reads = MAX_BASELINE_READS;
    lidar_tripwire.baseline_read_interval = BASELINE_READ_INTERVAL;
    lidar_tripwire.min_detect_distance = TRIPWIRE_MIN_DETECT_DISTANCE;
    lidar_tripwire.max_detect_distance = TRIPWIRE_MAX_DETECT_DISTANCE;

    lidar_tripwire.start();
    Log.Debug(P("Lidar - Establishing baseline range..."));
    lidar_tripwire.calibrate();
    Log.Debug(P("Lidar - Baseline range: %d\tvariance: %d"), lidar_tripwire.baseline_distance,
              lidar_tripwire.baseline_variance);

    if (lidar_tripwire.is_calibrated) {
        lidar_tripwire.set_event_start_callback(lidar_start_event);
        lidar_tripwire.set_event_end_callback(lidar_end_event);
        lidar_tripwire.set_cooldown_over_callback(lidar_cooldown_event);
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

void lidar_start_event() {
    Log.Debug(P("%c{\"id\":\"lidar\",\"type\":\"start\",\"count\":%l, \"distance\":%l}%c"), PACKET_START,
              lidar_tripwire.num_detections, lidar_tripwire.distance, PACKET_END);
}

void lidar_end_event() {
    Log.Info(P("%c{\"id\":\"lidar\",\"type\":\"end\",\"count\":%l, \"distance\":%l, \"time\":%l, "
               "\"height\":%l, \"reads\":%d}%c"),
             PACKET_START, lidar_tripwire.num_detections, lidar_tripwire.distance, lidar_tripwire.last_event_width,
             lidar_tripwire.average_target_height, lidar_tripwire.active_event_reads, PACKET_END);
}

void lidar_cooldown_event() {
    Log.Debug(P("%c{\"id\":\"lidar\",\"type\":\"cool\",\"count\":%l, \"distance\":%l}%c"), PACKET_START,
              lidar_tripwire.num_detections, lidar_tripwire.distance, PACKET_END);
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
