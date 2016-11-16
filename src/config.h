#include "Arduino.h"
#include <Logging.h>

///////////////////////////////////////////////////////////////////////////////
// Unit
const int SSL_TESTBED_VERSION = 9;
const byte UNIT_ID = 1;
const char UNIT_NAME[] = "Pixie";

// Use Serial for debug; Serial1 for normal operation
#define USE_SERIAL Serial
const int LOGGER_LEVEL = LOG_LEVEL_VERBOSE;

// Misc
const bool REAL_TIME_CLOCK_ENABLED = true;
const bool CURRENT_MONITOR_ENABLED = false;
const bool LAMP_CONTROL_ENABLED = false;

// Comms
const bool BLUETOOTH_ENABLED = true;
const bool XBEE_ENABLED = true;

// Environmental
const bool AIR_TEMPERATURE_ENABLED = false;
const bool HUMIDITY_ENABLED = false;
const bool ILLUMINANCE_ENABLED = false;

// Traffic
const bool PIR_ENABLED = true;
const bool LIDAR_ENABLED = true;
const bool THERMO_FLOW_ENABLED = true;

///////////////////////////////////////////////////////////////////////////////
// Temperature Probe
const byte TEMPERATURE_RESOLUTION = 12; //12-bit temperature resolution
enum TEMPERATURE_SENSORS{
    TMP36 = 1,
    DS18B20 = 2
};
const byte TEMPERATURE_SENSOR = DS18B20;
const float AREF_VOLTAGE = 5.0;


///////////////////////////////////////////////////////////////////////////////
// Comms
const int YUN_BOOT_DELAY = 5000; //Time to wait for Yun to boot up before checking the handshake
const long YUN_LINUX_BAUD_RATE = 250000;
const int COMM_BUFFER_SIZE = 300;
const int BLUETOOTH_BUFFER_SIZE = 300;

const long SERIAL_BAUD = 57600;

// XBee Socket 1
const long XBEE_BAUD = 9600;
enum XBEE_1_COMMS{
    UART = 0,
    SPI = 1
};
const byte XBEE_1_MODE = UART;

// XBee Socket 2 - Bluetooth module
const long BLUETOOTH_BAUD = 9600;


///////////////////////////////////////////////////////////////////////////////
// Pin assignments
const byte YUN_RX = 0;  // Unused Pin - Tied to USB UART
const byte YUN_TX = 1;  // Unused Pin - Tied to USB UART
const byte SPI_ATTN = 4;    // SPI Attention pin for XBEE-SPI comms
const byte SPI_SS = 5;  // SPI Slave Select
const byte LAMP_CONTROL_PIN = 6;    // Lamp control - uses PWM with inverse duty cycle
const byte YUN_HANDSHAKE_PIN = 7;   // Yun handshake - HIGH when Yun is busy/booting
const byte XBEE_SLEEP_PIN = 8;  // XBEE sleep pin; Bring HIGH to allow XBEE comms
const byte WIDE_PIR_PIN = 9;  // PIR motion detect pin; pulled HIGH when motion is detected
const byte COMM_2_TX = 10;  // Send pin for XBEE socket 2; UART-only
const byte COMM_2_RX = 11;  // Receive pin for XBEE socket 2; UART-only
const byte LIDAR_PWM_PIN = 12;  // Data pin for LIDARLite
const byte LIDAR_TRIGGER_PIN = 13;  // PWM Trigger pin for LIDARLite
const byte COMM_1_RX = 14;  // Send pin for XBEE socket 1; UART and SPI compatible
const byte COMM_1_TX = 16;  // Receive pin for XBEE socket 1; UART and SPI compatible

const byte WIDE_PIR_2_PIN = A0;  // Sonar receive pin - can use pulse width or ADC for distance measurements
const byte NARROW_PIR_PIN = A1;
const byte HUMIDITY_PIN = A3;   // Humidity level pin - Gives ADC humidity level
const byte TEMPERATURE_PIN = A4;    // Temperature data pin for DS18B20 digital temperature probe
const byte CURRENT_DETECT_PIN = A5; // Current monitor pin for current clamp - gives offset ADC current wave


///////////////////////////////////////////////////////////////////////////////
// Timer

const int CHECK_FLOW_INTERVAL = 200;
const int FLOW_COOLDOWN = 2000; // Optical flow detection cooldown in ms

const int CHECK_RANGE_INTERVAL = 100;
const int RANGE_DETECT_THRESHOLD = 70; // Minimum threshold for range detection in cm. (Target must move at least this much)

const int LIDAR_CHECK_RANGE_INTERVAL = 100;
const int LIDAR_DETECT_THRESHOLD = 50;

const long PRINT_INTERVAL = 10000;
const long CHECK_ENVIRONMENTAL_SENSOR_INTERVAL = PRINT_INTERVAL/3;
const long XBEE_TRANSMIT_INTERVAL = PRINT_INTERVAL; // Time between XBee transmissions
const long SYSTEM_CLOCK_UPDATE_INTERVAL = 6000;

const int BLUETOOTH_SCAN_TIME = 5;  // Bluetooth scan duration in seconds
const long BLUETOOTH_SCAN_TIME_MS = BLUETOOTH_SCAN_TIME * 1000; // Length of a bluetooth scan in milliseconds
const long BLUETOOTH_SCAN_INTERVAL = BLUETOOTH_SCAN_TIME_MS + 6000; // Time between bluetooth scans in milliseconds
const long BLUETOOTH_CHECK_INTERVAL = 20;  // Time between bluetooth response checks
const long BLUETOOTH_SERIAL_WAIT = 1000;


///////////////////////////////////////////////////////////////////////////////
// Real-time Clock

const int DATETIME_WIDTH = 20;

///////////////////////////////////////////////////////////////////////////////
// Sonar/Lidar
const byte MIN_BASELINE_READS = 20; // Minimum number of reads needed to establish a baseline
const byte MAX_BASELINE_READS = 30; // Maximum number of reads to establishe a baseline
const int BASELINE_READ_INTERVAL = 200; // Time between range baseline calibration reads in milliseconds
const int BASELINE_VARIANCE_THRESHOLD = 20; // Maximum acceptable range sensor baseline error in cm.
const byte MIN_SUCCESSIVE_LIDAR_READS = 0;  // Number of consecutive lidar reads that will result in a 'detection'

///////////////////////////////////////////////////////////////////////////////
// PIR

enum PIR_TYPE{
    PIR_WIDE = 0,
    PIR_NARROW = 1
};

const byte NUM_PIR_SENSORS = 3;
const byte PIR_PINS[NUM_PIR_SENSORS] = {WIDE_PIR_PIN, NARROW_PIR_PIN, WIDE_PIR_2_PIN};
const byte PIR_TYPES[NUM_PIR_SENSORS] = {PIR_WIDE, PIR_NARROW, PIR_WIDE};
const bool PIR_ACTIVE_STATES[NUM_PIR_SENSORS] = {HIGH, HIGH, HIGH};
const bool PIR_MUST_GO_LOW_BETWEEN_TRIGGERS = false;
const unsigned long PIR_COOLDOWNS[NUM_PIR_SENSORS] = {5000, 2500, 5000};

const int CHECK_MOTION_INTERVAL = 100;

const long MOTION_INITIALISATION_TIME = 10000; // Time given for PIR sensor to calibrate in ms.
const byte MOTION_DETECTED = HIGH;
const byte MOTION_INITIALISATION_INTERVALS = 5;


///////////////////////////////////////////////////////////////////////////////
// Current Monitor
const int TRANSFORMER_RATIO = 2000;	// The ratio of the current transformer
const int BURDEN_RESISTOR = 2200;		// Value of the burden resistor in Ohms
const int CURRENT_SAMPLES = 1000;	// Number of ADC samples taken when measuring current
const double CURRENT_CALIBRATION_FACTOR = TRANSFORMER_RATIO / BURDEN_RESISTOR;


///////////////////////////////////////////////////////////////////////////////
// Commands & Tags
const char COMMAND_TERMINATOR = '$';
const char PACKET_START = '#';
const char PACKET_END = '$';


///////////////////////////////////////////////////////////////////////////////
// Lamp Dimming
const byte ACTIVE_BRIGHTNESS = 255;
const byte INACTIVE_BRIGHTNESS = ACTIVE_BRIGHTNESS * 0.2;
const long LAMP_ACTIVE_TIME = 300; // Active timeout in seconds
const int LAMP_TRANSITION_UP_PERIOD = 2; // Time between transition steps when going from low to high brightness in ms
const int LAMP_TRANSISTION_DOWN_PERIOD = 20; // Time between transition steps when going from high to low brightness in ms
const int LAMP_TRANSITION_STEP_UP = 1; //Transition step in 8-bit levels
const int LAMP_TRANSITION_STEP_DOWN = LAMP_TRANSITION_STEP_UP;


///////////////////////////////////////////////////////////////////////////////
// Data structures
struct SensorEntry{
    const char* id;
    int version;
    bool event_flag;
    long pir_count[NUM_PIR_SENSORS];
    bool pir_in_cooldown[NUM_PIR_SENSORS];
    long lidar_count;
    int lidar_range;
    int lidar_baseline;
    float air_temperature;
    float case_temperature;
    float road_temperature;
    float humidity;
    int illuminance;
    float current_draw;
    char timestamp[DATETIME_WIDTH];
};

struct LampControl{
    bool is_active;
    byte current_level;
    byte target;
    long timeout;
    int timer_id;
    bool control_enabled;
};


///////////////////////////////////////////////////////////////////////////////
// Function headers (for autocomplete)
void start_yun_serial();
void boot_status_change_ISR();

void start_xbee();
void prepare_xbee_packet();
void transmit_packet();
void send_xbee_packet();

void start_sensors();
void print_regular_entry();
void trigger_traffic_event();
void print_data();
void print_json_string();

void start_lidar();
int get_lidar_baseline(int variance);
int get_lidar_range();
void update_lidar();

void start_pir();
void update_pir();
void increment_pir_count(int sensor_num);

void start_air_temperature();
void update_air_temperature();
float get_air_temperature();

void start_road_temperature();
void update_road_temperature();
float get_road_temperature();
void start_case_temperature();
void update_case_temperature();
float get_case_temperature();
void start_thermal_flow();
void get_frame();

void start_humidity();
void update_humidity();
float get_humidity(float air_temperature);

void start_illuminance();
void update_illuminance();
int get_illuminance();

void start_current_monitor();
void update_current_draw();
float get_current_draw();

void start_rtc();
void update_timestamp();
long get_timestamp();
void get_datetime(char* buffer);
void update_system_clock();

void enable_lamp_control();
void activate_lamp();
void update_lamp_timeout();
void deactivate_lamp();
void set_lamp_target(int level);
void write_lamp_level(int level);
void transition_lamp();

void start_bluetooth_scanner();
void start_bluetooth_scan();
void start_bluetooth_read();
void read_bluetooth_buffer();
void stop_bluetooth_read();
