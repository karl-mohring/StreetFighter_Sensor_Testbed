#include "Arduino.h"
#include <Logging.h>

// Unit Settings
const int SSL_TESTBED_VERSION = 8;
const byte UNIT_ID = 1;
const char UNIT_NAME[] = "Flauros";
#define USE_SERIAL Serial1

// Comms settings
const long SERIAL_BAUD = 57600;
const int LOGGER_LEVEL = LOG_LEVEL_INFOS;
const long XBEE_BAUD = 9600;

// Pin assignments
const byte YUN_HANDSHAKE_PIN = 7;
const byte XBEE_TX = 5;
const byte XBEE_RX = 4;
const byte XBEE_SLEEP_PIN = 8;
const byte LAMP_CONTROL_PIN = 6;
const byte PIR_MOTION_PIN = 9;
const byte LIDAR_PWM_PIN = 10;
const byte LIDAR_TRIGGER_PIN = 13;

const byte LAMP_STATUS_PIN = A0;
const byte RANGEFINDER_AN_PIN = A1;
const byte MICROPHONE_PIN = A2;
const byte HUMIDITY_PIN = A3;
const byte TEMPERATURE_PIN = A4;
const byte CURRENT_DETECT_PIN = A5;

// Timer config
const int MOTION_COOLDOWN = 2000; // Time left for the PIR sensor to cool down after a detection in ms
const int CHECK_MOTION_INTERVAL = MOTION_COOLDOWN/8;

const int CHECK_FLOW_INTERVAL = 200;
const int FLOW_COOLDOWN = 2000; // Optical flow detection cooldown in ms

const int CHECK_RANGE_INTERVAL = 100;
const int RANGE_DETECT_THRESHOLD = 70; // Minimum threshold for range detection in cm. (Target must move at least this much)

const int LIDAR_CHECK_RANGE_INTERVAL = 100;
const int LIDAR_DETECT_THRESHOLD = 50;

const long PRINT_INTERVAL = 60000;
const int CHECK_ENVIRONMENTAL_SENSOR_INTERVAL = PRINT_INTERVAL/3;
const long XBEE_TRANSMIT_INTERVAL = PRINT_INTERVAL; // Time between XBee transmissions


// Misc
const int YUN_BOOT_DELAY = 5000; //Time to wait for Yun to boot up before checking the handshake
const long YUN_LINUX_BAUD_RATE = 250000;

const byte COMMAND_CACHE_SIZE = 80;
const byte SEND_BUFFER_SIZE = 50;
const byte MIN_BASELINE_READS = 20; // Minimum number of reads needed to establish a baseline
const byte MAX_BASELINE_READS = 30;
const int BASELINE_READ_INTERVAL = 200; // Time between range baseline calibration reads
const int BASELINE_VARIANCE_THRESHOLD = 20; // Maximum acceptable range sensor baseline error in cm.
const byte MIN_SUCCESSIVE_SONAR_READS = 4;
const byte MIN_SUCCESSIVE_LIDAR_READS = 0;
const int MOTION_INITIALISATION_TIME = 10000; // Time given for PIR sensor to calibrate in ms.
const byte MOTION_DETECTED = HIGH;
const byte MOTION_INITIALISATION_INTERVALS = 5;
const float AREF_VOLTAGE = 5.0;
const byte SOUND_SAMPLE_TIME = 100; // Width of sample window in ms
const byte TEMPERATURE_RESOLUTION = 12; //12-bit temperature resolution

// Current
const int TRANSFORMER_RATIO = 2000;	// The ratio of the current transformer
const int BURDEN_RESISTOR = 330;		// Value of the burden resistor in Ohms
const int CURRENT_SAMPLES = 1000;	// Number of ADC samples taken when measuring current
const float CURRENT_CALIBRATION_FACTOR = TRANSFORMER_RATIO / BURDEN_RESISTOR;

// Commands & Tags
const char COMMAND_TERMINATOR = '$';
const char RESET_UVD_COUNT = 'q';
const char DISABLE_UVD = 'w';
const char ENABLE_UVD = 'e';
const char PACKET_START = '#';
const char PACKET_END = '$';
const char AMBIENT_TEMPERATURE_TAG = 'A';
const char ROAD_TEMPERATURE_TAG = 'R';
const char CASE_TEMPERATURE_TAG = 'E';
const char HUMIDITY_TAG = 'H';
const char ILLUMINANCE_TAG = 'I';
const char LAMP_STATUS_TAG = 'S';
const char NOISE_TAG = 'N';
const char CURRENT_DRAW_TAG = 'C';
const char TIMESTAMP_TAG = 'T';
const char UVD_RANGE_TAG = 'U';
const char UVD_COUNT_TAG = 'u';
const char MOTION_STATUS_TAG = 'M';
const char MOTION_COUNT_TAG = 'm';
const char LIDAR_RANGE_TAG = 'L';
const char LIDAR_COUNT_TAG = 'l';
const char TRAFFIC_EVENT_TAG = 't';

// Lamp Dimming
const byte ACTIVE_BRIGHTNESS = 255;
const byte INACTIVE_BRIGHTNESS = ACTIVE_BRIGHTNESS * 0.2;
const long LAMP_ACTIVE_TIME = 300; // Active timeout in seconds
const int LAMP_TRANSITION_UP_PERIOD = 2; // Time between transition steps when going from low to high brightness in ms
const int LAMP_TRANSISTION_DOWN_PERIOD = 20; // Time between transition steps when going from high to low brightness in ms
const int LAMP_TRANSITION_STEP_UP = 1; //Transition step in 8-bit levels
const int LAMP_TRANSITION_STEP_DOWN = LAMP_TRANSITION_STEP_UP;

// Data structures
struct SensorEntry{
    const char* id;
    int version;
    bool event_flag;
    long pir_count;
    bool last_pir_status;
    long lidar_count;
    int lidar_range;
    int lidar_baseline;
    long sonar_count;
    int sonar_range;
    int sonar_baseline;
    float air_temperature;
    float case_temperature;
    float road_temperature;
    float humidity;
    int illuminance;
    float current_draw;
    int noise_level;
    long timestamp;
};

struct LampControl{
    bool is_active;
    byte current_level;
    byte target;
    long timeout;
    int timer_id;
    bool control_enabled;
};

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
String pack_json_string();
void start_sonar();
int get_sonar_baseline(int variance);
int get_sonar_range();
void update_sonar();
void start_lidar();
int get_lidar_baseline(int variance);
int get_lidar_range();
void update_lidar();
void start_pir();
void update_pir();
void enter_pir_cooldown();
void resume_pir_detection();
void start_air_temperature();
void update_air_temperature();
float get_air_temperature();
void start_road_temperature();
void update_road_temperatur();
float get_road_temperature();
void start_case_temperature();
void update_case_temperature();
float getCaseTemperature();
void start_humidity();
void update_humidity();
float get_humidity(float air_temperature);
void start_illuminance();
void update_illuminance();
int get_illuminance();
void start_noise_monitoring();
void update_noise_level();
int get_noise_level(int sample_period);
void start_current_monitor();
void update_current_draw();
float get_current_draw();
void start_rtc();
void update_timestamp();
long get_timestamp();
void enable_lamp_control();
void activate_lamp();
void update_lamp_timeout();
void deactivate_lamp();
void set_lamp_target(int level);
void write_lamp_level(int level);
void transition_lamp();
