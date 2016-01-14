#include "Arduino.h"
#include <Logging.h>

// Unit Settings
const byte UNIT_ID = 1;

// Comms settings
const long SERIAL_BAUD = 115200;
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
const int CHECK_MOTION_INTERVAL = 500;
const int MOTION_COOLDOWN = 2000; // Time left for the PIR sensor to cool down after a detection in ms

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
const int YUN_BOOT_DELAY = 20000; //Time to wait for Yun to boot up before checking the handshake
const byte COMMAND_CACHE_SIZE = 80;
const byte SEND_BUFFER_SIZE = 50;
const byte MIN_BASELINE_READS = 20; // Minimum number of reads needed to establish a baseline
const byte MAX_BASELINE_READS = 30;
const int BASELINE_READ_INTERVAL = 200; // Time between range baseline calibration reads
const int BASELINE_VARIANCE_THRESHOLD = 20; // Maximum acceptable range sensor baseline error in cm.

const int MOTION_INITIALISATION_TIME = 10000; // Time given for PIR sensor to calibrate in ms.
const byte MOTION_DETECTED = HIGH;
const byte MOTION_INITIALISATION_INTERVALS = 6;

const byte OPTICAL_FLOW_MAX_RETRIES = 8;
const float AREF_VOLTAGE = 5.0;

const byte LIGHT_HEALTHY_THRESHOLD = 500;
const byte SOUND_SAMPLE_TIME = 100;

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


// JSON string tags
const char ID[] = "id";
const char VERSION[] = "version";
const char COUNT_UVD[] = "count_uvd";
const char UVD_RANGE[] = "uvd_range";
const char COUNT_PIR[] = "count_pir";
const char PIR_STATUS[] = "pir_status";
const char COUNT_LIDAR[] = "count_lidar";
const char LIDAR_RANGE[] = "lidar_range";
const char TIME_STAMP[] = "timestamp";
const char CURRENT_DRAW[] = "current_draw";
const char NOISE[] = "noise";
const char LAMP_STATUS[] = "lamp status";
const char AIR_TEMP[] = "air_temp";
const char ROAD_TEMP[] = "road_temp";
const char CASE_TEMP[] = "case_temp";
const char HUMIDITY[] = "humidity";
const char ILLUMINANCE[] = "illuminance";
const char EVENT_FLAG[] = "event_flag";