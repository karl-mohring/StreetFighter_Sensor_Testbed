/**************************************************************************/
/*!
@file     LIDARLite_registers.h
@author   Stuart Feichtinger
@license  MIT (see license.txt)

Arduino Library for the LIDAR-Lite from PulsedLight, Inc. Register
descriptions and notes from LIDAR-Lite Operating Manual by PulsedLight
Rev. 12/2/2014.

@section  HISTORY
v0.0.1 - First release

*/
/**************************************************************************/


#pragma once


//TODO: Finish Adding bit definitions and descriptions

//TODO: Clean up file and make more readable

//DEFAULT I2C ADDRESS
#define LIDARLite_ADDRESS   0x62


/*####################INTERNAL REGISTERS######################*/


/**************************************************************/
/*                COMMAND CONTROL REGISTER                    */
/*                    control_reg[0x0]                        */
/*                     (READ & WRITE)                         */
/**************************************************************/
#define LIDAR_COMMAND       0x0   // Register Address

  /*Bit definitions*/

  #define RESET_FPGA          0
  //  Reset FPGA. Re-loads FPGA from internal Flash memory
  //  (all registers return to default values)


  #define PROCESS_2ND         1
  //  Correlation processing without new acquisition – used to process delay of
  //    second peak after bit 0 in control register 0x4b is set to "1"


  #define REPROCESS           2
  //  Process correlation without new acquisition (used to reprocess)


  #define NO_DC               3
  //  Take acquisition & correlation processing WITHOUT DC correction


  #define USE_DC              4
  //  Take acquisition & correlation processing WITH DC correction


/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                  SYSTEM STATUS REGISTER                    */
/*                    control_reg[0x1]                        */
/*                     (READ & WRITE)                         */
/**************************************************************/

#define LIDAR_SYS_STATUS    0x1     //  Register Address

  /*Bit definitions*/
  #define HEALTH                0
  //  “1” state indicates that all health monitoring criteria were met on the
  //  last acquisition. ”0” == possible problem. SEE NOTE BELOW


  #define REF_OVRFLOW           1
  //  Overflow detected in correlation process associated with a reference
  //  acquisition


  #define SIG_OVERFLOW          2
  //  Overflow detected in correlation process associated with a signal
  //  acquisition


  #define SIGNAL_NOT_VALID      3
  //  Indicates that the signal correlation peak is equal to or below
  //  correlation record noise threshold


  #define SECOND_RETURN         4
  //  Secondary return detected above correlation noise floor threshold


  #define VELOCITY_COMPLETE     5
  //  Velocity measurement completed


  #define EXT_TRIG_COMPLETE     6
  //  External measurement performed


  #define EYE_SAFE              7
  //  This bit will go high if eye-safety protection has been activated

/*
NOTE: Health status (BIT 1) indicates that the preamp is operating properly,
transmit power is active and a reference pulse has been processed and stored.
*/


/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*             MAXIMUM AQUISITION COUNT REGISTER              */
/*                    control_reg[0x2]                        */
/*                     (READ & WRITE)                         */
/*                      default 0x80                          */
/**************************************************************/

#define LIDAR_MAX_COUNT     0x2   // Register Address

/*  Bits 0-7:
  Maximum Acquisition Count - 0-255. Control the FPGA maximum signal integration
  time. Stronger signal results in reduced acquisition count to prevent
  internal register overflow.
  Sig overflow flag and Ref overflow flag in control register 1 are set when
  automatic limiting occurs.
*/


/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*           CORRELATION RECORD LENGTH REGISTER               */
/*                    control_reg[0x3]                        */
/*                     (READ & WRITE)                         */
/*                      default 0x21                          */
/**************************************************************/

// control_reg[0x51] is WRITE only version of register

#define LIDAR_CORR_LENGTH   0x3   // Register Address

  /*Bit definitions*/
  #define CORR_START_ADDR   0
  /*  BITS 0-3:
      Value in the range from 0x00-0x0f – starting point in correlation record
      (record broken into 64 element segments 1024 total
      Default 0x1 corresponding to 64
  */

  #define CORR_STOP_ADDR    4
  /*  BITS 4-7:
      Value in the range from 0x00-0x0f – stopping point in correlation record
      Default 0x2 (bit 5) corresponding to 512
  */

/*
NOTE: With longer correlation records, burst pulse period is
roughly proportional to the length of the  correlation record.
Unnecessarily long record length increases the probability of
false detections.
*/

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*             AQUISITION MODE CONTROL REGISTER               */
/*                    control_reg[0x4]                        */
/*                     (READ & WRITE)                         */
/*                      default 0x00                          */
/**************************************************************/


#define LIDAR_MODE          0x4   // Register Address

  /*Bit definitions*/

  #define PREAMP_OFF          0
  // Shutdown preamp between measurements

/*------------NOT USED IN STANDARD LIDAR LIGHT----------- */
/*                                                        */
  #define CLK_SHUT            1
/*  // External Clock Shutdown                            */
/*                                                        */
/*------------NOT USED IN STANDARD LIDAR LIGHT----------- */


  #define FPGA_SLEEP          2
  // Full FPGA sleep after measurement

  #define DET_OFF             3
  // Turns off detector bias after measurement

  // BIT 4 not currently used


  #define V_SCALE             5
  //  “1” sets the velocity measurement separation to 10msec resulting in a
  //  velocity calibration in meters/sec. A “0” value results  in a measurement
  //  separation of 100msec.


  #define REF_INHIBIT         6
  //  “1” inhibits the acquisition of reference pulses reducing measurement
  //  times and reducing measurement variations at the  expense of decreasing
  //  accuracy over time. “0” allows normal operation


  #define V_ENABLE            7
  // Enable velocity measurement

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*       MEASURED AQUISITION THRESHOLD OFFSET REGISTER        */
/*                    control_reg[0x5]                        */
/*                     (READ & WRITE)                         */
/*                      nominal 128                           */
/**************************************************************/

#define LIDAR_THOLD_OFFSET  0x5       // Register Address

/*  BITS 0-7:
    Measured DC value out of correlation sampler input. Value based on the
    ratio of 1’s and 0’s (read only) preamp
    (Parameter used as part of health flag criteria)
*/

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*       MEASURED REFERENCE CORRELATION DELAY REGISTERS       */
/*                    control_reg[0x6-7]                      */
/*                       (READ ONLY)                          */
/**************************************************************/

#define LIDAR_REF_DELAY_H   0x6       // Register Address
// BITS 0-7: Measured delay of reference in correlation record (HIGH byte)

#define LIDAR_REF_DELAY_L   0x7       // Register Address
// BITS 0-7: Measured delay of reference in correlation record (LOW byte)

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*          REFERENCE PEAK CORRELATION VALUE REGISTER         */
/*                    control_reg[0x8]                        */
/*                       (READ ONLY)                          */
/**************************************************************/

#define LIDAR_REF_PEAK      0x8       // Register Address
  /*  BITS 0-7:
      Correlation Peak value reference scaled to 0 – 0xff max peak value
      (Parameter used as part of health flag criteria)
  */


/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*            VELOCITY MEASURMENT OUTPUT REGISTER             */
/*                    control_reg[0x9]                        */
/*                       (READ ONLY)                          */
/**************************************************************/

#define LIDAR_VELOCITY_READ 0x9       // Register Address
  /*  BITS 0-7:
      Velocity in .1 meters/sec - (8 bit signed value)
      See Mode control, Register 4 for information on changing the
      scale factor to 1m/sec
  */

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*         MEASURED SIGNAL CORRELATION DELAY REGISTERS        */
/*                    control_reg[0xa-b]                      */
/*                       (READ ONLY)                          */
/**************************************************************/

#define LIDAR_CORR_DELAY_H  0xa       // Register Address
//  BITS 0-7:
//  Measured delay of signal return in correlation record (HIGH byte)

#define LIDAR_CORR_DELAY_L  0xb       // Register Address
//  BITS 0-7:
//  Measured delay of signal return in correlation record (LOW byte)

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*            SIGNAL PEAK CORRELATION VALUE REGISTER          */
/*                     control_reg[0xc]                       */
/*                       (READ ONLY)                          */
/**************************************************************/

#define LIDAR_CORR_PEAK     0xc       // Register Address
  /*  BITS 0-7:
      Correlation Peak value of signal correlation
      (scaled to 0 – 0xff max peak value)
  */

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*           CORRELATION RECORD NOISE FLOOR REGISTER          */
/*                     control_reg[0xd]                       */
/*                        (READ ONLY)                         */
/**************************************************************/

#define LIDAR_CORR_NOISE    0xd       // Register Address
//  BITS 0-7:
//  Correlation record noise floor * 1.25 (for setting valid signal threshold)

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*              RECEIVED SIGNAL STRENGHTH REGISTER            */
/*                     control_reg[0xe]                       */
/*                        (READ ONLY)                         */
/**************************************************************/

#define LIDAR_SIG_STRENGTH  0xe       // Register Address
//  BITS 0-7:
//  Calculated signal strength (typical range 10 min - 128 maximum)

/*
NOTE: Calculation based on the number of averaged samples and
      correlation peak value.
*/

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                 CALCULATED DISTANCE REGISTERS              */
/*                   control_reg[0xf-10 & 0x8f]               */
/*                        (READ ONLY)                         */
/**************************************************************/


#define LIDAR_DIST_READ_H   0xf     //  HIGH Byte Register Address
//  BITS 0-7:
//  Calculated distance in cm (difference between signal and reference delay)
//  NOTE: if the MSB is 1 then the reading is NOT considered valid.

#define LIDAR_DIST_READ_L   0x10    //  LOW Byte Register Address
//  BITS 0-7:
//  Calculated distance in cm (difference between signal and reference delay)

#define LIDAR_DIST_READ_HL  0x8f    // 16-bit Distance in cm.
// Two byte read == LIDAR_DIST_READ_H << 8 | LIDAR_DIST_READ_L

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*             DC THRESHOLD COMMAND REGISTER                  */
/*                   control_reg[0x11]                        */
/*                     (READ & WRITE)                         */
/**************************************************************/

#define LIDAR_DC_THRESHOLD  0x11      // Register Address
//  BITS 0-7:
//  The required DC compensation command value to maintain zero 
//  crossing offset at preamp.
//  (Parameter used as part of health flag criteria)

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                 SIGNAL BURST DELAY REGISTER                */
/*                     control_reg[0x12]                      */
/*                       (WRITE ONLY)                         */
/**************************************************************/

#define LIDAR_BURST_DELAY   0x12      // Register Address
// BITS 0-7:
// Added delay between signal bursts.
// Used to lower transmission duty cycle and pulse frequency

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                DISTANCE CALIBRATION REGISTER               */
/*                     control_reg[0x13]                      */
/*                      (READ & WRITE)                        */
/**************************************************************/

#define LIDAR_DISTANCE_CAL  0x13      // Register Address
//  BITS 0-7:
//  Distance calibration. Signed 8 bit value adds or subtracts from distance

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                  PREVIOUS DISTANCE REGISTERS               */
/*                    control_reg[0x14-15]                    */
/*                        (READ ONLY)                         */
/**************************************************************/

#define LIDAR_PREV_DIST_H   0x14      // Register Address
  //  BITS 0-7:
  //  Previous measured distance (HIGH Byte)

#define LIDAR_PREV_DIST_L   0x15      // Register Address
  //  BITS 0-7:
  //  Previous measured distance (LOW Byte)

/*------------------------END REGISTER------------------------*/



/*####################EXTERNAL REGISTERS######################*/


/**************************************************************/
/*               COMMAND AND CONTROL REGISTER                 */
/*                     control_reg[0x40]                      */
/*                       (WRITE ONLY)                         */
/**************************************************************/

#define LIDAR_CMD_CTRL      0x40    // Register Address

/*
BITS 0-2: Starting action address
______________________________________________________
VALUE    |       DESCRITPTION                        |
------------------------------------------------------
 ￼000     | No Operation                              |
------------------------------------------------------
￼ 001     | Start processes at template store         |
------------------------------------------------------
 010     | Start processes at signal acquisition     |
------------------------------------------------------
 011     | Start processes at clear correlation      |
         | memory                                    |
------------------------------------------------------
 100     | Start processes at Perform Correlation    |
         | process                                   |
------------------------------------------------------
 101     | Start processes at Delay calculation      |
         | processing                                |
------------------------------------------------------
 110     | Perform only memory bank access enable    |
------------------------------------------------------
 111     | Perform only correlation record filtering |
------------------------------------------------------
*/

//BIT 3
  #define STORE_TEMPLATE      3
  // Store template pattern enable

//BIT 4
  #define SIGNAL_AQ_ENABLE    4
  // Signal acquisition enable

//BIT 5
  #define CLR_CORR_MEM        5
  // Clear Correlation Memory

//BIT 6
  #define RUN_CORR            6
  // Perform Correlation process

//BIT 7
  #define RUN_DELAY_CALC      7
  // Perform Delay calculation Processing

/*
NOTE: Command control register - Writing to this register
through the I2C interface immediately initiates a command
operation. Thus it is important to initiate this command
only at the completion of defining other registers.
*/

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                 HARDWARE VERSION REGISTER                  */
/*                     control_reg[0x41]                      */
/*                       (READ ONLY)                          */
/**************************************************************/

#define LIDAR_HW_VER        0x41    // Register Address
/*
  BITS 0-7:
  Laser units revisions begin with 0x01 (short range),
  0x20 for long range lasers, and led units begin with 0x40.
*/

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                 PREAMP DC CONTROL REGISTER                 */
/*                     control_reg[0x42]                      */
/*                        (WRITE ONLY)                        */
/**************************************************************/

#define LIDAR_PREAMP_CTRL   0x42      // Register Address
//  BITS 0-7:
//  Preamp DC control 0-255 – used in DC compensation servo loop


/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*               TRANSMIT POWER CONTROL REGISTER              */
/*                     control_reg[0x43]                      */
/*                        (WRITE ONLY)                        */
/*                        default 0x69                        */
/**************************************************************/

#define LIDAR_TX_PWR_CTRL   0x43      // Register Address
  //  BITS 0-3:
  //  Signal laser or led power level control (0x00-0x0f)
  //  BITS 4-7:
  //  Reference  led power level control (0x00-0x0f)

/* NOTE: Maximum value set at factory 1amp LED and 3amps Laser
         product. The maximum output power of the optical source
         is set through a factory selected resistor which limits
         available current to the transmit source. When using laser
         devices, which have a much higher operating current, do not
         increase the reference control level beyond the factory
         settings. Increasing the reference control level may cause
         premature failure of the internal LED reference diode.
*/

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*               PROCESSING RANGE GATE REGISTERS              */
/*                   control_reg[0x44-0x45]                   */
/*                        (WRITE ONLY)                        */
/**************************************************************/


 #define LIDAR_GATE_L        0x44      // Register Address
//  BITS 0-7:
//  Processing range gate (LOW byte)

 #define LIDAR_GATE_H        0x45      // Register Address
//  BITS 0-7:
//  Processing range gate (HIGH byte)

// Sets the staring location for range processing in the correlation

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                  PWM RANGE OUTPUT REGISTER                 */
/*                     control_reg[0x46]                      */
/*                        (WRITE ONLY)                        */
/**************************************************************/

#define LIDAR_PWM_MODE      0x46      // Register Address

/*  BIT 1:
    Single bit passed from microcontroller PWM output. High at mode pin
    returns pulse width proportional distance.
*/
/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                AQUISITION STATUS REGISTER                  */
/*                     control_reg[0x47]                      */
/*                        (READ ONLY)                         */
/**************************************************************/

#define LIDAR_READ_STATUS   0x47    // Register Address

/*Bit Definitions*/

  #define BUSY              0
  /*  Busy, indicates that the processor is actively performing an
      acquisition process.  */

  #define REF_OVRFLOW_EXT   1
  /*  Overflow detected in correlation process associated with the
      reference  acquisition */

  #define SIG_OVRFLOW_EXT   2
  /*  Overflow detected in correlation process associated with the
      signal  acquisition */

  #define PIN_STATUS        3
  /*  Indicates the state of the Mode Select external pin.
      De-bounced, masked from output signals, inverted  */

  #define SECOND_PEAK       4
  /*  Indicates a second peak was detected. 2nd peak value compared to
      noise floor    */

  #define TIMESTAMP         5
  /*  Active between velocity measurement pairs   */

  #define SIGNAL_INVALID    6
  /* "1" No signal detected. "0" Signal detected. */

  #define EYE_SAFE_EXT      7
  /*  ￼Indicates that eye safety average power limit has been exceeded
      and power reduction is in place.    */

  /*------------------------END REGISTER------------------------*/


  // REGISTER 0x48 NOT USED


/**************************************************************/
/*                AQUISITION STATUS REGISTER                  */
/*                     control_reg[0x49]                      */
/*                        (READ ONLY)                         */
/**************************************************************/


#define LIDAR_DC_OFFSET     0x49    // Register Address

  /*  BIT 0-7:
      Measured DC value out of correlation sampler input. Value
      based on the ratio of 1’s and 0’s.
  */

/*------------------------END REGISTER------------------------*/



/**************************************************************/
/*                    OUTPUT PORT REGISTER                    */
/*                     control_reg[0x4a]                      */
/*                       (WRITE ONLY)                         */
/**************************************************************/

#define LIDAR_OUTPUT_PORT   0x4a    // Output port

/*  BIT DEFINITIONS:  */

  #define XMT_1_EN          0
  //  Enables transmit channel 1 high-side current source

  #define XMT_2_EN          1
  //  Enables transmit channel 2 high-side current source

  #define PREAMP_GAIN       2
  //  Selects high 2x or low gain 1x setting - NOT USED BY LIDAR LITE

  #define DETECTOR_SEL      3
  //  Select reveiver detector 1 or 2 - NOT USED BY LIDAR LITE

  #define BUSY_FLAG         4
  /*  Busy flag outputs on mode select pin with I2C initiated acquisition.
      High causes pin output to be driven low  */


/*------------------------END REGISTER------------------------*/


#define LIDAR_TWO_ECHO      0x4b    // Range Processing Criteria for two echoes. Max signal, Max/Min Range.
#define LIDAR_NXT_LG_PEAK   0x4c    // 2nd largest detected peak in signal correlation record.
#define LIDAR_SW_VER        0x4f    // Software Version.
#define LIDAR_CORR_SIZE     0x51    // Correlation record size select – (reference and signal return)
#define LIDAR_CORR_DATA_L   0x52    // Correlation Data access port (low byte)
#define LIDAR_READ_SETTINGS 0x53    // Acquisition Settings – selects ext. memory access, signal record select
#define LIDAR_READ_DELAY_H  0x57    // Measured delay of reference or signal in correlation window
#define LIDAR_READ_DELAY_L  0x58    // Measured delay of reference or signal in correlation window
#define LIDAR_CORR_PEAK_EXT 0x59    // Correlation peak value of reference or signal
#define LIDAR_MAX_NOISE     0x5a    // Correlation record noise floor * 1.25 (for setting valid signal threshold)
#define LIDAR_RSSI          0x5b    //Received signal strength (typical range 10min - 255 maximum)
#define LIDAR_CORR_SIG_PAT  0x5c    //Reset correlator / increment transmit signal pattern
#define LIDAR_CORR_SIGN_BIT 0x5d    //Correlation Data access port (sign bit)
#define LIDAR_READ_TX_PWR   0x5f    //Measured transmit power – Supports Laser safety monitoring
#define LIDAR_FINE_DELAY    0x60    //Measured fine delay (used as part of measured delay calculation)
#define LIDAR_COARSE_DELAY_H 0x61    //Coarse delay (used as part of measured delay calculation)(HIGH Byte)
#define LIDAR_COARSE_DELAY_L 0x62    //Coarse delay (used as part of measured delay calculation)(LOW Byte)
#define LIDAR_POS_CROSS     0x63    //Positive correlation sample before zero crossing (correlation pulse falling edge)
#define LIDAR_NEG_CROSS     0x64    //Negative correlation sample after zero crossing (correlation pulse falling edge)
#define LIDAR_PWR_CTRL      0x65    //Power control settings
#define LIDAR_VELOCITY_CTRL 0x68    //Velocity measurement window setting register
