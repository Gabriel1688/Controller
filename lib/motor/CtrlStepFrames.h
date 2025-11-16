#pragma once
#include <stddef.h>
#include <stdint.h>

extern "C" {

/*********CAN ID Defines***********/
// API class;

// 0x00~0x0F No Memory CMDs
const int CMD_API_ENABLE_MOTOR = 0x01;                    // Enable Motor
const int CMD_API_DO_CALIBRATION = 0x02;                  // Do Calibration
const int CMD_API_SET_CURRENT_SET_POINT = 0x03;           // Set Current SetPoint
const int CMD_API_SET_VELOCITY_SET_POINT = 0x04;          // Set Velocity SetPoint
const int CMD_API_SET_POSITION_SET_POINT = 0x05;          // Set Position SetPoint
const int CMD_API_SET_POSITION_WITH_TIME = 0x06;          // Set Position with Time
const int CMD_API_SET_POSITION_WITH_VELOCITY_LIMIT = 0x07;// Set Position with Velocity-Limit

// 0x10~0x1F CMDs with Memory
const int CMD_API_SET_NODE_ID = 0x11;             // Set Node-ID and Store to EEPROM
const int CMD_API_SET_CURRENT_LIMIT = 0x12;       // Set Current-Limit and Store to EEPROM
const int CMD_API_SET_POSITION_LIMIT = 0x13;      // Set Velocity-Limit and Store to EEPROM
const int CMD_API_SET_ACCELERATION = 0x14;        // Set Acceleration （and Store to EEPROM）
const int CMD_API_APPLY_HOME_POSITION = 0x15;     // Apply Home-Position and Store to EEPROM
const int CMD_API_SET_AUTO_ENABLE = 0x16;         // Set Auto-Enable and Store to EEPROM
const int CMD_API_SET_DEC_KP = 0x17;              // Set DCE Kp
const int CMD_API_SET_DEC_KV = 0x18;              // Set DCE Kv
const int CMD_API_SET_DEC_KI = 0x19;              // Set DCE Ki
const int CMD_API_SET_DEC_KD = 0x1A;              // Set DCE Kd
const int CMD_API_SET_ENABLE_STALL_PROTECT = 0x1B;// Set Enable Stall-Protect

// 0x20~0x2F Inquiry CMDs
const int CMD_API_GET_CURRENT = 0x21;    // Get Current
const int CMD_API_GET_VELOCITY = 0x22;   // Get Velocity
const int CMD_API_GET_POSITION = 0x23;   // Get Position
const int CMD_API_GET_OFFSET = 0x24;     // Get Offset
const int CMD_API_GET_TEMPERATURE = 0x25;// Get temperature

//0x7d~0xFF MISC CMDs
const int CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH = 0x7d;// enable motor temperature watch
const int CMD_API_ERASE_CONFIGS = 0x7e;                // Erase Configs
const int CMD_API_REBOOT = 0x7f;                       // Reboot

// Note: This is dependent on __attribute__((PACKED)) and processor is
// little-endian
//NOTE::Following data structure need to update according to
//https://github.com/Gabriel1688/dummy/blob/403d62d1c7ea570c233f986e2b6118dd3a53d0c8/firmware/dummy-35motor-fw/UserApp/protocols/interface_can.cpp
#define PACKED __attribute__((__packed__))
typedef struct PACKED {
    float setpoint;
    int16_t auxSetpoint;
    uint32_t pidSlot : 2;
    uint8_t arbFFUnits : 1;
    uint32_t rsvd0 : 5;
    uint32_t rsvd1 : 8;
} dataframe_enable_motor_out_t;

typedef struct PACKED {
    float setpoint;
    int16_t auxSetpoint;
    uint32_t pidSlot : 2;
    uint8_t arbFFUnits : 1;
    uint32_t rsvd0 : 5;
    uint32_t rsvd1 : 8;
} dataframe_do_calibration_out_t;

typedef struct PACKED {
    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    uint16_t firmwareBuild;
    uint8_t debugBuild;
    uint8_t hardwareRevision;
} dataframe_get_current_in_t;

typedef union {
    dataframe_enable_motor_out_t enableMotorOut;
    dataframe_do_calibration_out_t doCalibrationOut;
    dataframe_get_current_in_t getCurrentIn;
    uint8_t data[8];
} dataframe_t;

typedef enum {
    deviceBroadcast = 0,
    armController,
    gripperController,
    motionController,
    gyroSensor,
    firmwareUpdate = 31
} deviceType_t;

typedef enum {
    manufacturerBroadcast = 0,
    DAMIAO = 1,
    DUMMY = 2,
    TeamUse = 8
} manufacturer_t;
//NOTE:: frameIDFields need to update according to
//https://github.com/Gabriel1688/dummy/blob/403d62d1c7ea570c233f986e2b6118dd3a53d0c8/firmware/dummy-35motor-fw/UserApp/protocols/interface_can.cpp
typedef struct PACKED {
    uint16_t deviceNumber : 6;
    uint16_t api : 10;
    manufacturer_t manufacturer : 8;
    deviceType_t deviceType : 5;
    uint8_t rsvd : 3;//these are DNC
} frameIDFields_t;

typedef union {
    frameIDFields_t fields;
    uint32_t raw;
} frameID_t;

typedef struct {
    frameID_t id;
    dataframe_t dataframe;
    size_t length;
} motor_frame_t;

#undef PACKED
}//extern "C"
