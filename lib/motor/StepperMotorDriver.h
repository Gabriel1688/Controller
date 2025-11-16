#pragma once
#include <stddef.h>
#include <stdint.h>
#include <vector>

// get from the CANSparkMaxLowLevel which manage the SparkMax driver.
struct StepperMotor_Obj {

    uint8_t m_inverted;

    int m_deviceId;

    int m_canTimeoutMs;
    int m_controlFramePeriod;
    int m_status0PeriodMs;
    int m_status1PeriodMs;
    int m_status2PeriodMs;
    int m_status3PeriodMs;
    int m_status4PeriodMs;

    float m_sensorRangeMin;
    float m_sensorRangeMax;

    int32_t m_canHandle;

    int m_activeSetpointApi;
};

extern "C" {
typedef struct StepperMotor_Obj *StepperMotor_handle;

typedef enum {
    StepperMotor_kForward = 0,
    StepperMotor_kReverse = 1,
} StepperMotor_LimitDirection;

typedef enum {
    StepperMotor_kCanID = 0,
    StepperMotor_kInputMode = 1,
    StepperMotor_kMotorType = 2,
    StepperMotor_kCommAdvance = 3,
    StepperMotor_kSensorType = 4,
    StepperMotor_kCtrlType = 5,
    StepperMotor_kIdleMode = 6,
    StepperMotor_kInputDeadband = 7,
    StepperMotor_kFeedbackSensorPID0 = 8,
    StepperMotor_kFeedbackSensorPID1 = 9,
    StepperMotor_kPolePairs = 10,
    StepperMotor_kCurrentChop = 11,
    StepperMotor_kCurrentChopCycles = 12,
    StepperMotor_kP_0 = 13,
    StepperMotor_kI_0 = 14,
    StepperMotor_kD_0 = 15,
    StepperMotor_kF_0 = 16,
    StepperMotor_kIZone_0 = 17,
    StepperMotor_kDFilter_0 = 18,
    StepperMotor_kOutputMin_0 = 19,
    StepperMotor_kOutputMax_0 = 20,
    StepperMotor_kP_1 = 21,
    StepperMotor_kI_1 = 22,
    StepperMotor_kD_1 = 23,
    StepperMotor_kF_1 = 24,
    StepperMotor_kIZone_1 = 25,
    StepperMotor_kDFilter_1 = 26,
    StepperMotor_kOutputMin_1 = 27,
    StepperMotor_kOutputMax_1 = 28,
    StepperMotor_kP_2 = 29,
    StepperMotor_kI_2 = 30,
    StepperMotor_kD_2 = 31,
    StepperMotor_kF_2 = 32,
    StepperMotor_kIZone_2 = 33,
    StepperMotor_kDFilter_2 = 34,
    StepperMotor_kOutputMin_2 = 35,
    StepperMotor_kOutputMax_2 = 36,
    StepperMotor_kP_3 = 37,
    StepperMotor_kI_3 = 38,
    StepperMotor_kD_3 = 39,
    StepperMotor_kF_3 = 40,
    StepperMotor_kIZone_3 = 41,
    StepperMotor_kDFilter_3 = 42,
    StepperMotor_kOutputMin_3 = 43,
    StepperMotor_kOutputMax_3 = 44,
    StepperMotor_kInverted = 45,
    StepperMotor_kOutputRatio = 46,
    StepperMotor_kSerialNumberLow = 47,
    StepperMotor_kSerialNumberMid = 48,
    StepperMotor_kSerialNumberHigh = 49,
    StepperMotor_kLimitSwitchFwdPolarity = 50,
    StepperMotor_kLimitSwitchRevPolarity = 51,
    StepperMotor_kHardLimitFwdEn = 52,
    StepperMotor_kHardLimitRevEn = 53,
    StepperMotor_kSoftLimitFwdEn = 54,
    StepperMotor_kSoftLimitRevEn = 55,
    StepperMotor_kRampRate = 56,
    StepperMotor_kFollowerID = 57,
    StepperMotor_kFollowerConfig = 58,
    StepperMotor_kSmartCurrentStallLimit = 59,
    StepperMotor_kSmartCurrentFreeLimit = 60,
    StepperMotor_kSmartCurrentConfig = 61,
    StepperMotor_kSmartCurrentReserved = 62,
    StepperMotor_kMotorKv = 63,
    StepperMotor_kMotorR = 64,
    StepperMotor_kMotorL = 65,
    StepperMotor_kMotorRsvd1 = 66,
    StepperMotor_kMotorRsvd2 = 67,
    StepperMotor_kMotorRsvd3 = 68,
    StepperMotor_kEncoderCountsPerRev = 69,
    StepperMotor_kEncoderAverageDepth = 70,
    StepperMotor_kEncoderSampleDelta = 71,
    StepperMotor_kEncoderInverted = 72,
    StepperMotor_kEncoderRsvd1 = 73,
    StepperMotor_kVoltageCompMode = 74,
    StepperMotor_kCompensatedNominalVoltage = 75,
    StepperMotor_kSmartMotionMaxVelocity_0 = 76,
    StepperMotor_kSmartMotionMaxAccel_0 = 77,
    StepperMotor_kSmartMotionMinVelOutput_0 = 78,
    StepperMotor_kSmartMotionAllowedClosedLoopError_0 = 79,
    StepperMotor_kSmartMotionAccelStrategy_0 = 80,
    StepperMotor_kSmartMotionMaxVelocity_1 = 81,
    StepperMotor_kSmartMotionMaxAccel_1 = 82,
    StepperMotor_kSmartMotionMinVelOutput_1 = 83,
    StepperMotor_kSmartMotionAllowedClosedLoopError_1 = 84,
    StepperMotor_kSmartMotionAccelStrategy_1 = 85,
    StepperMotor_kSmartMotionMaxVelocity_2 = 86,
    StepperMotor_kSmartMotionMaxAccel_2 = 87,
    StepperMotor_kSmartMotionMinVelOutput_2 = 88,
    StepperMotor_kSmartMotionAllowedClosedLoopError_2 = 89,
    StepperMotor_kSmartMotionAccelStrategy_2 = 90,
    StepperMotor_kSmartMotionMaxVelocity_3 = 91,
    StepperMotor_kSmartMotionMaxAccel_3 = 92,
    StepperMotor_kSmartMotionMinVelOutput_3 = 93,
    StepperMotor_kSmartMotionAllowedClosedLoopError_3 = 94,
    StepperMotor_kSmartMotionAccelStrategy_3 = 95,
    StepperMotor_kIMaxAccum_0 = 96,
    StepperMotor_kSlot3Placeholder1_0 = 97,
    StepperMotor_kSlot3Placeholder2_0 = 98,
    StepperMotor_kSlot3Placeholder3_0 = 99,
    StepperMotor_kIMaxAccum_1 = 100,
    StepperMotor_kSlot3Placeholder1_1 = 101,
    StepperMotor_kSlot3Placeholder2_1 = 102,
    StepperMotor_kSlot3Placeholder3_1 = 103,
    StepperMotor_kIMaxAccum_2 = 104,
    StepperMotor_kSlot3Placeholder1_2 = 105,
    StepperMotor_kSlot3Placeholder2_2 = 106,
    StepperMotor_kSlot3Placeholder3_2 = 107,
    StepperMotor_kIMaxAccum_3 = 108,
    StepperMotor_kSlot3Placeholder1_3 = 109,
    StepperMotor_kSlot3Placeholder2_3 = 110,
    StepperMotor_kSlot3Placeholder3_3 = 111,
    StepperMotor_kPositionConversionFactor = 112,
    StepperMotor_kVelocityConversionFactor = 113,
    StepperMotor_kClosedLoopRampRate = 114,
    StepperMotor_kSoftLimitFwd = 115,
    StepperMotor_kSoftLimitRev = 116,
    StepperMotor_kSoftLimitRsvd0 = 117,
    StepperMotor_kSoftLimitRsvd1 = 118,
    StepperMotor_kAnalogRevPerVolt = 119,
    StepperMotor_kAnalogRotationsPerVoltSec = 120,
    StepperMotor_kAnalogAverageDepth = 121,
    StepperMotor_kAnalogSensorMode = 122,
    StepperMotor_kAnalogInverted = 123,
    StepperMotor_kAnalogSampleDelta = 124,
    StepperMotor_kAnalogRsvd0 = 125,
    StepperMotor_kAnalogRsvd1 = 126,
    StepperMotor_kDataPortConfig = 127,
    StepperMotor_kAltEncoderCountsPerRev = 128,
    StepperMotor_kAltEncoderAverageDepth = 129,
    StepperMotor_kAltEncoderSampleDelta = 130,
    StepperMotor_kAltEncoderInverted = 131,
    StepperMotor_kAltEncodePositionFactor = 132,
    StepperMotor_kAltEncoderVelocityFactor = 133,

    StepperMotor_NumParameters
} StepperMotor_ConfigParameter;

typedef enum {
    StepperMotor_kInt32 = 0,
    StepperMotor_kUint32 = 1,
    StepperMotor_kFloat32 = 2,
    StepperMotor_kBool = 3
} StepperMotor_ParameterType;

typedef enum {
    StepperMotor_kDutyCycle = 0,
    StepperMotor_kVelocity = 1,
    StepperMotor_kVoltage = 2,
    StepperMotor_kPosition = 3,
    StepperMotor_kSmartMotion = 4,
    StepperMotor_kCurrent = 5,
    StepperMotor_kSmartVelocity = 6
} StepperMotor_ControlType;

typedef struct {
    float analogVoltage;
    float analogVelocity;
    float analogPosition;
    uint64_t timestamp;
} StepperMotor_PeriodicStatus3;

// CANSparkMaxLowLevel
StepperMotor_handle StepperMotor_Create(int deviceId);
void StepperMotor_Destroy(StepperMotor_handle handle);
bool StepperMotor_CheckId(int deviceId);
int StepperMotor_GetDeviceId(StepperMotor_handle handle, int *deviceId);
//int  StepperMotor_SetPeriodicFramePeriod(StepperMotor_handle handle, StepperMotor_PeriodicFrame frameId, int periodMs);

void StepperMotor_SetControlFramePeriod(StepperMotor_handle handle, int periodMs);
int StepperMotor_GetControlFramePeriod(StepperMotor_handle handle);

int StepperMotor_SetParameterFloat32(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, float value);
int StepperMotor_SetParameterInt32(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, int32_t value);
int StepperMotor_SetParameterUint32(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, uint32_t value);
int StepperMotor_SetParameterBool(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, uint8_t value);
int StepperMotor_GetParameterFloat32(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, float *value);
int StepperMotor_GetParameterInt32(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, int32_t *value);
int StepperMotor_GetParameterUint32(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, uint32_t *value);
int StepperMotor_GetParameterBool(StepperMotor_handle handle, StepperMotor_ConfigParameter paramId, uint8_t *value);
int StepperMotor_GetPeriodicStatus3(StepperMotor_handle handle, StepperMotor_PeriodicStatus3 *rawframe);

int StepperMotor_RestoreFactoryDefaults(StepperMotor_handle handle, uint8_t persist);
int StepperMotor_FactoryWipe(StepperMotor_handle handle, uint8_t persist);
float StepperMotor_SafeFloat(float f);

int StepperMotor_SetpointCommand(StepperMotor_handle handle, float value, StepperMotor_ControlType ctrl,
                                 int pidSlot, float arbFeedforward, int arbFFUnits);

// CANSparkMax
int StepperMotor_SetInverted(StepperMotor_handle handle, uint8_t inverted);
int StepperMotor_GetInverted(StepperMotor_handle handle, uint8_t *inverted);
int StepperMotor_SetSmartCurrentLimit(StepperMotor_handle handle, uint8_t stallLimit, uint8_t freeLimit, uint32_t limitRPM);
int StepperMotor_GetSmartCurrentLimit(StepperMotor_handle handle, uint8_t *stallLimit, uint8_t *freeLimit, uint32_t *limitRPM);
int StepperMotor_SetSecondaryCurrentLimit(StepperMotor_handle handle, float limit, int chopCycles);
int StepperMotor_GetSecondaryCurrentLimit(StepperMotor_handle handle, float *limit, int *chopCycles);
int StepperMotor_EnableVoltageCompensation(StepperMotor_handle handle, float nominalVoltage);
int StepperMotor_GetVoltageCompensationNominalVoltage(StepperMotor_handle handle, float *nominalVoltage);
int StepperMotor_DisableVoltageCompensation(StepperMotor_handle handle);
int StepperMotor_SetOpenLoopRampRate(StepperMotor_handle handle, float rate);
int StepperMotor_GetOpenLoopRampRate(StepperMotor_handle handle, float *rate);
int StepperMotor_SetClosedLoopRampRate(StepperMotor_handle handle, float rate);
int StepperMotor_GetClosedLoopRampRate(StepperMotor_handle handle, float *rate);
int StepperMotor_GetBusVoltage(StepperMotor_handle handle, float *busVoltage);
int StepperMotor_GetAppliedOutput(StepperMotor_handle handle, float *appliedOutput);
int StepperMotor_GetOutputCurrent(StepperMotor_handle handle, float *outputCurrent);
int StepperMotor_GetMotorTemperature(StepperMotor_handle handle, float *motorTemperature);
int StepperMotor_ClearFaults(StepperMotor_handle handle);
int StepperMotor_BurnFlash(StepperMotor_handle handle);
int StepperMotor_SetCANTimeout(StepperMotor_handle handle, int timeoutMs);

// CANPIDController
int c_SparkMax_SetP(StepperMotor_handle handle, int slotID, float gain);
int StepperMotor_SetI(StepperMotor_handle handle, int slotID, float gain);
int StepperMotor_SetD(StepperMotor_handle handle, int slotID, float gain);
int StepperMotor_SetDFilter(StepperMotor_handle handle, int slotID, float gain);
int StepperMotor_SetFF(StepperMotor_handle handle, int slotID, float gain);
int StepperMotor_SetIZone(StepperMotor_handle handle, int slotID, float IZone);
int StepperMotor_SetOutputRange(StepperMotor_handle handle, int slotID, float min, float max);
int StepperMotor_GetP(StepperMotor_handle handle, int slotID, float *gain);
int StepperMotor_GetI(StepperMotor_handle handle, int slotID, float *gain);
int StepperMotor_GetD(StepperMotor_handle handle, int slotID, float *gain);
int StepperMotor_GetDFilter(StepperMotor_handle handle, int slotID, float *gain);
int StepperMotor_GetFF(StepperMotor_handle handle, int slotID, float *gain);
int StepperMotor_GetIZone(StepperMotor_handle handle, int slotID, float *IZone);
int StepperMotor_GetOutputMin(StepperMotor_handle handle, int slotID, float *min);
int StepperMotor_GetOutputMax(StepperMotor_handle handle, int slotID, float *max);
}// extern "C"
