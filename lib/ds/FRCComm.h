#pragma  once

#include <stdint.h>
#include <pthread.h>

//https://github.com/wpilibsuite/ni-libraries/blob/5ef44a3de346da689ade4e7f0c5263e238f8a81a/src/include/FRC_NetworkCommunication/FRCComm.h

#define ERR_FRCSystem_NetCommNotResponding -44049
#define ERR_FRCSystem_NoDSConnection -44018


// clang-format off
#define kTcpRecvMask_Joysticks      0x000000FF
#define kTcpRecvMask_MatchInfoOld   0x00000100
#define kTcpRecvMask_MatchInfo      0x00000200
#define kTcpRecvMask_GameSpecific   0x00000400
// clang-format on

enum AllianceStationID_t
{
    kAllianceStationID_red1,
    kAllianceStationID_red2,
    kAllianceStationID_red3,
    kAllianceStationID_blue1,
    kAllianceStationID_blue2,
    kAllianceStationID_blue3,
};

enum MatchType_t
{
    kMatchType_none,
    kMatchType_practice,
    kMatchType_qualification,
    kMatchType_elimination,
};

//struct ControlWord_t
//{
//    uint32_t enabled : 1;
//    uint32_t autonomous : 1;
//    uint32_t test : 1;
//    uint32_t eStop : 1;
//    uint32_t fmsAttached : 1;
//    uint32_t dsAttached : 1;
//    uint32_t control_reserved : 26;
//};

struct JoystickAxes_t
{
    uint16_t count;
    int16_t axes[1];
};

struct JoystickPOV_t
{
    uint16_t count;
    int16_t povs[1];
};

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Send a console output line to the Driver Station
 * @param line a null-terminated string
 * @return 0 on success, other on failure
 */
int  FRC_NetworkCommunication_sendConsoleLine(const char* line);

/**
 * Send an error to the Driver Station
 * @param isError true if error, false if warning
 * @param errorCode value of error condition
 * @param isLVCode true if error code is defined in errors.txt, false if not (i.e. made up for C++)
 * @param details error description that contains details such as which resource number caused the failure
 * @param location Source file, function, and line number that the error was generated - optional
 * @param callStack The details about what functions were called through before the error was reported - optional
 * @return 0 on success, other on failure
 */
int  FRC_NetworkCommunication_sendError(int isError, int32_t errorCode, int isLVCode, const char* details, const char* location, const char* callStack);

void  setNewDataSem(pthread_cond_t*);
void  FRC_NetworkCommunication_setNewTcpDataSem(pthread_cond_t*);

// this uint32_t is really a LVRefNum
int  setNewDataOccurRef(uint32_t refnum);
int  FRC_NetworkCommunication_setNewTcpDataOccurRef(uint32_t refnum);
uint32_t  FRC_NetworkCommunication_getNewTcpRecvMask();

int  FRC_NetworkCommunication_getControlWord(struct ControlWord_t* controlWord); // Low-latency
int  FRC_NetworkCommunication_getJoystickAxes(uint8_t joystickNum, struct JoystickAxes_t* axes, uint8_t maxAxes); // Low-latency
int  FRC_NetworkCommunication_getJoystickButtons(uint8_t joystickNum, uint32_t* buttons, uint8_t* count); // Low-latency
int  FRC_NetworkCommunication_getJoystickPOVs(uint8_t joystickNum, struct JoystickPOV_t* povs, uint8_t maxPOVs); // Low-latency
int  FRC_NetworkCommunication_setJoystickOutputs(uint8_t joystickNum, uint32_t hidOutputs, uint16_t leftRumble, uint16_t rightRumble);
int  FRC_NetworkCommunication_getJoystickDesc(uint8_t joystickNum, uint8_t* isXBox, uint8_t* type, char* name, uint8_t* axisCount, uint8_t* axisTypes, uint8_t* buttonCount, uint8_t* povCount);
#ifdef __cplusplus
}
#endif
