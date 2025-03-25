#include "FRCComm.h"
#ifdef __cplusplus
extern "C"
{
#endif

int  FRC_NetworkCommunication_getJoystickAxes(uint8_t joystickNum, struct JoystickAxes_t* axes, uint8_t maxAxes) {
    return 0;
}

int  FRC_NetworkCommunication_getJoystickPOVs(uint8_t joystickNum, struct JoystickPOV_t* povs, uint8_t maxPOVs) {
    return 0;
}
int  FRC_NetworkCommunication_getJoystickButtons(uint8_t joystickNum, uint32_t* buttons, uint8_t* count){
    return  0;
}

int  FRC_NetworkCommunication_getControlWord(struct ControlWord_t* controlWord){
    return 0;
}

int  FRC_NetworkCommunication_getJoystickDesc(uint8_t joystickNum, uint8_t* isXBox, uint8_t* type, char* name, uint8_t* axisCount, uint8_t* axisTypes, uint8_t* buttonCount, uint8_t* povCount)
{
    return 0;
}

int  FRC_NetworkCommunication_setJoystickOutputs(uint8_t joystickNum, uint32_t hidOutputs, uint16_t leftRumble, uint16_t rightRumble){
    return 0;
}


///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:12: undefined reference to `DriverStation::GetStickButton(int, int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:16: undefined reference to `DriverStation::GetStickButtonPressed(int, int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:20: undefined reference to `DriverStation::GetStickButtonReleased(int, int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:28: undefined reference to `DriverStation::GetStickAxis(int, int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:32: undefined reference to `DriverStation::GetStickPOV(int, int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:92: undefined reference to `DriverStation::GetStickAxisCount(int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:96: undefined reference to `DriverStation::GetStickPOVCount(int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:100: undefined reference to `DriverStation::GetStickButtonCount(int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:104: undefined reference to `DriverStation::IsJoystickConnected(int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:108: undefined reference to `DriverStation::GetJoystickType(int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:112: undefined reference to `DriverStation::GetJoystickName[abi:cxx11](int)'
///home/gabriel_wang/work/Controller/lib/ds/GenericHID.cpp:116: undefined reference to `DriverStation::GetJoystickAxisType(int, int)'
///home/gabriel_wang/work/Controller/lib/framework/IterativeRobotBase.cpp:51: undefined reference to `DriverStation::RefreshData()'
///opt/rh/gcc-toolset-10/root/usr/bin/ld: CMakeFiles/Controller.dir/lib/framework/IterativeRobotBase.cpp.o:(.data.rel.ro._ZTV18IterativeRobotBase[_ZTV18IterativeRobotBase]+0x58): undefined reference to `IterativeRobotBase::StartCompetition()'

#ifdef __cplusplus
}
#endif