
#include "Robot.h"
#include "Constants.h"
#include "motor/CAN.h"

static const auto SHOT_VELOCITY = 200;    //200_rpm;
static const auto TOLERANCE = 8;          // 8_rpm;
static const auto KICKER_THRESHOLD = 15;  // 15_mm;

//https://github.com/frc3512/Robot-2020/blob/b416c202794fb7deea0081beff2f986de7001ed9/docs/system-architecture.md?plain=1#L120
Robot::Robot() {
}
void Robot::RobotInit() {
    std::shared_ptr<CAN> can = std::make_shared<CAN>(DriveConstants::kFrontLeftDrivingCanId, HAL_CAN_Man_Dummy, HAL_CAN_Dev_karmController);
    uint8_t data[]={0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data,sizeof(data)/sizeof(uint8_t),apiId);
}

Robot::~Robot() {}

int main() {
    return StartRobot<Robot>();
}
