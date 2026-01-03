#include "Gripper.h"
#include "motor/CtrlStepMotor.h"
#include "robot/RobotBase.h"
#include "spdlog/spdlog.h"

Gripper::Gripper() {
    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    can = std::make_shared<CAN>(0x1, HAL_CAN_Man_Dummy, HAL_CAN_Dev_kgripperController);
    Reset();
}

void Gripper::Reset() {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
}

//void Gripper::ControllerPeriodic() {
//
//}

void Gripper::RobotPeriodic() {
    spdlog::info("Gripper::RobotPeriodic");
}

void Gripper::DisabledInit() {
    SetBrakeMode();
    Disable();
}

void Gripper::AutonomousInit() {
    SetBrakeMode();
    Enable();
}

void Gripper::TeleopInit() {
    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    // m_controller.AbortTrajectories();

    // If the robot was disabled while still turning in place in
    // autonomous, it will continue to do so in teleop. This aborts any
    // turning action so teleop driving can occur.
    // AbortTurnInPlace();

    Enable();
}

void Gripper::TeleopPeriodic() {
    //static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    // scan the joystick and .
}

void Gripper::SetBrakeMode() {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
}

void Gripper::SetCoastMode() {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
}

/**
 * Enables the control loop.
 */
void Gripper::Enable() {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
    m_isEnabled = true;
}

/**
 * Disables the control loop.
 */
void Gripper::Disable() {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
    m_isEnabled = false;
}

/**
 * Set the angle of both fingers.
 */
void Gripper::SetAngle(float left_angle, float right_angle) {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
}

void Gripper::SetMaxCurrent(float _val) {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t apiId = 0x11;
    can->WritePacket(data, sizeof(data) / sizeof(uint8_t), apiId);
}
