#include "GenericHID.h"
#include <string>
#include "DriverStation.h"
#include "BooleanEvent.h"

GenericHID::GenericHID(int port) {
    m_port = port;
}

bool GenericHID::GetRawButton(int button) const {
    return DriverStation::GetStickButton(m_port, button);
}

bool GenericHID::GetRawButtonPressed(int button) {
    return DriverStation::GetStickButtonPressed(m_port, button);
}

bool GenericHID::GetRawButtonReleased(int button) {
    return DriverStation::GetStickButtonReleased(m_port, button);
}

BooleanEvent GenericHID::Button(int button, EventLoop* loop) const {
    return BooleanEvent(loop, [this, button]() { return this->GetRawButton(button); });
}

double GenericHID::GetRawAxis(int axis) const {
    return DriverStation::GetStickAxis(m_port, axis);
}

int GenericHID::GetPOV(int pov) const {
    return DriverStation::GetStickPOV(m_port, pov);
}

BooleanEvent GenericHID::POV(int angle, EventLoop* loop) const {
    return POV(0, angle, loop);
}

BooleanEvent GenericHID::POV(int pov, int angle, EventLoop* loop) const {
    return BooleanEvent( loop, [this, pov, angle] { return this->GetPOV(pov) == angle; });
}

BooleanEvent GenericHID::POVUp(EventLoop* loop) const {
    return POV(0, loop);
}

BooleanEvent GenericHID::POVUpRight(EventLoop* loop) const {
    return POV(45, loop);
}

BooleanEvent GenericHID::POVRight(EventLoop* loop) const {
    return POV(90, loop);
}

BooleanEvent GenericHID::POVDownRight(EventLoop* loop) const {
    return POV(135, loop);
}

BooleanEvent GenericHID::POVDown(EventLoop* loop) const {
    return POV(180, loop);
}

BooleanEvent GenericHID::POVDownLeft(EventLoop* loop) const {
    return POV(225, loop);
}

BooleanEvent GenericHID::POVLeft(EventLoop* loop) const {
    return POV(270, loop);
}

BooleanEvent GenericHID::POVUpLeft(EventLoop* loop) const {
    return POV(315, loop);
}

BooleanEvent GenericHID::POVCenter(EventLoop* loop) const {
    return POV(360, loop);
}

BooleanEvent GenericHID::AxisLessThan(int axis, double threshold, EventLoop* loop) const {
    return BooleanEvent(loop, [this, axis, threshold]() {
        return this->GetRawAxis(axis) < threshold;
    });
}

BooleanEvent GenericHID::AxisGreaterThan(int axis, double threshold, EventLoop* loop) const {
    return BooleanEvent(loop, [this, axis, threshold]() {
        return this->GetRawAxis(axis) > threshold;
    });
}

int GenericHID::GetAxisCount() const {
    return DriverStation::GetStickAxisCount(m_port);
}

int GenericHID::GetPOVCount() const {
    return DriverStation::GetStickPOVCount(m_port);
}

int GenericHID::GetButtonCount() const {
    return DriverStation::GetStickButtonCount(m_port);
}

bool GenericHID::IsConnected() const {
    return DriverStation::IsJoystickConnected(m_port);
}

GenericHID::HIDType GenericHID::GetType() const {
    return static_cast<HIDType>(DriverStation::GetJoystickType(m_port));
}

std::string GenericHID::GetName() const {
    return DriverStation::GetJoystickName(m_port);
}

int GenericHID::GetAxisType(int axis) const {
    return DriverStation::GetJoystickAxisType(m_port, axis);
}

int GenericHID::GetPort() const {
    return m_port;
}