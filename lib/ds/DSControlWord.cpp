#include "DSControlWord.h"
#include "DriverStation.h"

DSControlWord::DSControlWord() {
    HAL_GetControlWord(&m_controlWord);
}

bool DSControlWord::IsEnabled() const {
    return m_controlWord.enabled && m_controlWord.dsAttached;
}

bool DSControlWord::IsDisabled() const {
    return !(m_controlWord.enabled && m_controlWord.dsAttached);
}

bool DSControlWord::IsEStopped() const {
    return m_controlWord.eStop;
}

bool DSControlWord::IsAutonomous() const {
    return m_controlWord.autonomous;
}

bool DSControlWord::IsAutonomousEnabled() const {
    return m_controlWord.autonomous && m_controlWord.enabled && m_controlWord.dsAttached;
}

bool DSControlWord::IsTeleop() const {
    return !(m_controlWord.autonomous || m_controlWord.test);
}

bool DSControlWord::IsTeleopEnabled() const {
    return !m_controlWord.autonomous && !m_controlWord.test && m_controlWord.enabled && m_controlWord.dsAttached;
}

bool DSControlWord::IsTest() const {
    return m_controlWord.test;
}

bool DSControlWord::IsDSAttached() const {
    return m_controlWord.dsAttached;
}