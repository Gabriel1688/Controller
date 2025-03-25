
#include "RobotBase.h"

RobotBase::RobotBase() {
    m_threadId = (unsigned long) pthread_self();

//    SetupMathShared();

    // Call DriverStation::RefreshData() to kick things off
    DriverStation::RefreshData();
}
