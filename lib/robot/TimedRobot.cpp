#include "TimedRobot.h"
#include <utility>

void TimedRobot::StartCompetition() {
    RobotInit();

    // Loop forever, calling the appropriate mode-dependent function
    while (true) {
        // We don't have to check there's an element in the queue first because
        // there's always at least one (the constructor adds one). It's reenqueued
        // at the end of the loop.
        auto callback = m_callbacks.pop();

        //        int32_t status = 0;
        //        HAL_UpdateNotifierAlarm(m_notifier, callback.expirationTime.count(), &status);

        std::chrono::microseconds currentTime{2048};
        //        std::chrono::microseconds currentTime{ HAL_WaitForNotifierAlarm(m_notifier, &status)};
        //        if (currentTime.count() == 0 || status != 0) {
        //            break;
        //        }

        callback.func();

        // Increment the expiration time by the number of full periods it's behind
        // plus one to avoid rapid repeat fires from a large loop overrun. We assume
        // currentTime ≥ expirationTime rather than checking for it since the
        // callback wouldn't be running otherwise.
        //        callback.expirationTime += callback.period + (currentTime - callback.expirationTime) / callback.period * callback.period;
        m_callbacks.push(std::move(callback));

        // Process all other callbacks that are ready to run
        while (m_callbacks.top().expirationTime <= currentTime) {
            callback = m_callbacks.pop();

            callback.func();

            callback.expirationTime += callback.period + (currentTime - callback.expirationTime) / callback.period * callback.period;
            m_callbacks.push(std::move(callback));
        }
    }
}

TimedRobot::TimedRobot(int period) : IterativeRobotBase(period) {
    //    m_startTime = std::chrono::microseconds{RobotController::GetFPGATime()};
    m_startTime = std::chrono::microseconds{2400};
    AddPeriodic([=, this] { LoopFunc(); }, period);
    int32_t status = 0;
}
void TimedRobot::EndCompetition() {
    int32_t status = 0;
}

TimedRobot::~TimedRobot() {
}
void TimedRobot::AddPeriodic(std::function<void()> callback, int period, int offset) {
    m_callbacks.emplace(callback,
                        m_startTime,
                        std::chrono::microseconds{static_cast<int64_t>(period * 1e6)},
                        std::chrono::microseconds{static_cast<int64_t>(offset * 1e6)});
}
