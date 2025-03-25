#pragma once

#include <chrono>
#include <functional>
#include <utility>
#include <vector>

//#include <hal/Notifier.h>
//#include <hal/Types.h>
//#include <units/math.h>
#include <queue>
#include "IterativeRobotBase.h"

/**
 * TimedRobot implements the IterativeRobotBase robot program framework.
 *
 * The TimedRobot class is intended to be subclassed by a user creating a
 * robot program.
 *
 * Periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
class TimedRobot : public IterativeRobotBase {
public:
    /// Default loop period.
    static constexpr auto kDefaultPeriod = 20;

    /**
     * Constructor for TimedRobot.
     *
     * @param period Period.
     */
    explicit TimedRobot(int period = kDefaultPeriod);

    TimedRobot(TimedRobot&&) = default;
    TimedRobot& operator=(TimedRobot&&) = default;

    ~TimedRobot() ;

    /**
     * Add a callback to run at a specific period with a starting time offset.
     *
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run synchronously. Interactions between them are thread-safe.
     *
     * @param callback The callback to run.
     * @param period   The period at which to run the callback.
     * @param offset   The offset from the common starting time. This is useful
     *                 for scheduling a callback in a different timeslot relative
     *                 to TimedRobot.
     */
    void AddPeriodic(std::function<void()> callback, int  period,int  offset = 0);
    void StartCompetition();
private:
    class Callback {
    public:
        std::function<void()> func;
        std::chrono::microseconds period;
        std::chrono::microseconds expirationTime;

        /**
         * Construct a callback container.
         *
         * @param func      The callback to run.
         * @param startTime The common starting point for all callback scheduling.
         * @param period    The period at which to run the callback.
         * @param offset    The offset from the common starting time.
         */
        Callback(std::function<void()> func, std::chrono::microseconds startTime,
                 std::chrono::microseconds period, std::chrono::microseconds offset)
                : func{std::move(func)},
                  period{period},
                  expirationTime(( startTime + offset + period +  startTime) / period * period) {}

        bool operator>(const Callback& rhs) const {
            return expirationTime > rhs.expirationTime;
        }
    };

    std::chrono::microseconds m_startTime;
    uint64_t m_loopStartTimeUs = 0;

    std::priority_queue<Callback, std::vector<Callback>, std::greater<Callback>>  m_callbacks;
};