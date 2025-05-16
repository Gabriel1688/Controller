#pragma once
#include <chrono>
#include <pthread.h>
#include "../ds/DriverStation.h"
#include <condition_variable>
#include <mutex>
#include <string>

void InitializeHAL();
extern "C" {
namespace hal {
    void InitializeDriverStation();
}
}

template <class Robot>
void RunRobot(std::mutex& m, Robot** robot) {
     static Robot theRobot;
     {
         std::scoped_lock lock{m};
         *robot = &theRobot;
     }
     theRobot.StartCompetition();
}

template <class Robot>
int StartRobot() {
    static std::mutex m;
    static Robot* robot = nullptr;
    InitializeHAL();
    hal::InitializeDriverStation();
    RunRobot<Robot>(m, &robot);
    return 0;
}

/**
 * Implement a Robot Program framework. The RobotBase class is intended to be
 * subclassed to create a robot program. The user must implement
 * StartCompetition() which will be called once and is not expected to exit. The
 * user must also implement EndCompetition(), which signals to the code in
 * StartCompetition() that it should exit.
 *
 * It is not recommended to subclass this class directly - instead subclass
 * IterativeRobotBase or TimedRobot.
 */
class RobotBase {
public:
    /**
     * Determine if the Robot is currently enabled.
     *
     * @return True if the Robot is currently enabled by the Driver Station.
     */
    bool IsEnabled() const;

    /**
     * Determine if the Robot is currently disabled.
     *
     * @return True if the Robot is currently disabled by the Driver Station.
     */
    bool IsDisabled() const;

    /**
     * Determine if the robot is currently in Autonomous mode.
     *
     * @return True if the robot is currently operating Autonomously as determined
     *         by the Driver Station.
     */
    bool IsAutonomous() const;

    /**
     * Determine if the robot is currently in Autonomous mode and enabled.
     *
     * @return True if the robot us currently operating Autonomously while enabled
     * as determined by the Driver Station.
     */
    bool IsAutonomousEnabled() const;

    /**
     * Determine if the robot is currently in Operator Control mode.
     *
     * @return True if the robot is currently operating in Tele-Op mode as
     *         determined by the Driver Station.
     */
    bool IsTeleop() const;

    /**
     * Determine if the robot is current in Operator Control mode and enabled.
     *
     * @return True if the robot is currently operating in Tele-Op mode while
     * enabled as determined by the Driver Station.
     */
    bool IsTeleopEnabled() const;

    /**
     * Determine if the robot is currently in Test mode.
     *
     * @return True if the robot is currently running in Test mode as determined
     * by the Driver Station.
     */
    bool IsTest() const;

    /**
     * Determine if the robot is current in Test mode and enabled.
     *
     * @return True if the robot is currently operating in Test mode while
     * enabled as determined by the Driver Station.
     */
    bool IsTestEnabled() const;

    /**
     * Returns the main thread ID.
     *
     * @return The main thread ID.
     */
//    static std::thread::id GetThreadId();

    /**
     * Start the main robot code. This function will be called once and should not
     * exit until signalled by EndCompetition()
     */
    virtual void StartCompetition() = 0;

    /** Ends the main loop in StartCompetition(). */
    virtual void EndCompetition() = 0;

    /**
     * Get the current runtime type.
     *
     * @return Current runtime type.
     */
    //static RuntimeType GetRuntimeType();

    /**
     * Get if the robot is real.
     *
     * @return If the robot is running in the real world.
     */
    static constexpr bool IsReal() {
#ifdef __FRC_ROBORIO__
        return true;
#else
        return false;
#endif
    }

    /**
     * Get if the robot is a simulation.
     *
     * @return If the robot is running in simulation.
     */
    static constexpr bool IsSimulation() {
#ifdef __FRC_ROBORIO__
        return false;
#else
        return true;
#endif
    }

    /**
     * Constructor for a generic robot program.
     *
     * User code can be placed in the constructor that runs before the
     * Autonomous or Operator Control period starts. The constructor will run to
     * completion before Autonomous is entered.
     *
     * This must be used to ensure that the communications code starts. In the
     * future it would be nice to put this code into it's own task that loads on
     * boot so ensure that it runs.
     */
    RobotBase();

    virtual ~RobotBase() = default;

protected:
    RobotBase(RobotBase&&) = default;
    RobotBase& operator=(RobotBase&&) = default;

    pthread_t m_threadId;
//    NT_Listener connListenerHandle;
    bool m_dashboardDetected = false;
};

