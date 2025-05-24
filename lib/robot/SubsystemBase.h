#pragma once

#include <vector>
/**
 * A standardized base for subsystems.
 */
    class SubsystemBase {
    public:
        /**
         * Constructs a SubsystemBase.
         */
        SubsystemBase();

        /**
         * Copy constructor.
         */
        SubsystemBase(const SubsystemBase&) = default;

        /**
         * Copy assignment operator.
         */
        SubsystemBase& operator=(const SubsystemBase&) = default;

        /**
         * Move constructor.
         */
        SubsystemBase(SubsystemBase&&) = default;

        /**
         * Move assignment operator.
         */
        SubsystemBase& operator=(SubsystemBase&&) = default;

        virtual ~SubsystemBase();

        /**
         * Initialization code for disabled mode should go here.
         */
        virtual void DisabledInit() {}

        /**
         * Initialization code for autonomous mode should go here.
         */
        virtual void AutonomousInit() {}

        /**
         * Initialization code for teleop mode should go here.
         */
        virtual void TeleopInit() {}

        /**
         * Periodic code for all modes should go here.
         */
        virtual void RobotPeriodic() {}

        /**
         * Periodic code for disabled mode should go here.
         */
        virtual void DisabledPeriodic() {}

        /**
         * Periodic code for autonomous mode should go here.
         */
        virtual void AutonomousPeriodic() {}

        /**
         * Periodic code for teleop mode should go here.
         */
        virtual void TeleopPeriodic() {}

        /**
         * Call all subsystems's DisabledInit().
         */
        static void RunAllDisabledInit();

        /**
         * Call all subsystems's AutonomousInit().
         */
        static void RunAllAutonomousInit();

        /**
         * Call all subsystems's TeleopInit().
         */
        static void RunAllTeleopInit();

        /**
         * Call all subsystems's RobotPeriodic().
         */
        static void RunAllRobotPeriodic();

        /**
         * Call all subsystems's DisabledPeriodic().
         */
        static void RunAllDisabledPeriodic();

        /**
         * Call all subsystems's AutonomousPeriodic().
         */
        static void RunAllAutonomousPeriodic();

        /**
         * Call all subsystems's TeleopPeriodic().
         */
        static void RunAllTeleopPeriodic();

    private:
        static std::vector<SubsystemBase*> m_subsystems;

        /**
         * Consumes button edge events produced in disabled mode.
         */
        static void ConsumeButtonEdgeEvents();
    };