#include <string>
#include <string_view>
#include <vector>

#include "robot/TimedRobot.h"
#include "../../subsystems/Drivetrain.h"

/**
 * The main robot class.
 */
class Robot : public TimedRobot {
public:
    /**
     * States used for the multi-subsystem shooting procedure
     */
    enum class ShootingState {
        kIdle,
        kVisionSpinUp,
        kSpinUp,
        kVisionAim,
        kStartConveyor,
        kEndShoot
    };

    // The subsystem initialization order determines the controller run order.

    /// Drivetrain subsystem.
    //Drivetrain drivetrain;

    /// Vision subsystem
//    Vision vision;


    Robot();

    ~Robot();

    /**
     * Robot-wide simulation initialization code should go here.
     *
     * Users should override this method for default Robot-wide simulation
     * related initialization which will be called when the robot is first
     * started. It will be called exactly one time after RobotInit is called
     * only when the robot is in simulation.
     */
//    void SimulationInit() override;

    /**
     * Initialization code for disabled mode should go here.
     */
//    void DisabledInit() override;

    /**
     * Initialization code for autonomous mode should go here.
     */
//    void AutonomousInit() override;

    /**
     * Initialization code for teleop mode should go here.
     */
//    void TeleopInit() override;

    /**
     * Initialization coe for test mode should go here.
     */
//    void TestInit() override;

    /**
     * Periodic code for all modes should go here.
     */
//    void RobotPeriodic() override;

    /**
     * Periodic simulation code should go here.
     *
     * This function is called in a simulated robot after user code executes.
     */
//    void SimulationPeriodic() override;

    /**
     * Periodic code for disabled mode should go here.
     */
//    void DisabledPeriodic() override;

    /**
     * Periodic code for autonomous mode should go here.
     */
//    void AutonomousPeriodic() override;

    /**
     * Periodic code for teleop mode should go here.
     */
//    void TeleopPeriodic() override;



    /**
     * No-op autonomous.
     */
    void AutoNoOp();

    /**
     * Robot shoots one ball from the agianst the hub and drives off the tarmac.
     */
    void AutoShootOne();

    /**
     * Drive backwards autonomous.
     */
    void AutoBackwards();

    /**
     * Shoot two autonomous.
     */
    void AutoShootTwo();

    /**
     * Shoot three autonomous.
     */
    void AutoShootThree();

    /**
     * Shoot four autonomous.
     */
    void AutoShootFour();

    /**
     * Returns a pose with the same x and y coordinates, but an updated heading.
     * A utility function for autonomous positions used when the robot turns in
     * place
     *
     * @param pose the position object being updated.
     * @param newHeading The new heading of the position.
     */
//    frc::Pose2d UpdateAutoPoseRotation(const frc::Pose2d& pose, units::radian_t newHeading);

    /**
     * Sets the selected autonomous mode for testing purposes.
     *
     * @param name The autonomous mode's name passed to
     *             AutonomousChooser::AddAutonomous().
     */
    void SelectAutonomous(std::string_view name);

    /**
     * Returns the names of autonomous modes to test.
     */
    const std::vector<std::string>& GetAutonomousNames() const;

    /**
     * Assertions to check at the end of each autonomous mode during unit
     * testing.
     */
    void ExpectAutonomousEndConds();

private:
};

#if 0
class Robot : public TimedRobot {
public:
    void AutonomousPeriodic() override {
        DriveWithJoystick(false);
    }

    void RobotInit() {
        std::shared_ptr<CAN> can = std::make_shared<CAN>(DriveConstants::kFrontLeftDrivingCanId, HAL_CAN_Man_Dummy, HAL_CAN_Dev_karmController);
        uint8_t data[]={0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFC};
        uint8_t apiId = 0x11;
        can->WritePacket(data,sizeof(data)/sizeof(uint8_t),apiId);
    }

    void TeleopPeriodic() override { DriveWithJoystick(true); }


    void DriveWithJoystick(__attribute__((unused)) bool fieldRelative) {
        //std::cout <<"Robot DriveWithJoystick" << std::endl;
        const auto xSpeed = m_controller.GetLeftY();
        const auto ySpeed = m_controller.GetLeftX();
        const auto rot = m_controller.GetRightX();



/*
    https://github.com/wpilibsuite/allwpilib/blob/d32e60233fe516e8f67e0e94d3de87615e09f00f/wpilibcExamples/src/main/cpp/examples/EventLoop/cpp/Robot.cpp#L12
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        const auto xSpeed = -m_xspeedLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetLeftY(), 0.02)) *
                            Drivetrain::kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        const auto ySpeed = -m_yspeedLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetLeftX(), 0.02)) *
                            Drivetrain::kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        const auto rot = -m_rotLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetRightX(), 0.02)) *
                         Drivetrain::kMaxAngularSpeed;

        m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
*/

        BooleanEvent intakeButton{ &m_loop, [&joystick = m_joystick] { return joystick.GetRawButton(2); }};
        BooleanEvent shootTrigger{ &m_loop, [&joystick = m_joystick] { return joystick.GetRawButtonPressed(2);}};
    }

    void RobotPeriodic() override { m_loop.Poll(); }

private:
    EventLoop m_loop{};
    XboxController m_controller{0};
    GenericHID m_joystick{0};
};
#endif