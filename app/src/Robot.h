#include <string>
#include <string_view>
#include <vector>
#include "ds/XboxController.h"
#include "ds/EventLoop.h"
#include "ds/BooleanEvent.h"
#include "robot/TimedRobot.h"
#include "Drivetrain.h"

/**
 * The main robot class.
 */
class Robot : public TimedRobot {
public:
    // The subsystem initialization order determines the controller run order.

    /// Drivetrain subsystem.
    //Drivetrain drivetrain;

    /// Vision subsystem
//    Vision vision;


    Robot();

    ~Robot();

    void RobotInit();
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
     * Periodic code for all modes should go here.
     */
    void RobotPeriodic() override { m_loop.Poll(); }


    /**
     * Periodic code for disabled mode should go here.
     */
//    void DisabledPeriodic() override;

    /**
     * Periodic code for autonomous mode should go here.
     */
    void AutonomousPeriodic() override { DriveWithJoystick(true); }

    /**
     * Periodic code for teleop mode should go here.
     */
    void TeleopPeriodic() override { DriveWithJoystick(true); }

    /**
     * No-op autonomous.
     */
    void AutoNoOp();

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

    void DriveWithJoystick(__attribute__((unused)) bool fieldRelative) {
        //std::cout <<"Robot DriveWithJoystick" << std::endl;
        const auto xSpeed = m_controller.GetLeftY();
        const auto ySpeed = m_controller.GetLeftX();
        const auto rot = m_controller.GetRightX();
        BooleanEvent intakeButton{ &m_loop, [&joystick = m_joystick] { return joystick.GetRawButton(2); }};
        BooleanEvent shootTrigger{ &m_loop, [&joystick = m_joystick] { return joystick.GetRawButtonPressed(2);}};
    }

private:
    EventLoop m_loop{};
    XboxController m_controller{0};
    GenericHID m_joystick{0};
};