#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/SparseCore>
//#include <frc/controller/ImplicitModelFollower.h>
//#include <frc/estimator/AngleStatistics.h>
//#include <frc/estimator/KalmanFilterLatencyCompensator.h>
//#include <frc/estimator/UnscentedKalmanFilter.h>
//#include <frc/filter/LinearFilter.h>
//#include <frc/motorcontrol/MotorControllerGroup.h>
//#include <frc/system/plant/LinearSystemId.h>

//#include <rev/CANSparkMax.h>
#include "TrajectoryConfig.h"
#include "../controllers/DrivetrainController.h"
#include "ControlledSubsystemBase.h"

/**
 * The drivetrain subsystem.
 *
 * The drivetrain uses an unscented Kalman filter for state estimation.
 */
class Drivetrain : public ControlledSubsystemBase<7, 2, 5> {
public:
    /// The drivetrain length.  unit <meter>
    static constexpr float kLength = 0.9398;

    /**
     * Distance from middle of robot to intake. unit <meter>
     */
    static constexpr float kMiddleOfRobotToIntake = 0.656;

    /**
     * Producer-consumer queue for global pose measurements from Vision
     * subsystem.
     */
    //wpi::static_circular_buffer<Vision::GlobalMeasurement, 8> visionQueue;

    Drivetrain();

    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;


    /**
     * Returns left encoder displacement. unit<meter>
     */
    float GetLeftPosition() const;

    /**
     * Returns right encoder displacement. unit<meter>
     */
    float GetRightPosition() const;

    /**
     * Returns left encoder velocity. unit<meters_per_second>
     */
    float GetLeftVelocity() const;

    /**
     * Returns right encoder velocity.  unit<meters_per_second>
     */
    float GetRightVelocity() const;

    /**
     * Resets all sensors and controller.
     */
    void Reset(const Pose2d& initialPose = Pose2d());

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(float x, float y,long timestamp);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
//    void AddTrajectory(const frc::Pose2d& start,
//                       const std::vector<frc::Translation2d>& interior,
//                       const frc::Pose2d& end);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by MakeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
//    void AddTrajectory(const frc::Pose2d& start,
//                       const std::vector<frc::Translation2d>& interior,
//                       const frc::Pose2d& end,
//                       const frc::TrajectoryConfig& config);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     */
    void AddTrajectory(const std::vector<Pose2d>& waypoints);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by MakeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void AddTrajectory(const std::vector<Pose2d>& waypoints,
                       const TrajectoryConfig& config);

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint with the start and end velocities set to zero.
     */
    static TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory.
     * @param endVelocity   The end velocity of the trajectory.
     */
    static TrajectoryConfig MakeTrajectoryConfig(
            float startVelocity,
            float endVelocity);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Returns the drivetrain state estimate.
     */
    const Eigen::Vector<double, 7>& GetStates() const;

    /**
     * Returns the drivetrain inputs.
     */
    const Eigen::Vector<double, 2>& GetInputs() const;

    /**
     * Returns how many times the vision measurement was too far from the
     * drivetrain pose estimate.
     */
    int GetPoseMeasurementFaultCounter();

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void TestInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void ControllerPeriodic() override;

private:
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    float m_headingOffset = 0.0;

    DrivetrainController m_controller;
    Eigen::Vector<double, 2> m_u = Eigen::Vector<double, 2>::Zero();

    int m_poseMeasurementFaultCounter = 0;

    /**
     * Set drivetrain motors to brake mode, which the feedback controllers
     * expect.
     */
    void SetBrakeMode();

    /**
     * Set drivetrain motors to coast mode so the robot is easier to push when
     * it's disabled.
     */
    void SetCoastMode();
};
