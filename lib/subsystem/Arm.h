#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "TrajectoryConfig.h"
#include "controllers/ArmController.h"
#include "ControlledSubsystemBase.h"

/**
 * The Arm subsystem.
 *
 * The Arm uses an unscented Kalman filter for state estimation.
 */
class Arm : public ControlledSubsystemBase<7, 2, 5> {
public:
    /// The Arm length.  unit <meter>
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

    Arm();

    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;


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
     * Returns whether the Arm controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Returns the Arm state estimate.
     */
    const Eigen::Vector<double, 7>& GetStates() const;

    /**
     * Returns the Arm inputs.
     */
    const Eigen::Vector<double, 2>& GetInputs() const;

    /**
     * Returns how many times the vision measurement was too far from the
     * Arm pose estimate.
     */
    int GetPoseMeasurementFaultCounter();

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void ControllerPeriodic() override;

private:
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    float m_headingOffset = 0.0;

    ArmController m_controller;
    Eigen::Vector<double, 2> m_u = Eigen::Vector<double, 2>::Zero();

    int m_poseMeasurementFaultCounter = 0;

    /**
     * Set Arm motors to brake mode, which the feedback controllers
     * expect.
     */
    void SetBrakeMode();

    /**
     * Set Arm motors to coast mode so the robot is easier to push when
     * it's disabled.
     */
    void SetCoastMode();
};
