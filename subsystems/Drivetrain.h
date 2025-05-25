#pragma once
#include <limits>
#include <vector>
#include "Constants.h"
#include "DrivetrainController.h"
#include "robot/ControlledSubsystemBase.h"

/**
 * The drivetrain subsystem.
 *
 * The drivetrain uses a Kalman Filter for encoder position and velocity
 * estimation and the DifferentialDriveOdometry class for pose estimation.
 */
    class Drivetrain : public ControlledSubsystemBase<7, 2, 5> {
    public:
        Drivetrain();

        Drivetrain(const Drivetrain&) = delete;
        Drivetrain& operator=(const Drivetrain&) = delete;
#if 0

        /**
         * Returns the drivetrain's reference pose.
         */
        frc::Pose2d GetReferencePose() const;

        /**
         * Returns the drivetrain's pose estimate.
         */
        frc::Pose2d GetPose() const;

        /**
         * Returns gyro's heading measurement in the global coordinate frame.
         */
        units::radian_t GetAngle() const;

        /**
         * Returns left encoder velocity.
         */
        units::meters_per_second_t GetLeftVelocity() const;

        /**
         * Returns right encoder velocity.
         */
        units::meters_per_second_t GetRightVelocity() const;

        /**
         * Resets all sensors and controller.
         */
        void Reset(const frc::Pose2d& initialPose = frc::Pose2d());

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
        void AddTrajectory(const frc::Pose2d& start,
                           const std::vector<frc::Translation2d>& interior,
                           const frc::Pose2d& end);

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
        void AddTrajectory(const frc::Pose2d& start,
                           const std::vector<frc::Translation2d>& interior,
                           const frc::Pose2d& end,
                           const frc::TrajectoryConfig& config);

        /**
         * Adds a trajectory with the given waypoints.
         *
         * This can be called more than once to create a queue of trajectories.
         * Closed-loop control will be enabled to track the first trajectory.
         *
         * @param waypoints Waypoints.
         */
        void AddTrajectory(const std::vector<frc::Pose2d>& waypoints);

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
        void AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                           const frc::TrajectoryConfig& config);

        /**
         * Adds a trajectory with a pre-determined trajectory.
         *
         * This can be called more than once to create a queue of trajectories.
         * Closed-loop control will be enabled to track the first trajectory.
         *
         * @param trajectory Trajectory.
         * @param config    TrajectoryConfig for this trajectory. This can include
         *                  constraints on the trajectory dynamics. If adding custom
         *                  constraints, it is recommended to start with the config
         *                  returned by MakeTrajectoryConfig() so differential drive
         *                  dynamics constraints are included automatically.
         */
        void AddTrajectory(
                const frc::Trajectory trajectory,
                const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

        /**
         * Sets a new heading goal for the drivetrain to achieve.
         *
         * @param heading The heading you want to the drivetrain to achieve (in
         * radians)
         */
        void SetHeadingGoal(const units::radian_t heading);

        /**
         * Returns whether or not a new heading goal is set.
         */
        bool HasHeadingGoal() const;

        /**
         * Aborts turning in place.
         */
        void AbortTurnInPlace();

        /**
         * Returns a TrajectoryConfig containing a differential drive dynamics
         * constraint with the start and end velocities set to zero.
         */
        static frc::TrajectoryConfig MakeTrajectoryConfig();

        /**
         * Returns a TrajectoryConfig containing a differential drive dynamics
         * constraint and the specified start and end velocities.
         *
         * @param startVelocity The start velocity of the trajectory.
         * @param endVelocity   The end velocity of the trajectory.
         */
        static frc::TrajectoryConfig MakeTrajectoryConfig(
                units::meters_per_second_t startVelocity,
                units::meters_per_second_t endVelocity);

        /**
         * Returns whether the drivetrain controller is at the goal waypoint.
         */
        bool AtGoal() const;

        /**
         * Returns whether the drivetrain is at the goal heading.
         */
        bool AtHeading();

        /**
         * Returns the current heading state. Used for setting points after
         * turn-in-places
         */
        units::radian_t GetHeading();

        /**
         * Sets the tolerance of the turn-in-place ProfiledPID.
         *
         * @param headingTolerance The tolerance on the heading.
         * @param velocityTolerance The tolerance on the velocity.
         */
        void SetTurningTolerance(units::radian_t headingTolerance,
                                 units::radians_per_second_t velocityTolerance =
                                 units::radians_per_second_t{
                                         std::numeric_limits<double>::infinity()});

        /**
         * Sets the constraint of the turn-in-place ProfiledPID.
         *
         * @param constraint the new constraint of the controller.
         */
        void SetTurningConstraints(
                frc::TrapezoidProfile<units::radian>::Constraints constraint);

        /**
         * Returns the drivetrain state estimate.
         */
        const Eigen::Vector<double, 7>& GetStates();

        /**
         * Returns the drivetrain inputs.
         */
        const Eigen::Vector<double, 2>& GetInputs() const;

        /**
         * Returns whether or not the robot is moving.
         */
        bool IsStationary();

        void DisabledInit() override;

        void AutonomousInit() override;

        void TeleopInit() override;

        void TestInit() override;

        void RobotPeriodic() override;

        void TeleopPeriodic() override;

        void TestPeriodic() override;

        void ControllerPeriodic() override;
#endif
    private:
        static constexpr double kTurningP = 3.0;
        static constexpr double kTurningI = 0.0;
        static constexpr double kTurningD = 0.0;
#if 0
        static const Eigen::Matrix<double, 2, 2> kGlobalR;

        static frc::LinearSystem<2, 2, 2> kPlant;

        rev::CANSparkMax m_leftLeader{HWConfig::Drivetrain::kLeftMotorLeaderID,
                                      rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_leftFollower{HWConfig::Drivetrain::kLeftMotorFollowerID,
                                        rev::CANSparkMax::MotorType::kBrushless};
        frc::MotorControllerGroup m_leftGrbx{m_leftLeader, m_leftFollower};

        rev::CANSparkMax m_rightLeader{HWConfig::Drivetrain::kRightMotorLeaderID,
                                       rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax m_rightFollower{
                HWConfig::Drivetrain::kRightMotorFollowerID,
                rev::CANSparkMax::MotorType::kBrushless};
        frc::MotorControllerGroup m_rightGrbx{m_rightLeader, m_rightFollower};

        frc::Encoder m_leftEncoder{HWConfig::Drivetrain::kLeftEncoderA,
                                   HWConfig::Drivetrain::kLeftEncoderB};

        frc::Encoder m_rightEncoder{HWConfig::Drivetrain::kRightEncoderA,
                                    HWConfig::Drivetrain::kRightEncoderB};

        frc::ADIS16470_IMU m_imu;
        units::radian_t m_headingOffset = 0_rad;

        units::meter_t m_leftPos;
        units::meter_t m_lastLeftPos;
        units::meter_t m_rightPos;
        units::meter_t m_lastRightPos;
        units::second_t m_time = frc::Timer::GetFPGATimestamp();
        units::second_t m_lastTime = m_time - Constants::kControllerPeriod;

        units::meters_per_second_t m_leftVelocity;
        units::meters_per_second_t m_rightVelocity;
        // Filters out encoder quantization noise
        frc::LinearFilter<units::meters_per_second_t> m_leftVelocityFilter =
                frc::LinearFilter<units::meters_per_second_t>::MovingAverage(4);
        frc::LinearFilter<units::meters_per_second_t> m_rightVelocityFilter =
                frc::LinearFilter<units::meters_per_second_t>::MovingAverage(4);

        frc::KalmanFilter<2, 2, 2> m_velocityObserver{
                kPlant,
                {0.25, 0.25},
                {DrivetrainController::kDpP / Constants::kControllerPeriod.value(),
                 DrivetrainController::kDpP / Constants::kControllerPeriod.value()},
                Constants::kControllerPeriod};

        frc::DifferentialDriveOdometry m_observer{frc::Rotation2d(), frc::Pose2d()};
        Eigen::Vector<double, 7> m_xHat = Eigen::Vector<double, 7>::Zero();

        DrivetrainController m_controller;
        Eigen::Vector<double, 2> m_u = Eigen::Vector<double, 2>::Zero();

        frc3512::Vision vision;

        frc::Timer m_visionTimer;
        bool m_aimWithVision = false;
        bool m_atVisionTarget = false;

        frc::TrapezoidProfile<units::radian>::Constraints m_turningConstraints{
                6_rad_per_s, 3.3_rad_per_s_sq};
        frc::ProfiledPIDController<units::radian> m_turningPID{
                kTurningP, kTurningI, kTurningD, m_turningConstraints,
                Constants::kControllerPeriod};
        bool m_hasNewHeading = false;
        frc::SimpleMotorFeedforward<units::radian> m_turningFeedforward{
                0.17964_V, 2.6447_V / 1_rad_per_s};
        frc2::PIDController m_aimPID{kTurningP, kTurningI, kTurningD};

        frc::DifferentialDrive m_visionAim{m_leftGrbx, m_rightGrbx};
        frc2::PIDController m_visionController{1.5, kTurningI, 0.25};

        frc::LinearSystem<2, 2, 2> m_imfRef =
                frc::LinearSystemId::IdentifyDrivetrainSystem(
                        DrivetrainController::kLinearV, DrivetrainController::kLinearA,
                        DrivetrainController::kAngularV, DrivetrainController::kAngularA);
        frc::ImplicitModelFollower<2, 2> m_imf{kPlant, m_imfRef, 20_ms};

        // Simulation variables
        frc::sim::DifferentialDrivetrainSim m_drivetrainSim{
                DrivetrainController::GetPlant(), DrivetrainController::kWidth,
                frc::DCMotor::NEO(2), DrivetrainController::kDriveGearRatio,
                DrivetrainController::kWheelRadius};
        frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
        frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
        frc::sim::ADIS16470_IMUSim m_imuSim{m_imu};
        frc::Field2d m_field;

        nt::NetworkTableEntry m_headingGoalEntry = NetworkTableUtil::MakeBoolEntry(
                "/Diagnostics/Drivetrain/Outputs/Goal Heading Achieved");

        nt::NetworkTableEntry m_yawControllerEntry =
                NetworkTableUtil::MakeDoubleEntry(
                        "/Diagnostics/Drivetrain/Outputs/Controller Yaw Value");

        nt::NetworkTableEntry m_rangeControllerEntry =
                NetworkTableUtil::MakeDoubleEntry(
                        "/Diagnostics/Drivetrain/Outputs/Controller Range Value");

        nt::NetworkTableEntry m_hasHeadingGoalEntry =
                NetworkTableUtil::MakeBoolEntry(
                        "/Diagnostics/Drivetrain/Outputs/Has New Goal Heading");

        nt::NetworkTableEntry m_currHeadingEntry =
                NetworkTableUtil::MakeDoubleEntry(
                        "/Diagnostics/Drivetrain/Outputs/Current Heading");

        nt::NetworkTableEntry m_headingGoalValueEntry =
                NetworkTableUtil::MakeDoubleEntry(
                        "/Diagnostics/Drivetrain/Outputs/Heading Goal");

        nt::NetworkTableEntry m_rotationEntry =
                NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Drivetrain/Rotation");
        nt::NetworkTableEntry m_isStationaryEntry = NetworkTableUtil::MakeBoolEntry(
                "/Diagnostics/Drivetrain/Outputs/Is Stationary");

        /**
         * Set drivetrain motors to brake mode, which the feedback controllers
         * expect.
         */
        void SetBrakeMode();
#endif
};