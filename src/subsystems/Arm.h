#pragma once

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "common/TrajectoryConfig.h"
#include "controllers/ArmController.h"
#include "motor/CtrlStepMotor.h"
#include "robot/ControlledSubsystemBase.h"
#include "string"
#include <vector>
#define ALL 0

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
    ~Arm();

    Arm(const Arm &) = delete;
    Arm &operator=(const Arm &) = delete;

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
    void Reset(const Pose2d &initialPose = Pose2d());

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(float x, float y, long timestamp);

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
    void AddTrajectory(const std::vector<Pose2d> &waypoints);

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
    void AddTrajectory(const std::vector<Pose2d> &waypoints,
                       const TrajectoryConfig &config);

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
    const Eigen::Vector<double, 7> &GetStates() const;

    /**
     * Returns the Arm inputs.
     */
    const Eigen::Vector<double, 2> &GetInputs() const;

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

    ////Following part copied from the dummy robot //////
    /*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */
    //
    //
    //    class DummyRobot
    //    {
    //    public:
    //        explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    //
    //        ~DummyRobot();

    enum CommandMode {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };

    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -73, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 30;// degree/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;  // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;// 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;

    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor *motorJ[7] = {nullptr};

    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void SetRGBEnable(bool _enable);
    bool GetRGBEnabled();
    void SetRGBMode(uint32_t mode);
    uint32_t GetRGBMode();
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);

    class CommandHandler {
    public:
        explicit CommandHandler(Arm *_context) : context(_context) {
            //TODO            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();

    private:
        Arm *context;
        //TODO        osMessageQueueId_t commandFifo;
        int commandFifo;
        char strBuffer[64]{};
    };
    //        CommandHandler commandHandler = CommandHandler(this);

private:
    // CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;//by default
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic *dof6Solver;
    bool isEnabled = false;
    bool isRGBEnabled = false;
    uint32_t rgbMode = 0;
};
