#include "Eigen/Core"
#include <algorithm>

#include "Arm.h"
#include "ds/DriverStation.h"
#include "motor/CtrlStepMotor.h"
#include "robot/ControlledSubsystemBase.h"
#include "robot/RobotBase.h"

Arm::Arm() {
    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(Pose2d{0.0, 0.0, 0.0});
    //    rev::CANSparkMax m_leftFollower{HWConfig::Drivetrain::kLeftMotorFollowerID,
    //                                    rev::CANSparkMax::MotorType::kBrushless};

    motorJ[ALL] = new CtrlStepMotor(0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(1, true, 30, -170, 170);
    motorJ[2] = new CtrlStepMotor(2, false, 30, -73, 90);
    motorJ[3] = new CtrlStepMotor(3, true, 30, 35, 180);
    motorJ[4] = new CtrlStepMotor(4, false, 24, -180, 180);
    motorJ[5] = new CtrlStepMotor(5, true, 30, -120, 120);
    motorJ[6] = new CtrlStepMotor(6, true, 50, -720, 720);

    dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
    std::cout << "enter DummyRobot()." << std::endl;
}

//
//Pose2d Arm::GetReferencePose() const {
//    const auto& x = m_controller.GetReferences();
//    return Pose2d{
//            float {x(ArmController::State::kX)},
//            float {x(ArmController::State::kY)},
//            float {x(ArmController::State::kHeading)}};
//}

//Pose2d Arm::GetPose() const { return m_observer.GetPose(); }

void Arm::Reset(const Pose2d &initialPose) {
}

void Arm::ControllerPeriodic() {
}

void Arm::RobotPeriodic() {
}

const Eigen::Vector<double, 2> &Arm::GetInputs() const {
    return m_controller.GetInputs();
}

void Arm::DisabledInit() {
    SetBrakeMode();
    Disable();
}

void Arm::AutonomousInit() {
    SetBrakeMode();
    //SetTurningTolerance(0.25);
    Enable();
}

void Arm::TeleopInit() {
    //---get from dummy robot.
    //    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
    ////

    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    // m_controller.AbortTrajectories();

    // If the robot was disabled while still turning in place in
    // autonomous, it will continue to do so in teleop. This aborts any
    // turning action so teleop driving can occur.
    // AbortTurnInPlace();

    Enable();
}

void Arm::TeleopPeriodic() {
    /*
    using Input = ArmController::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y = frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x = frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick2.GetRawButton(2)) {
        x *= 0.4;
    }

    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
            y, x, driveStick2.GetRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
            m_imf.Calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                     GetRightVelocity().value()},
                            Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    if (!IsVisionAiming()) {
        m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
        m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});
    }

    m_headingGoalEntry.SetBoolean(AtHeading());
    m_hasHeadingGoalEntry.SetBoolean(HasHeadingGoal());

    m_yawControllerEntry.SetDouble(GetVisionYaw().value());
    m_rangeControllerEntry.SetDouble(m_controller.GetVisionRange().value());
*/
}

void Arm::SetBrakeMode() {
}

void Arm::SetCoastMode() {
}

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index) {
    float max = -1;
    for (uint8_t i = 0; i < 6; i++) {
        if (abs(_joints.a[i]) > max) {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}

Arm::~Arm() {
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    //    delete hand;
    delete dof6Solver;
}

void Arm::MoveJoints(DOF6Kinematic::Joint6D_t _joints) {
    for (int j = 1; j <= 6; j++) {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1], dynamicJointSpeeds.a[j - 1]);
        std::cout << "Move Joints, [" << j << " ],  Angle[ " << _joints.a[j - 1] - initPose.a[j - 1]
                  << " ], VelocityLimit [" << dynamicJointSpeeds.a[j - 1] << "]." << std::endl;
    }
}

bool Arm::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6) {
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++) {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax || targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid) {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        //float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        // TODO:: motorJ without the ctrlstep instance.
        float time = 0;// maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++) {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f);//0~10r/s
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}

bool Arm::MoveL(float _x, float _y, float _z, float _a, float _b, float _c) {
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++) {
        valid[i] = true;

        //        for (int j = 1; j <= 6; j++)
        //        {
        //            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
        //                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
        //            {
        //                valid[i] = false;
        //                continue;
        //            }
        //        }

        if (valid[i])
            validCnt++;
    }

    if (validCnt) {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++) {
            if (valid[i]) {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min) {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void Arm::UpdateJointAngles() {
    motorJ[ALL]->UpdateAngle();
}

void Arm::UpdateJointAnglesCallback() {
    for (int i = 1; i <= 6; i++) {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}

void Arm::SetJointSpeed(float _speed) {
    if (_speed < 0)
        _speed = 0;
    else if (_speed > 100)
        _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}

void Arm::SetJointAcceleration(float _acc) {
    if (_acc < 0)
        _acc = 0;
    else if (_acc > 100)
        _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}

#if 0
void Arm::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}
#endif

void Arm::Homing() {
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    //    while (IsMoving())
    //        osDelay(10);

    SetJointSpeed(lastSpeed);
}

#if 0
void Arm::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void Arm::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}

#endif
void Arm::UpdateJointPose6D() {
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000;// m -> mm
    currentPose6D.Y *= 1000;// m -> mm
    currentPose6D.Z *= 1000;// m -> mm
}

bool Arm::IsMoving() {
    return jointsStateFlag != 0b1111110;
}

bool Arm::IsEnabled() {
    return isEnabled;
}

#if 0
void Arm::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}

uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}

void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false;
    ClearFifo();
}

std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}

uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}
#endif

uint32_t Arm::CommandHandler::ParseCommand(const std::string &_cmd) {
    uint8_t argNum;

    switch (context->commandMode) {
    case COMMAND_TARGET_POINT_SEQUENTIAL:
    case COMMAND_CONTINUES_TRAJECTORY:
        if (_cmd[0] == '>' || _cmd[0] == '&') {
            float joints[6];
            float speed;

            if (_cmd[0] == '>')
                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
            if (_cmd[0] == '&')
                argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
            if (argNum == 6) {
                context->MoveJ(joints[0], joints[1], joints[2],
                               joints[3], joints[4], joints[5]);
            } else if (argNum == 7) {
                context->SetJointSpeed(speed);
                context->MoveJ(joints[0], joints[1], joints[2],
                               joints[3], joints[4], joints[5]);
            }
            // Trigger a transmission immediately, in case IsMoving() returns false
            context->MoveJoints(context->targetJoints);

            //                while (context->IsMoving() && context->IsEnabled())
            //                    osDelay(5);
            //                Respond(*usbStreamOutputPtr, "ok");
            //                Respond(*uart4StreamOutputPtr, "ok");
        } else if (_cmd[0] == '@') {
            float pose[6];
            float speed;

            argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                            pose + 3, pose + 4, pose + 5, &speed);
            if (argNum == 6) {
                context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            } else if (argNum == 7) {
                context->SetJointSpeed(speed);
                context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            }
            //                Respond(*usbStreamOutputPtr, "ok");
            //                Respond(*uart4StreamOutputPtr, "ok");
        }

        break;

    case COMMAND_TARGET_POINT_INTERRUPTABLE:
        if (_cmd[0] == '>' || _cmd[0] == '&') {
            float joints[6];
            float speed;

            if (_cmd[0] == '>')
                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
            if (_cmd[0] == '&')
                argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
            if (argNum == 6) {
                context->MoveJ(joints[0], joints[1], joints[2],
                               joints[3], joints[4], joints[5]);
            } else if (argNum == 7) {
                context->SetJointSpeed(speed);
                context->MoveJ(joints[0], joints[1], joints[2],
                               joints[3], joints[4], joints[5]);
            }
            //                Respond(*usbStreamOutputPtr, "ok");
            //                Respond(*uart4StreamOutputPtr, "ok");
        } else if (_cmd[0] == '@') {
            float pose[6];
            float speed;

            argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                            pose + 3, pose + 4, pose + 5, &speed);
            if (argNum == 6) {
                context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            } else if (argNum == 7) {
                context->SetJointSpeed(speed);
                context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            }
            //                Respond(*usbStreamOutputPtr, "ok");
            //                Respond(*uart4StreamOutputPtr, "ok");
        }
        break;

    case COMMAND_MOTOR_TUNING:
        break;
    }
    return 1;// TODO osMessageQueueGetSpace(commandFifo);
}