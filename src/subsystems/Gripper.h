#pragma once
#include "motor/CAN.h"
#include "robot/SubsystemBase.h"
#include <memory>

class Gripper : public SubsystemBase {

public:
    /// The Gripper length.  unit <meter>
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

    Gripper();

    Gripper(const Gripper &) = delete;
    Gripper &operator=(const Gripper &) = delete;

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
    void Reset();

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(float x, float y, long timestamp);

    /**
     * Returns whether the Gripper controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Returns how many times the vision measurement was too far from the
     * Gripper pose estimate.
     */
    int GetPoseMeasurementFaultCounter();

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    //void ControllerPeriodic() override;

private:
    /**
     * Set Gripper motors to brake mode, which the feedback controllers
     * expect.
     */
    void SetBrakeMode();

    /**
     * Set Gripper motors to coast mode so the robot is easier to push when
     * it's disabled.
     */
    void SetCoastMode();

    /**
     * Enables the gripper.
     */
    void Enable();

    /**
     * Disables the gripper.
     */
    void Disable();

    /**
     * Set the angle for both fingers.
     */
    void SetAngle(float left_angle, float right_angle);

    void SetMaxCurrent(float _val);

    /**
     * Set the angle for both fingers.
     */
    void Close(int left_angle, int right_angle);

    bool m_isEnabled;
    std::shared_ptr<CAN> can;
};