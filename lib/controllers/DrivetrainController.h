#pragma once

#include <functional>
#include <tuple>
#include <vector>
#include "subsystem/Pose2d.h"
#include "subsystem/TrajectoryConfig.h"
#include "ControllerBase.h"

/**
 * The drivetrain controller.
 *
 * The drivetrain uses a linear time-varying LQR for feedback control. Since the
 * model is control-affine (the dynamics are nonlinear, but the control inputs
 * provide a linear contribution), a plant inversion feedforward was used.
 * Trajectories generated from splines provide the motion profile to follow.
 *
 * The linear time-varying controller has a similar form to the LQR, but the
 * model used to compute the controller gain is the nonlinear model linearized
 * around the drivetrain's current state. We precomputed gains for important
 * places in our state-space, then interpolated between them with a LUT to save
 * computational resources.
 *
 * We decided to control for longitudinal error and cross-track error in the
 * chassis frame instead of x and y error in the global frame, so the state
 * Jacobian simplified such that we only had to sweep velocities (-4m/s to
 * 4m/s).
 *
 * See section 9.6 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 9.6.3.
 */
class DrivetrainController : public ControllerBase<7, 2, 4> {
public:
    /**
     * Constructs a drivetrain controller.
     */
    DrivetrainController();

    /**
     * Move constructor.
     */
    DrivetrainController(DrivetrainController&&) = default;

    /**
     * Move assignment operator.
     */
    DrivetrainController& operator=(DrivetrainController&&) = default;

    /**
     * Sets the current estimated global pose of the drivetrain.
     */
    void SetDrivetrainStates(const Eigen::Vector<double, 7>& x);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void Reset(const Pose2d& initialPose);

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    Eigen::Vector<double, 2> Calculate( const Eigen::Vector<double, 7>& x) override;

    /**
     * The drivetrain system dynamics.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 7> Dynamics(const Eigen::Vector<double, 7>& x,
                                             const Eigen::Vector<double, 2>& u);

    /**
     * Returns the global measurements that correspond to the given state and
     * input vectors.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 2> GlobalMeasurementModel(
            const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u);

private:
    static constexpr auto kPositionTolerance = 0.25;
    static constexpr auto kVelocityTolerance = 2;
    static constexpr auto kAngleTolerance = 0.52;

    Pose2d m_goal;

    float  m_visionYaw = 0;    //rad
    float  m_visionPitch = 0;  //rad
    float  m_visionRange = 0;  //meter
    long  m_timestamp = 0;     //sec

    Pose2d m_drivetrainNextPoseInGlobal;
    float m_drivetrainLeftVelocity = 0.0;
    float m_drivetrainRightVelocity = 0.0;
};