#pragma once

#include <limits>
#include "Pose2d.h"

/**
 * An interface for defining user-defined velocity and acceleration constraints
 * while generating trajectories.
 */
class  TrajectoryConstraint {
    public:
           TrajectoryConstraint(float kinematics,float maxSpeed);
           TrajectoryConstraint(const TrajectoryConstraint&);
           TrajectoryConstraint& operator=(const TrajectoryConstraint&);

           TrajectoryConstraint(TrajectoryConstraint&&);
           TrajectoryConstraint& operator=(TrajectoryConstraint&&);

           ~TrajectoryConstraint();

            /**
             * Represents a minimum and maximum acceleration.
             */
            struct MinMax {
                /**
                 * The minimum acceleration.
                 */
                double minAcceleration{ -std::numeric_limits<double>::max()};

                /**
                 * The maximum acceleration.
                 */
                double maxAcceleration{ std::numeric_limits<double>::max()};
            };

            /**
             * Returns the max velocity given the current pose and curvature.
             *
             * @param pose The pose at the current point in the trajectory.
             * @param curvature The curvature at the current point in the trajectory.
             * @param velocity The velocity at the current point in the trajectory before
             *                                constraints are applied.
             *
             * @return The absolute maximum velocity.
             */
    float MaxVelocity(
            const Pose2d& pose, //units::curvature_t curvature,
            float velocity);

            /**
             * Returns the minimum and maximum allowable acceleration for the trajectory
             * given pose, curvature, and speed.
             *
             * @param pose The pose at the current point in the trajectory.
             * @param curvature The curvature at the current point in the trajectory.
             * @param speed The speed at the current point in the trajectory.
             *
             * @return The min and max acceleration bounds.
             */
   MinMax MinMaxAcceleration(const Pose2d& pose,
           // units::curvature_t curvature,
           float speed);
private:
    //DifferentialDriveKinematics m_kinematics;
    float m_maxSpeed;
};

