package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.types.ClawControl
import org.firstinspires.ftc.teamcode.core.types.ClawMeasurement
import org.firstinspires.ftc.teamcode.core.types.ClawStateEstimate
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.DriveControl
import org.firstinspires.ftc.teamcode.core.types.DriveMeasurement
import org.firstinspires.ftc.teamcode.core.types.DriveStateEstimate
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.teamcode.core.types.StateEstimate

interface StateEstimator {
    /**
     * Update the state estimate based on new measurements.
     */
    fun update(measurement: Measurement)

    /**
     * Predict the future state for the next time step.
     */
    fun predict(control: Control)

    /**
     * Our current best estimate of the state.
     */
    val estimate: StateEstimate
}

data class InitialConditions(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
)

fun stateEstimator(initial: InitialConditions): StateEstimator
        = StateEstimatorImpl(initial)

private class StateEstimatorImpl(initial: InitialConditions): StateEstimator {

    /* current drive state estimate */
    var xMeters = initial.xMeters
    var yMeters = initial.yMeters
    var yawRads = initial.yawRads
    var dXMetersPerCycle = 0.0
    var dYMetersPerCycle = 0.0
    var dYawRadsPerCycle = 0.0
    var leftWheelMeters = 0.0
    var rightWheelMeters = 0.0
    var centerWheelMeters = 0.0

    /* Current estimate of the claw height, relative to home. */
    var clawHeightMeters = 0.0
    /* Current estimate of the home position, relative to encoder 0. */
    var clawHomeMeters = 0.0

    /* estimate as viewed from outside */
    override val estimate: StateEstimate
        get() = StateEstimate(
            drive = DriveStateEstimate(
                xMeters = xMeters,
                yMeters = yMeters,
                yawRads = yawRads,
                dXMetersPerCycle = dXMetersPerCycle,
                dYMetersPerCycle = dYMetersPerCycle,
                dYawRadsPerCycle = dYawRadsPerCycle,
            ),
            claw = ClawStateEstimate(
                heightMeters = clawHeightMeters
            )
        )

    fun updateClaw(measurement: ClawMeasurement) {
        if (measurement.isHome) {
            clawHomeMeters = measurement.heightMeters
        }
        clawHeightMeters = measurement.heightMeters - clawHomeMeters
    }

    fun predictClaw(control: ClawControl) {
        /* No-op for now. */
    }

    fun updateDrive(measurement: DriveMeasurement) {
        TODO("not implemented")
        /*
         * Predict the value of each measurement.  Our model is that the robot
         * moves with constant velocity, and any deviation from that is noise.
         *
         * We're working in the group SE(2):
         * [ R R X ]
         * [ R R Y ]
         * [ 0 0 1 ]
         *
         * This acts on R^3 by multiplication.
         *
         * Translation by a fixed velocity is
         *
         * [ 1 0 (vx * t) ] [ x ]   [ x + vx * t ]
         * [ 0 1 (vy * t) ] [ y ] = [ y + vy * t ]
         * [ 0 0 1        ] [ 1 ]   [ 1          ]
         *
         * At 0 this has derivative
         *
         * [ 0 0 vx ]
         * [ 0 0 vy ]
         * [ 0 0 0  ]
         *
         * If I square it, I get 0.  Thus
         *
         * e^(At) = 1 + A * t = [ 1 0 vx * t ]
         *                      [ 0 1 vy * t ]
         *                      [ 0 0 1      ]
         *
         * Rotation by a fixed velocity is
         *
         * [ cos(omega*t) -sin(omega*t) 0 ]
         * [ sin(omega*t)  cos(omega*t) 0 ]
         * [ 0             0            1 ]
         *
         * At 0 this has derivative
         *
         *     [ 0 -1 0 ]
         * B = [ 1  0 0 ]
         *     [ 0  0 0 ]
         *
         *       [ -1 0 0 ]
         * B^2 = [ 0 -1 0 ] = -P
         *       [ 0  0 0 ]
         *
         *
         * This behaves a lot like sqrt(-1):
         *
         * e^(Bt) = 1 + (Bt) + (Bt)^2/2 + ... + (Bt)^i/i!
         *        = 1 + Bt - P t^2/2 + -PB t^3/6 + ...
         *        = (1 - P) + B * P * sum_i [(-1)^i*t^(2i+1)/(2i+1)!]
         *                  +     P   sum_i [(-1)^i*t(2i)/(2i)!]
         *        = (1 - P) + (B * sin(t) + P * cos(t))
         *          [ cos(t) -sin(t) 0 ]
         *          [ sin(t) cos(t)  0 ]
         *          [ 0      0       1 ]
         *
         * X = [0 0 1]
         *     [0 0 0]
         *     [0 0 0]
         *
         * Y = [0 0 0]
         *     [0 0 1]
         *     [0 0 0]
         *
         * R = [0 -1 0]
         *     [1  0 0]
         *     [0  0 0]
         * [X,Y] = 0
         * [R,X] = Y
         * [R,Y] = -X
         *
         * How does this affect the encoders?
         * And how do we design a state estimator?
         *
         * Our current pose is an element of SE(2), translating from robot to
         * field coords.
         * Our current velocity is vx, vy, omega.
         * Our current encoder readings are e1, e2, e3.
         *
         * Driving forwards changes the pose by g -> g * e^(Yt)
         * Driving right changes the pose by    g -> g * e^(Xt)
         * Driving CCW changes the pose by      g -> g * e^(Rt)
         * Driving in the global X direction:   g -> e^(Xt) * g
         * Driving in the global Y direction:   g -> e^(Yt) * g
         *
         * We want to represent the pose as a composition
         *  g = e^(Xa)e^(Yb)e^(Rc)
         *
         * X and Y commute, so we can equivalently say:
         *  g = e^(Xa+Yb)*e^(Rc)
         *
         * Our model is that on each timestep, the velocity changes to an
         * unknown constant (in field-relative coords), with that constant
         * having some uncertainty.  We'll assume each component of velocity is
         * independent for now.
         *
         * The nominal new pose is then
         * g -> e^((X*vx + Y*vy)*t) * g * e^(R*omega*t)
         *
         * How do I phrase this in terms of diffeqs?
         * g' should be a vector in the tangent space of SE(2), i.e. an element
         * of the lie algebra.  But are we multiplying on the left or the right?
         * Maybe multiply everything on the right, so we are working robot-
         * relative.  In that case, the (robot-relative) velocity also changes
         * as we rotate.
         *
         * rvx(t) = rvx(0) * cos(omega*t) - rvy(0) * sin(omega*t)
         * rvy(t) = rvx(0) * sin(omega*t) + rvy(0) * cos(omega*t)
         *
         * so
         *
         * rvx'    = -omega * rvy
         * rvy'    = omega * rvx
         * omega'  = 0
         * pose'   = X * rvx + Y * rvy + R * omega
         * enc[i]' = (linear function of rvx, rvy, omega)
         *
         * So the time-update equations (euler integration) are
         *
         * velocity on the interval (k, k+1):
         *
         * rvx[k]   =              rvx[k-1] - omega[k-1] * rvy[k-1] + noise
         * rvy[k]   = omega[k-1] * rvx[k-1] +              rvy[k-1] + noise
         * omega[k] = omega[k-1]                                    + noise
         *
         * position at time k:
         *
         * pose[k]  = pose[k-1] * exp(X*rvx[k-1]+Y*rvy[k-1]+R*omega[k-1])
         * enc[i,k] = enc[i,k-1] + (zsomething with rvx[k-1],rvy[k-1],omega[k-1])
         *
         * That matrix exponential is:
         *     [0     -omega rvx]
         * exp [omega  0     rvy]
         *     [0      0     0  ]
         *
         * so
         *          [cos(omega*t) | -sin(omega*t) | vx*sin(omega*t)/omega + vy*(cos(omega*t)-1)/omega]
         * e^(tA) = [sin(omega*t) | cos(omega*t)  | vx*(1-cos(omega*t))/omega + vy*sin(omega*t)/omega]
         *          [0            | 0             | 1                                                ]
         *
         * This matches the formulas online!
         *
         * Let ux, uy be the displacement values in the above matrix.
         *
         * x[k] = x[k-1] + cos(theta[k-1]) * ux[k-1] - sin(theta[k-1]) * uy[k-1]
         * y[k] = y[k-1] + sin(theta[k-1]) * ux[k-1] + cos(theta[k-1]) * uy[k-1]
         * theta[k] = theta[k-1] + omega[k-1]
         *
         * What's left? encoders and *covariance matrix*.
         * Assume encoder measurements have 0 variance, all uncertainty is from process.
         *
         * So we find an error for enc[i,k].  We pull this back into an error for
         * rvx[k-1],rvy[k-1],omega[k-1], and then push that forwards to an error
         * for (x[k],y[k],theta[k],rvx[k],rvy[k],omega[k]).  Then we correct the
         * state according to that error.
         *
         * What is the Kalman gain matrix?
         */


        xMeters += dXMetersPerCycle
        yMeters += dYMetersPerCycle
        yawRads += dYawRadsPerCycle


        /* Compare predictions with actual values to compute error. */
        /* Use the errors to correct our estimate. */
    }

    fun predictDrive(control: DriveControl) {
        /*
         * For now, we don't take the control signal into account at all, and
         * instead we just assume that the robot will always move with constant
         * velocity (as if the motors were set to coast and there was no
         * friction).  We'll replace this with a better model after we gather
         * some data.
         */
        xMeters += dXMetersPerCycle
        yMeters += dYMetersPerCycle
        yawRads += dYawRadsPerCycle
    }

    override fun update(measurement: Measurement) {
        updateClaw(measurement.claw)
        updateDrive(measurement.drive)
    }

    override fun predict(control: Control) {
        predictClaw(control.claw)
        predictDrive(control.drive)
    }
}