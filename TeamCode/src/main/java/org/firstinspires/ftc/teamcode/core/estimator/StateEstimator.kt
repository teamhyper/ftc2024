package org.firstinspires.ftc.teamcode.core.estimator

import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.teamcode.core.types.StateEstimate

/*
 * This file implements a state estimator.  Its job is to maintain an estimate
 * of the current state (position, velocity, heading, position of attachments,
 * etc.) of the robot.  There are two ways an update can be requested:
 *
 * 1. New data from sensors is available.  We compare the measured value with
 *    the expected value, and make a correction to our estimate.
 *
 * 2. The control cycle is ending and we are about to sleep.  We update the
 *    state estimate to the predicted state of the robot after the sleep.
 */

interface StateEstimator {
    /**
     * Update the state estimate based on new measurements.
     */
    fun measure(measurement: Measurement)

    /**
     * Predict the future state for the next time step.
     */
    fun predict(control: Control)

    /**
     * Our current best estimate of the state.
     */
    val estimate: StateEstimate
}

/* This function returns a new state estimator based on initial conditions. */
fun stateEstimator(
    arm: ArmEstimator = armEstimator(),
    lift: LiftEstimator = liftEstimator(),
    drive: DriveEstimator = driveEstimator(),
) = object : StateEstimator {
    override fun measure(m: Measurement) {
        arm.measure(encoderTicks = m.armEncTicks)
        lift.measure(
            leftEncoderTicks = m.leftLiftEncTicks,
            rightEncoderTicks = m.rightLiftEncTicks,
            isHome = m.liftIsHome
        )
        drive.measure(DriveMeasurement(
            leftEncoderTicks = m.leftDriveEncTicks,
            rightEncoderTicks = m.rightDriveEncTicks,
            centerEncoderTicks = m.centerDriveEncTicks,
        ))
    }

    override fun predict(control: Control) {
        drive.predict(DriveControl(
            frontLeftPower = control.frontLeftDrivePower,
            frontRightPower = control.frontRightDrivePower,
            backLeftPower = control.backLeftDrivePower,
            backRightPower = control.backRightDrivePower,
        ))
    }

    override val estimate: StateEstimate
        get() = StateEstimate(
            xMeters = drive.state.xMeters,
            yMeters = drive.state.yMeters,
            yawRads = drive.state.yawRads,
            latMetersPerCycle = drive.state.latMetersPerCycle,
            longMetersPerCycle = drive.state.longMetersPerCycle,
            rotRadsPerCycle = drive.state.yawRadsPerCycle,
            liftState = lift.state,
            armAngleRads = arm.angleRads,
        )

}
