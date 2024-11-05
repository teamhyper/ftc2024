package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.math.ekf
import org.firstinspires.ftc.teamcode.core.math.*
import org.firstinspires.ftc.teamcode.core.types.ClawStateEstimate
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.DriveStateEstimate
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import kotlin.math.PI

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

/*
 * Now we build a model of the robot's dynamics.  Each constant names a state
 * variable.  Its value is an index into a vector/matrix.
 */
private const val S_X = 0
private const val S_Y = 1
private const val S_YAW = 2
private const val S_VLAT = 3
private const val S_VLONG = 4
private const val S_VROT = 5
private const val S_ENC1 = 6
private const val S_ENC2 = 7
private const val S_ENC3 = 8

/*
 * When we start the robot up, we need to know what these values are.
 */
data class InitialConditions(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
    val enc1Meters: Double,
    val enc2Meters: Double,
    val enc3Meters: Double,
)

/* This function returns a new state estimator based on initial conditions. */
fun stateEstimator(initial: InitialConditions) = object : StateEstimator {
    val estimator = ekf {
        /* 5cm standard deviation in initial placement */
        init[S_X] = initial.xMeters + 0.05 * noise()
        init[S_Y] = initial.yMeters + 0.05 * noise()
        /* 10 degree standard deviation in initial heading */
        init[S_YAW] = initial.yawRads + 10.0 / 180.0 * PI * noise()
        /* Assume we're stationary when we start */
        init[S_VLAT] = 0.0
        init[S_VLONG] = 0.0
        init[S_VROT] = 0.0
        /* The encoders may not be zero at the start */
        init[S_ENC1] = initial.enc1Meters
        init[S_ENC2] = initial.enc2Meters
        init[S_ENC3] = initial.enc3Meters
    }

    override fun measure(measurement: Measurement) = estimator.measure {
        measured[state[S_ENC1]] = measurement.drive.encoders.leftMeters
        measured[state[S_ENC2]] = measurement.drive.encoders.rightMeters
        measured[state[S_ENC3]] = measurement.drive.encoders.centerMeters
    }

    override fun predict(control: Control) = estimator.predict {
        /*
         * Integrate the current velocity estimate to get the new position
         * estimate. See [1], section 10.2, for an explanation.
         *
         * [1]: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
         */
        val a = 1.0 - old[S_VROT] * old[S_VROT] / 6.0
        val b = old[S_VROT] / 2.0
        val u = a * old[S_VLAT] - b * old[S_VLONG]
        val v = b * old[S_VLAT] + a * old[S_VLONG]
        new[S_X] = old[S_X] + cos(old[S_YAW]) * u - sin(old[S_YAW]) * v
        new[S_Y] = old[S_Y] + sin(old[S_YAW]) * u + cos(old[S_YAW]) * v
        new[S_YAW] = old[S_YAW] + old[S_VROT]

        /*
         * Integrate the forces on the robot to predict the new velocity.  Also
         * take account for the rotating reference frame.
         */
        val aLat = 0.0
        val aLong = 0.0
        val aRot = 0.0
        val qLat = 0.0
        val qLong = 0.0
        val qRot = 0.0
        new[S_VLAT] = old[S_VLAT] - old[S_VROT] * old[S_VLONG] + aLat + qLat
        new[S_VLONG] = old[S_VLONG] + old[S_VROT] * old[S_VLAT] + aLong + qLong
        new[S_VROT] = old[S_VROT] + aRot + qRot

        /*
         * Integrate the encoder counts.  This computes the expected value of
         * the encoders on the next time step.
         */
        val q1 = 0.0
        val q2 = 0.0
        val q3 = 0.0
        val r1 = 0.0 // TODO these need to be real values
        val r2 = 0.0
        val r3 = 0.0
        new[S_ENC1] = old[S_ENC1] + old[S_VLONG] + r1 * old[S_VROT] + q1
        new[S_ENC2] = old[S_ENC2] + old[S_VLONG] + r2 * old[S_VROT] + q2
        new[S_ENC3] = old[S_ENC3] + old[S_VLAT] + r3 * old[S_VROT] + q3
    }

    override val estimate get() = with(estimator.estimate) {
        StateEstimate(
            drive = DriveStateEstimate(
                xMeters = mean[S_X],
                yMeters = mean[S_Y],
                yawRads = mean[S_YAW],
                latMetersPerCycle = mean[S_VLAT],
                longMetersPerCycle = mean[S_VLONG],
                rotRadsPerCycle = mean[S_VROT],
            ),
            claw = ClawStateEstimate(
                heightMeters = 0.0 // TODO implement this for real
            )
        )
    }
}
