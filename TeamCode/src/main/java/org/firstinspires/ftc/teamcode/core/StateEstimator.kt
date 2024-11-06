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
private const val S_LIFT_X = 9
private const val S_LIFT_V = 10
private const val S_LIFT_ENC1 = 11
private const val S_LIFT_ENC2 = 12

/*
 * When we start the robot up, we need to know what these values are.
 */
data class InitialConditions(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
)

/* This function returns a new state estimator based on initial conditions. */
fun stateEstimator(initial: InitialConditions) = object : StateEstimator {
    val estimator = ekf {
        /* 5cm standard deviation in initial placement */
        init[S_X] = initial.xMeters + 0.05 * noise()
        init[S_Y] = initial.yMeters + 0.05 * noise()
        /* 10 degree standard deviation in initial heading */
        init[S_YAW] = initial.yawRads + 10.0 / 180.0 * PI * noise()
        /* For everything else, it's "don't know". */
        init[S_VLAT] = noise()
        init[S_VLONG] = noise()
        init[S_VROT] = noise()
        init[S_ENC1] = noise()
        init[S_ENC2] = noise()
        init[S_ENC3] = noise()
        init[S_LIFT_X] = noise()
        init[S_LIFT_V] = noise()
        init[S_LIFT_ENC1] = noise()
        init[S_LIFT_ENC2] = noise()
    }

    override fun measure(measurement: Measurement) = estimator.measure {
        /*
         * Measure drive encoders.  We measure the state variable almost
         * directly.  The difference comes from quantization noise, which I
         * roughly estimate is on the order of 10^-4 meters.
         */
        val q1 = 1e-4 * noise()
        val q2 = 1e-4 * noise()
        val q3 = 1e-4 * noise()
        val encoders = measurement.drive.encoders
        measured[state[S_ENC1] + q1] = encoders.leftMeters
        measured[state[S_ENC2] + q2] = encoders.rightMeters
        measured[state[S_ENC3] + q3] = encoders.centerMeters

        /*
         * Measure the lift position.  The same comments about quantization
         * noise apply here.
         */
        val q4 = 1e-4 * noise()
        val q5 = 1e-4 * noise()
        measured[state[S_LIFT_ENC1] + q4] = measurement.claw.heightMeters
        measured[state[S_LIFT_ENC2] + q5] = measurement.claw.heightMeters

        /*
         * If the lift home switch is pressed, we know we are within ~1mm of the
         * home position.
         */
        if (measurement.claw.isHome) {
            val q6 = 1e-3 * noise()
            measured[state[S_LIFT_X] + q6] = 0.0
        }
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
        val aLat = 0.0 + 0.0 * noise()
        val aLong = 0.0 + 0.0 * noise()
        val aRot = 0.0 + 0.0 * noise()
        new[S_VLAT] = old[S_VLAT] - old[S_VROT] * old[S_VLONG] + aLat
        new[S_VLONG] = old[S_VLONG] + old[S_VROT] * old[S_VLAT] + aLong
        new[S_VROT] = old[S_VROT] + aRot

        /* Integrate the drive encoder counts. */
        val slip1 = 0.0 * noise()
        val slip2 = 0.0 * noise()
        val slip3 = 0.0 * noise()
        val r1 = 0.0 // TODO these need to be real values
        val r2 = 0.0
        val r3 = 0.0
        new[S_ENC1] = old[S_ENC1] + old[S_VLONG] + r1 * old[S_VROT] + slip1
        new[S_ENC2] = old[S_ENC2] + old[S_VLONG] + r2 * old[S_VROT] + slip2
        new[S_ENC3] = old[S_ENC3] + old[S_VLAT] + r3 * old[S_VROT] + slip3

        /* Integrate claw velocity to find position. */
        new[S_LIFT_X] = old[S_LIFT_X] + old[S_LIFT_V]

        /* Integrate forces on lifter. */
        val aLift = 0.0 + 0.0 * noise()
        new[S_LIFT_V] = old[S_LIFT_V] + aLift

        /* Integrate lifter encoder counts. */
        val slip4 = 0.0 * noise()
        val slip5 = 0.0 * noise()
        new[S_LIFT_ENC1] = old[S_LIFT_ENC1] + old[S_LIFT_V] + slip4
        new[S_LIFT_ENC2] = old[S_LIFT_ENC2] + old[S_LIFT_V] + slip5
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
                heightMeters = mean[S_LIFT_X],
            )
        )
    }
}
