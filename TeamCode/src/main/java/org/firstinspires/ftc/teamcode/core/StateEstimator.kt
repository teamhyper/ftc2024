package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.math.ekf
import org.firstinspires.ftc.teamcode.core.math.*
import org.firstinspires.ftc.teamcode.core.types.Control
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
 * variable.  Its value is an index into a vector/matrix.  We start with the
 * position and velocity of the robot and its mechanisms.
 */

private enum class S {
    X,
    Y,
    YAW,
    LIFT_HEIGHT,
    ARM_ANGLE,

    LAT_VEL,
    LONG_VEL,
    ROT_VEL,
    LIFT_VEL,
    ARM_VEL,

    ENC_DRIVE_LEFT,
    ENC_DRIVE_RIGHT,
    ENC_DRIVE_CENTER,
    ENC_LIFT_LEFT,
    ENC_LIFT_RIGHT,
    ENC_ARM,
}

private operator fun GetVar.get(s: S) = get(s.ordinal)
private operator fun SetVar.set(s: S, e: EKFExpr) = set(s.ordinal, e)
private operator fun Mean.get(s: S) = get(s.ordinal)

/*
 * When we start the robot up, we need to know what these values are.
 */
data class InitialConditions(
    val xMeters: Double = 0.0,
    val yMeters: Double = 0.0,
    val yawRads: Double = 0.0,
    val armRads: Double = 0.0,
)

/* This function returns a new state estimator based on initial conditions. */
fun stateEstimator(
    initial: InitialConditions = InitialConditions()
) = object : StateEstimator {
    val estimator = ekf {
        /* 10cm standard deviation in initial placement */
        init[S.X] = initial.xMeters + 0.1 * noise()
        init[S.Y] = initial.yMeters + 0.1 * noise()
        /* 10 degree standard deviation in initial heading */
        init[S.YAW] = initial.yawRads + 10.0 / 180.0 * PI * noise()
        /* Initial height is unknown until we press the homing switch. */
        init[S.LIFT_HEIGHT] = 0.5 + 1.0 * noise()
        /* Arm placement is specified known to within 10 degrees. */
        init[S.ARM_ANGLE] = initial.armRads + 10.0 / 180.0 * PI * noise()

        /* drive train moves initially at <0.01 m/s, <0.01 rad/s */
        init[S.LAT_VEL] = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        init[S.LONG_VEL] = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        init[S.ROT_VEL] = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        init[S.LIFT_VEL] = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        init[S.ARM_VEL] = 0.01 * CYCLE_PERIOD_SECONDS * noise()

        /*
         * Initially, we don't know what the count is, so just make the variance
         * really big.  After a few
         * A billion should be big enough, since encoder ticks are measured in
         * 32-bit ints anyway.
         */
        init[S.ENC_DRIVE_LEFT] = 1e9 * noise()
        init[S.ENC_DRIVE_RIGHT] = 1e9 * noise()
        init[S.ENC_DRIVE_CENTER] = 1e9 * noise()
        init[S.ENC_LIFT_LEFT] = 1e9 * noise()
        init[S.ENC_LIFT_RIGHT] = 1e9 * noise()
        init[S.ENC_ARM] = 1e9 * noise()
    }

    override fun measure(measurement: Measurement) = estimator.measure {
        /* Measure encoders + quantization noise on the order of 1 tick. */
        measured[state[S.ENC_DRIVE_LEFT] + noise()] = measurement.leftDriveEncTicks
        measured[state[S.ENC_DRIVE_RIGHT] + noise()] = measurement.rightDriveEncTicks
        measured[state[S.ENC_DRIVE_CENTER] + noise()] = measurement.centerDriveEncTicks
        measured[state[S.ENC_LIFT_LEFT] + noise()] = measurement.leftLiftEncTicks
        measured[state[S.ENC_LIFT_RIGHT] + noise()] = measurement.rightLiftEncTicks
        measured[state[S.ENC_ARM] + noise()] = measurement.armEncTicks

        if (measurement.liftIsHome) {
            /* We are within ~1mm of the home position. */
            measured[state[S.LIFT_HEIGHT] + 0.001 * noise()] = 0.0
        }
    }

    override fun predict(control: Control) = estimator.predict {
        /*
         * Integrate the current velocity estimate to get the new position
         * estimate. See [1], section 10.2, for an explanation.
         *
         * [1]: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
         */
        val a = 1.0 - old[S.ROT_VEL] * old[S.ROT_VEL] / 6.0
        val b = old[S.ROT_VEL] / 2.0
        val u = a * old[S.LAT_VEL] - b * old[S.LONG_VEL]
        val v = b * old[S.LAT_VEL] + a * old[S.LONG_VEL]
        new[S.X] = old[S.X] + cos(old[S.YAW]) * u - sin(old[S.YAW]) * v
        new[S.Y] = old[S.Y] + sin(old[S.YAW]) * u + cos(old[S.YAW]) * v
        new[S.YAW] = old[S.YAW] + old[S.ROT_VEL]
        new[S.LIFT_HEIGHT] = old[S.LIFT_HEIGHT] + old[S.LIFT_VEL]
        new[S.ARM_ANGLE] = old[S.ARM_ANGLE] + old[S.ARM_VEL]

        /*
         * Integrate the forces on the robot to predict the new velocity.  Also
         * take account for the rotating reference frame.  For now we have only
         * a very rough model of this.
         */
        val aLat = 10.0 * CYCLE_PERIOD_SECONDS * noise()
        val aLong = 10.0 * CYCLE_PERIOD_SECONDS * noise()
        val aRot = 10.0 * CYCLE_PERIOD_SECONDS * noise()
        val aLift = 10.0 * CYCLE_PERIOD_SECONDS * noise()
        val aArm = 10.0 * CYCLE_PERIOD_SECONDS * noise()
        new[S.LAT_VEL] = old[S.LAT_VEL] - old[S.ROT_VEL] * old[S.LONG_VEL] + aLat
        new[S.LONG_VEL] = old[S.LONG_VEL] + old[S.ROT_VEL] * old[S.LAT_VEL] + aLong
        new[S.ROT_VEL] = old[S.ROT_VEL] + aRot
        new[S.LIFT_VEL] = old[S.LIFT_VEL] + aLift
        new[S.ARM_VEL] = old[S.ARM_VEL] + aArm

        /* Calculate displacements measured by each encoder. */
        val leftDriveMeters = old[S.LONG_VEL] - DRIVE_ENC_LEFT_POS * old[S.ROT_VEL]
        val rightDriveMeters = old[S.LONG_VEL] + DRIVE_ENC_RIGHT_POS * old[S.ROT_VEL]
        val centerDriveMeters = old[S.LONG_VEL] + DRIVE_ENC_CENTER_POS * old[S.ROT_VEL]
        val liftMeters = old[S.LIFT_VEL]
        val armRads = old[S.ARM_VEL]

        /* Convert encoder displacements to ticks. */
        val leftDriveTicks = DRIVE_METERS_PER_TICK * leftDriveMeters
        val rightDriveTicks = DRIVE_METERS_PER_TICK * rightDriveMeters
        val centerDriveTicks = DRIVE_METERS_PER_TICK * centerDriveMeters
        val liftTicks = LIFT_METERS_PER_TICK * liftMeters
        val armTicks = ARM_RADS_PER_TICK * armRads

        /* Add noise terms for occasional skipping/slipping, ~1 tick/100 sec. */
        val leftDriveSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        val rightDriveSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        val centerDriveSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        val leftLiftSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        val rightLiftSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
        val armSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()

        new[S.ENC_DRIVE_LEFT] = old[S.ENC_DRIVE_LEFT] + leftDriveTicks + leftDriveSlip
        new[S.ENC_DRIVE_RIGHT] = old[S.ENC_DRIVE_RIGHT] + rightDriveTicks + rightDriveSlip
        new[S.ENC_DRIVE_CENTER] = old[S.ENC_DRIVE_CENTER] + centerDriveTicks + centerDriveSlip
        new[S.ENC_LIFT_LEFT] = old[S.ENC_LIFT_LEFT] + liftTicks + leftLiftSlip
        new[S.ENC_LIFT_RIGHT] = old[S.ENC_LIFT_RIGHT] + liftTicks + rightLiftSlip
        new[S.ENC_ARM] = old[S.ENC_ARM] + armTicks + armSlip
    }

    override val estimate get() = estimator.estimate.mean.let {
        StateEstimate(
            xMeters = it[S.X],
            yMeters = it[S.Y],
            yawRads = it[S.YAW],
            latMetersPerCycle = it[S.LAT_VEL],
            longMetersPerCycle = it[S.LONG_VEL],
            rotRadsPerCycle = it[S.ROT_VEL],
            liftHeightMeters = it[S.LIFT_HEIGHT],
            armAngleRads = it[S.ARM_ANGLE],
        )
    }
}
