package org.firstinspires.ftc.teamcode.core.estimator

import org.firstinspires.ftc.teamcode.core.CYCLE_PERIOD_SECONDS
import org.firstinspires.ftc.teamcode.core.DRIVE_ENC_CENTER_POS_TICKS
import org.firstinspires.ftc.teamcode.core.DRIVE_ENC_LEFT_POS_TICKS
import org.firstinspires.ftc.teamcode.core.DRIVE_ENC_RIGHT_POS_TICKS
import org.firstinspires.ftc.teamcode.core.DRIVE_METERS_PER_TICK
import org.firstinspires.ftc.teamcode.core.DRIVE_YAW_RADS_PER_TICK
import org.firstinspires.ftc.teamcode.core.math.*
import org.firstinspires.ftc.teamcode.core.math.plus

data class DriveState(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
    val latMetersPerCycle: Double,
    val longMetersPerCycle: Double,
    val yawRadsPerCycle: Double,
)

data class DriveMeasurement(
    val leftEncoderTicks: Double,
    val rightEncoderTicks: Double,
    val centerEncoderTicks: Double,
)

data class DriveControl(
    val frontLeftPower: Double,
    val frontRightPower: Double,
    val backLeftPower: Double,
    val backRightPower: Double,
)

interface DriveEstimator {
    val state: DriveState
    fun measure(measurement: DriveMeasurement)
    fun predict(control: DriveControl)
}

sealed class DriveStateInternal {
    object Init: DriveStateInternal()
    class Running(val estimator: EKF): DriveStateInternal()
}

private enum class S {
    X,
    Y,
    YAW,
    LAT_VEL,
    LONG_VEL,
    ROT_VEL,
    ENC_LEFT,
    ENC_RIGHT,
    ENC_CENTER,
}

private operator fun GetVar.get(s: S) = get(s.ordinal)
private operator fun SetVar.set(s: S, e: EKFExpr) = set(s.ordinal, e)
private operator fun Mean.get(s: S) = get(s.ordinal)

fun driveEstimator(
    initialXMeters: Double = 0.0,
    initialYMeters: Double = 0.0,
    initialYawRads: Double = 0.0,
) = object : DriveEstimator {
    var stateInternal: DriveStateInternal = DriveStateInternal.Init

    override val state: DriveState
        get() = stateInternal.let { when (it) {
            is DriveStateInternal.Init -> DriveState(
                xMeters = initialXMeters,
                yMeters = initialYMeters,
                yawRads = initialYawRads,
                latMetersPerCycle = 0.0,
                longMetersPerCycle = 0.0,
                yawRadsPerCycle = 0.0,
            )
            is DriveStateInternal.Running -> DriveState(
                xMeters = it.estimator.estimate.mean[S.X] * DRIVE_METERS_PER_TICK,
                yMeters = it.estimator.estimate.mean[S.Y] * DRIVE_METERS_PER_TICK,
                yawRads = it.estimator.estimate.mean[S.YAW],
                latMetersPerCycle = it.estimator.estimate.mean[S.LAT_VEL] * DRIVE_METERS_PER_TICK,
                longMetersPerCycle = it.estimator.estimate.mean[S.LONG_VEL] * DRIVE_METERS_PER_TICK,
                yawRadsPerCycle = it.estimator.estimate.mean[S.ROT_VEL],
            )
        } }

    override fun measure(measurement: DriveMeasurement) {
        stateInternal.let { when (it) {
            is DriveStateInternal.Init -> {
                /* This is the first measurement.  This is when we initialize
                 * the EKF, based on the initial encoder counts.
                 */
                stateInternal = DriveStateInternal.Running(
                    estimator = ekf {
                        init[S.X] = initialXMeters / DRIVE_METERS_PER_TICK + noise()
                        init[S.Y] = initialYMeters / DRIVE_METERS_PER_TICK + noise()
                        init[S.YAW] = initialYawRads + noise()
                        init[S.LAT_VEL] = noise()
                        init[S.LONG_VEL] = noise()
                        init[S.ROT_VEL] = noise()
                        init[S.ENC_LEFT] = measurement.leftEncoderTicks.toDouble() + noise()
                        init[S.ENC_RIGHT] = measurement.rightEncoderTicks.toDouble() + noise()
                        init[S.ENC_CENTER] = measurement.centerEncoderTicks.toDouble() + noise()
                    }
                )
            }
            is DriveStateInternal.Running -> {
                it.estimator.measure {
                    measured[state[S.ENC_LEFT] + noise()] = measurement.leftEncoderTicks
                    measured[state[S.ENC_RIGHT] + noise()] = measurement.rightEncoderTicks
                    measured[state[S.ENC_CENTER] + noise()] = measurement.centerEncoderTicks
                }
            }
        } }
    }

    override fun predict(control: DriveControl) {
        stateInternal.let { if (it is DriveStateInternal.Running) {
            it.estimator.predict {
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

                /*
                 * Integrate the forces on the robot to predict the new velocity.  Also
                 * take account for the rotating reference frame.  For now we have only
                 * a very rough model of this.
                 */
                val aLat = 10.0 * CYCLE_PERIOD_SECONDS / DRIVE_METERS_PER_TICK * noise()
                val aLong = 10.0 * CYCLE_PERIOD_SECONDS / DRIVE_METERS_PER_TICK * noise()
                val aRot = 10.0 * CYCLE_PERIOD_SECONDS * noise()
                new[S.LAT_VEL] = old[S.LAT_VEL] - old[S.ROT_VEL] * old[S.LONG_VEL] + aLat
                new[S.LONG_VEL] = old[S.LONG_VEL] + old[S.ROT_VEL] * old[S.LAT_VEL] + aLong
                new[S.ROT_VEL] = old[S.ROT_VEL] + aRot

                /* Calculate displacements measured by each encoder. */
                val leftDriveTicks = old[S.LONG_VEL] - DRIVE_ENC_LEFT_POS_TICKS * old[S.ROT_VEL]
                val rightDriveTicks = old[S.LONG_VEL] + DRIVE_ENC_RIGHT_POS_TICKS * old[S.ROT_VEL]
                val centerDriveTicks = old[S.LAT_VEL] + DRIVE_ENC_CENTER_POS_TICKS * old[S.ROT_VEL]

                /* Add noise terms for occasional skipping/slipping, ~1 tick/100 sec. */
                val leftSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
                val rightSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
                val centerSlip = 0.01 * CYCLE_PERIOD_SECONDS * noise()
                new[S.ENC_LEFT] = old[S.ENC_LEFT] + leftDriveTicks + leftSlip
                new[S.ENC_RIGHT] = old[S.ENC_RIGHT] + rightDriveTicks + rightSlip
                new[S.ENC_CENTER] = old[S.ENC_CENTER] + centerDriveTicks + centerSlip
            }
        } }
    }
}