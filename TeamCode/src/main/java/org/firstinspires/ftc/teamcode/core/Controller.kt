package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.math.pid
import org.firstinspires.ftc.teamcode.core.types.Command
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.DriveReference
import org.firstinspires.ftc.teamcode.core.types.LiftState
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

interface Controller {
    /*
     * Decide what control signal to send to actuators.  This decision is based
     * on the command, as well as our current estimate of the state.  The
     * command consists of the desired position and velocity of the robot, as
     * well as feed-forward terms (the control signal that we expect to need
     * to use to follow that command).  The job of this function is to modify
     * this control signal to compensate for errors, as measured by the state
     * estimate.
     */
    fun follow(
        command: Command,
        state: StateEstimate,
    ): Control
}

fun controller() = object : Controller {
    /* guesses.  todo: actual control theory */
    val latPosPID = pid(kp = 4.0)
    val longPosPID = pid(kp = 2.0)
    val yawPosPID = pid(kp = 1.0)
    val latVelPID = pid(kp = 1.0)
    val longVelPID = pid(kp = 1.0)
    val rotVelPID = pid(kp = 1.0)
    val liftPID = pid()
    val armPID = pid()

    override fun follow(command: Command, state: StateEstimate): Control {
        /* Update drive train PID controllers. */
        val latFB: Double
        val longFB: Double
        val rotFB: Double
        when (command.driveReference) {
            is DriveReference.Position -> {
                val xError = command.driveReference.xMeters - state.xMeters
                val yError = command.driveReference.yMeters - state.yMeters
                val latError = cos(state.yawRads) * xError + sin(state.yawRads) * yError
                val longError = -sin(state.yawRads) * xError + cos(state.yawRads) * yError
                /*
                 * These coordinates are in the robot's reference frame, so
                 * our position is always 0 by definition.
                 */
                latFB = latPosPID.follow(
                    target = latError,
                    measured = 0.0,
                )
                longFB = longPosPID.follow(
                    target = longError,
                    measured = 0.0
                )
                rotFB = yawPosPID.follow(
                    target = command.driveReference.yawRads,
                    measured = state.yawRads,
                )
                latVelPID.reset()
                longVelPID.reset()
                rotVelPID.reset()
            }
            is DriveReference.Velocity -> {
                latPosPID.reset()
                longPosPID.reset()
                yawPosPID.reset()
                latFB = latVelPID.follow(
                    target = command.driveReference.latMetersPerCycle,
                    measured = state.latMetersPerCycle,
                )
                longFB = longVelPID.follow(
                    target = command.driveReference.longMetersPerCycle,
                    measured = state.longMetersPerCycle,
                )
                rotFB = rotVelPID.follow(
                    target = command.driveReference.rotRadsPerCycle,
                    measured = state.rotRadsPerCycle,
                )
            }
        }

        /* Add drive feed-forward */
        val latPower = latFB + command.driveLatFF
        val longPower = longFB + command.driveLongFF
        val rotPower = rotFB + command.driveRotFF

        /* Convert to wheel speeds. */
        val scale = 1.0 / max(1.0, abs(latPower) + abs(longPower) + abs(rotPower))
        val frontLeftPower = scale * (latPower + longPower - rotPower)
        val frontRightPower = scale * (-latPower + longPower + rotPower)
        val backLeftPower = scale * (-latPower + longPower - rotPower)
        val backRightPower = scale * (latPower + longPower + rotPower)

        /* Compute lift and arm PID */
        val liftFB = when (state.liftState) {
            is LiftState.Unknown -> 0.0
            is LiftState.AtHeight -> liftPID.follow(
                target = command.liftHeightMeters,
                measured = state.liftState.heightMeters,
            )
        }
        val liftPower = liftFB + command.liftFF
        val armFB = armPID.follow(
            target = command.armAngleRads,
            measured = state.armAngleRads,
        )
        val armPower = armFB + command.armFF

        return Control(
            frontLeftDrivePower = frontLeftPower,
            frontRightDrivePower = frontRightPower,
            backLeftDrivePower = backLeftPower,
            backRightDrivePower = backRightPower,
            liftPower = liftPower,
            armPower = armPower,
            clawTwistDutyCycle = command.clawTwistDutyCycle,
            clawGripDutyCycle = command.clawGripDutyCycle,
        )
    }
}