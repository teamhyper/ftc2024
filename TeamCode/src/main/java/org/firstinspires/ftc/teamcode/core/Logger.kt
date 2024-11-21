package org.firstinspires.ftc.teamcode.core

import android.os.Environment
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.core.csv.csv
import org.firstinspires.ftc.teamcode.core.csv.writeCsvHeader
import org.firstinspires.ftc.teamcode.core.csv.writeCsvRow
import org.firstinspires.ftc.teamcode.core.types.Command
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.DriveReference
import org.firstinspires.ftc.teamcode.core.types.LiftState
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/*
 * The logger, when used, is responsible for writing data to CSV files on the
 * robot's SD card.  It is also responsible for pushing data to telemetry.
 */

interface Logger: AutoCloseable {
    fun logMeasurement(meas: Measurement)
    fun logPrior(state: StateEstimate)
    fun logPosterior(state: StateEstimate)
    fun logCommand(command: Command)
    fun logControl(control: Control)
    fun flush()
}

fun logger(telemetry: Telemetry) = object : Logger {
    val dir: File
    init {
        val sdcard = Environment.getExternalStorageDirectory()
        val sdf = SimpleDateFormat("yyyy-MM-dd_hh-mm-ss", Locale.US)
        val date = sdf.format(Date())
        dir = sdcard.resolve("robot_logs/$date")
        dir.mkdirs()
    }

    val measWriter = dir.resolve("meas.csv").printWriter()
    val measCsv = csv<Measurement> {
        column("time") { it.timeSeconds }
        column("drive_enc_left") { it.leftDriveEncTicks }
        column("drive_enc_right") { it.rightDriveEncTicks }
        column("drive_enc_center") { it.centerDriveEncTicks }
        column("lift_enc_left") { it.leftLiftEncTicks }
        column("lift_enc_right") { it.rightLiftEncTicks }
        column("lift_home") { if (it.liftIsHome) 1 else 0 }
        column("arm_enc_ticks") { it.armEncTicks }
    }
    init { measWriter.writeCsvHeader(measCsv) }
    override fun logMeasurement(meas: Measurement) {
        measWriter.writeCsvRow(measCsv, meas)
        telemetry.addLine("left encoder: ${meas.leftDriveEncTicks}")
        telemetry.addLine("right encoder: ${meas.rightDriveEncTicks}")
        telemetry.addLine("center encoder: ${meas.centerDriveEncTicks}")
    }

    val priorWriter = dir.resolve("state_prior.csv").printWriter()
    val posteriorWriter = dir.resolve("state_posterior.csv").printWriter()
    val stateCsv = csv<StateEstimate> {
        column("drive_x") { it.xMeters }
        column("drive_y") { it.yMeters }
        column("drive_yaw") { it.yawRads }
        column("lift_height") {
            when (it.liftState) {
                is LiftState.Unknown -> "null"
                is LiftState.AtHeight -> it.liftState.heightMeters
            }
        }
        column("arm_angle") { it.armAngleRads }
        column("drive_lat_vel") { it.latMetersPerCycle }
        column("drive_long_vel") { it.longMetersPerCycle }
        column("drive_rot_vel") { it.rotRadsPerCycle }
    }
    init {
        priorWriter.writeCsvHeader(stateCsv)
        posteriorWriter.writeCsvHeader(stateCsv)
    }
    override fun logPrior(state: StateEstimate) {
        priorWriter.writeCsvRow(stateCsv, state)
    }

    override fun logPosterior(state: StateEstimate) {
        telemetry.addLine("position: (${state.xMeters} m, ${state.yMeters} m)")
        telemetry.addLine("heading: ${state.yawRads} rad")
        telemetry.addLine("lateral velocity: ${state.latMetersPerCycle * CYCLE_PERIOD_SECONDS} m/s")
        telemetry.addLine("longitudinal velocity: ${state.longMetersPerCycle * CYCLE_PERIOD_SECONDS} m/s")
        telemetry.addLine("angular velocity: ${state.rotRadsPerCycle * CYCLE_PERIOD_SECONDS} rad/s")
        when (state.liftState) {
            is LiftState.Unknown -> telemetry.addLine("lift not homed")
            is LiftState.AtHeight -> telemetry.addLine("lift height: ${state.liftState.heightMeters} m")
        }
        telemetry.addLine("arm angle: ${state.armAngleRads} rad")
        posteriorWriter.writeCsvRow(stateCsv, state)
    }

    val commandWriter = dir.resolve("command.csv").printWriter()
    val commandCsv = csv<Command> {
        column("drive_x") {
            when (it.driveReference) {
                is DriveReference.Velocity -> "null"
                is DriveReference.Position -> it.driveReference.xMeters
            }
        }
        column("drive_y") {
            when (it.driveReference) {
                is DriveReference.Velocity -> "null"
                is DriveReference.Position -> it.driveReference.yMeters
            }
        }
        column("drive_yaw") {
            when (it.driveReference) {
                is DriveReference.Velocity -> "null"
                is DriveReference.Position -> it.driveReference.yawRads
            }
        }
        column("drive_lat_vel") {
            when (it.driveReference) {
                is DriveReference.Velocity -> it.driveReference.latMetersPerCycle
                is DriveReference.Position -> "null"
            }
        }
        column("drive_long_vel") {
            when (it.driveReference) {
                is DriveReference.Velocity -> it.driveReference.longMetersPerCycle
                is DriveReference.Position -> "null"
            }
        }
        column("drive_rot_vel") {
            when (it.driveReference) {
                is DriveReference.Velocity -> it.driveReference.rotRadsPerCycle
                is DriveReference.Position -> "null"
            }
        }
        column("lift_height") { it.liftHeightMeters }
        column("arm_angle") { it.armAngleRads }
    }
    init { commandWriter.writeCsvHeader(commandCsv) }
    override fun logCommand(command: Command) {
        commandWriter.writeCsvRow(commandCsv, command)
    }

    val controlWriter = dir.resolve("control.csv").printWriter()
    val controlCsv = csv<Control> {
        column("drive_front_left") { it.frontLeftDrivePower }
        column("drive_front_right") { it.frontRightDrivePower }
        column("drive_back_left") { it.backRightDrivePower }
        column("drive_back_right") { it.backRightDrivePower }
        column("lift") { it.liftPower }
        column("arm") { it.armPower }
        column("claw_twist") { it.clawTwistDutyCycle }
        column("claw_grip") { it.clawGripDutyCycle }
    }
    init { controlWriter.writeCsvHeader(controlCsv) }
    override fun logControl(control: Control) {
        telemetry.addLine("claw twist: ${control.clawTwistDutyCycle}")
        telemetry.addLine("claw grip: ${control.clawGripDutyCycle}")
        controlWriter.writeCsvRow(controlCsv, control)
    }

    override fun flush() {
        measWriter.flush()
        priorWriter.flush()
        posteriorWriter.flush()
        commandWriter.flush()
        controlWriter.flush()
        telemetry.update()
    }

    override fun close() {
        measWriter.close()
        priorWriter.close()
        posteriorWriter.close()
        commandWriter.close()
        controlWriter.close()
    }
}
