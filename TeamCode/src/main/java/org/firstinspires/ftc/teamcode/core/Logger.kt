package org.firstinspires.ftc.teamcode.core

import android.os.Environment
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/*
 * The logger, when used, is responsible for writing data to CSV files on the
 * robot's SD card.
 */

interface Logger {
    fun log(meas: Measurement)
    fun log(state: StateEstimate)
    fun log(control: Control)
}

fun logger() = object : Logger {
    val dir
    init {
        val sdcard = Environment.getExternalStorageDirectory()
        val sdf = SimpleDateFormat("yyyy-MM-dd_hh-mm-ss", Locale.US)
        val date = sdf.format(Date())
        dir = sdcard.resolve("robot_logs/$date")
        dir.mkdirs()
    }

    val measWriter = dir.resolve("meas.csv").printWriter()
    val measCsv = csv<Measurement> {
        column("drive_enc_left") { it.drive.encoders.leftMeters }
        column("drive_enc_right") { it.drive.encoders.rightMeters }
        column("drive_enc_center") { it.drive.encoders.centerMeters }
        column("claw_enc") { it.claw.heightMeters }
    }
    init { measWriter.writeCsvHeader(measCsv) }
    override fun log(meas: Measurement) {
        measWriter.writeCsvRow(measCsv, meas)
    }

    val stateWriter = dir.resolve("state.csv").printWriter()
    val stateCsv = csv<StateEstimate> {
        column("drive_x") { it.drive.xMeters }
        column("drive_y") { it.drive.yMeters }
        column("drive_yaw") { it.drive.yawRads }
        column("claw_height") { it.claw.heightMeters }
    }
    init { stateWriter.writeCsvHeader(stateCsv) }
    override fun log(state: StateEstimate) {
        stateWriter.writeCsvRow(stateCsv, state)
    }

    val controlWriter = dir.resolve("control.csv").printWriter()
    val controlCsv = csv<Control> {
        column("drive_lat") { it.drive.latitudePower }
        column("drive_long") { it.drive.longitudePower }
        column("drive_rot") { it.drive.rotationPower }
        column("claw_lift") { it.claw.liftPower }
    }
    init { controlWriter.writeCsvHeader(controlCsv) }
    override fun log(control: Control) {
        controlWriter.writeCsvRow(controlCsv, control)
    }
}
