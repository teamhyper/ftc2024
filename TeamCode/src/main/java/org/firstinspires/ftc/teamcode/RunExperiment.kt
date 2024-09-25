package org.firstinspires.ftc.teamcode

import android.os.Environment
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import java.io.Writer
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.PI
import kotlin.math.sin

private fun<T> withOutputFile(block: (Writer) -> T): T {
    val now = Date()
    val format = SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
    val pathInSdCard = "FIRST/datalogs/experiment_${format.format(now)}.csv"
    val file = Environment.getExternalStorageDirectory().resolve(pathInSdCard)
    file.parentFile?.mkdirs()
    return file.bufferedWriter().use(block)
}

private fun Writer.writeHeader() {
    appendLine("Time,Power,Position")
}

private fun Writer.writeRow(time: Double, power: Double, position: Int) {
    appendLine("$time,$power,$position")
}

private fun controlSignal(time: Double): Double {
    val frequency = 1   // Hz
    val amplitude = 0.3  // % power
    return amplitude * sin(time * frequency * 2 * PI)
}

@TeleOp(name = "Data Logging Experiment")
class RunExperiment : LinearOpMode() {
    override fun runOpMode() {
        // initialize
        telemetry.addData("status", "initializing")
        val timer = ElapsedTime()
        val motor = hardwareMap.get(DcMotor::class.java, "arm")

        // wait for operator
        telemetry.addData("status", "ready")
        waitForStart()

        // start
        telemetry.addData("status", "running")
        timer.reset()
        withOutputFile { logFile ->
            logFile.writeHeader()
            while (timer.seconds() < 10) {
                // read inputs
                val time = timer.seconds()
                val power = controlSignal(time)
                val position = motor.currentPosition

                // write outputs
                logFile.writeRow(time = time, power = power, position = position)
                motor.power = power

                // sleep
                sleep(20)
            }
        }

        // shut down
        motor.power = 0.0
        telemetry.addData("status", "done")
    }
}