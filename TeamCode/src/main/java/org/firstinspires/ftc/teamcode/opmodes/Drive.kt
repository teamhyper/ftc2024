/*
 * This file provides an OpMode for manual driving only.
 */

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.core.CYCLE_PERIOD_SECONDS
import org.firstinspires.ftc.teamcode.core.hardware
import org.firstinspires.ftc.teamcode.core.types.ClawControl
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.DriveControl

@TeleOp(name = "Manual drive")
class Drive : LinearOpMode() {
    override fun runOpMode() {
        val hw = hardware(hardwareMap)
        val cycleTimer = ElapsedTime()

        waitForStart()
        while (opModeIsActive()) {
            /* The timer is set to 0 at the start of a cycle. */
            cycleTimer.reset()

            /* Measure inputs from all sensors. */
            val meas = hw.measure()

            /* Calculate control signal. */
            val driveCtl = DriveControl(
                latitudePower = gamepad1.left_stick_x.toDouble(),
                longitudePower = -gamepad1.left_stick_y.toDouble(),
                rotationPower = -gamepad1.right_stick_x.toDouble(),
            )
            val clawCtl = ClawControl(
                liftPower = 0.0,
                clawAngleRads = 0.0,
            )
            val robotCtl = Control(driveCtl, clawCtl)

            /* Send control signal to actuators. */
            hw.control(robotCtl)

            /* Wait before starting the next cycle. */
            while (cycleTimer.seconds() < CYCLE_PERIOD_SECONDS) {
                sleep(1)
            }
        }
    }
}