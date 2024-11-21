package org.firstinspires.ftc.teamcode.core.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.CLAW_GRIP_CLOSED
import org.firstinspires.ftc.teamcode.core.CLAW_TWIST_HORIZONTAL
import org.firstinspires.ftc.teamcode.core.Planner
import org.firstinspires.ftc.teamcode.core.types.Command
import org.firstinspires.ftc.teamcode.core.types.DriveReference
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import org.firstinspires.ftc.teamcode.core.types.UserInput
import kotlin.math.PI

/* 2 ft = 0.6096 m */
const val GRID_METERS = 0.6096
const val ANGLE_STEP_RADS = PI / 4

@TeleOp(name = "Grid Snapper")
class GridSnapper : StandardOpMode() {
    override fun createPlanner() = object : Planner {
        var x = 0
        var y = 0
        var angle = 0
        var upWasPressed = false
        var downWasPressed = false
        var leftWasPressed = false
        var rightWasPressed = false
        var lWasPressed = false
        var rWasPressed = false

        override fun process(input: UserInput, state: StateEstimate): Command {
            /*
             * We want to adjust x and y whenever the dpad buttons are pressed.
             * If we naively wrote something like:
             *
             *     if (input.gamepad1.dPadUp) y++
             *
             * then we'd end up rapidly incrementing y, once per tick, as long
             * as the button was held.  Since tapping a button for a fraction of
             * a second is hard, we cut the user a break and only increment the
             * coordinates on the *first* tick the button is pressed.
             */
            if (!upWasPressed && input.gamepad1.dPadUp) y++
            if (!downWasPressed && input.gamepad1.dPadDown) y--
            if (!leftWasPressed && input.gamepad1.dPadLeft) x--
            if (!rightWasPressed && input.gamepad1.dPadRight) x++
            if (!lWasPressed && input.gamepad1.leftBumper) angle++
            if (!rWasPressed && input.gamepad1.rightBumper) angle--

            upWasPressed = input.gamepad1.dPadUp
            downWasPressed = input.gamepad1.dPadDown
            leftWasPressed = input.gamepad1.dPadLeft
            rightWasPressed = input.gamepad1.dPadRight
            lWasPressed = input.gamepad1.leftBumper
            rWasPressed = input.gamepad1.rightBumper

            return Command(
                driveReference = DriveReference.Position(
                    xMeters = x * GRID_METERS,
                    yMeters = y * GRID_METERS,
                    yawRads = angle * ANGLE_STEP_RADS,
                ),
                driveLatFF = 0.0,
                driveLongFF = 0.0,
                driveRotFF = 0.0,
                liftHeightMeters = 0.0,
                liftFF = 0.0,
                armAngleRads = 0.0,
                armFF = 0.0,
                clawTwistDutyCycle = CLAW_TWIST_HORIZONTAL,
                clawGripDutyCycle = CLAW_GRIP_CLOSED,
            )
        }
    }
}