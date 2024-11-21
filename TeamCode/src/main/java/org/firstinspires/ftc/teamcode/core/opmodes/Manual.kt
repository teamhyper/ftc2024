package org.firstinspires.ftc.teamcode.core.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.CLAW_GRIP_CLOSED
import org.firstinspires.ftc.teamcode.core.CLAW_GRIP_OPEN
import org.firstinspires.ftc.teamcode.core.CLAW_TWIST_HORIZONTAL
import org.firstinspires.ftc.teamcode.core.CLAW_TWIST_VERTICAL
import org.firstinspires.ftc.teamcode.core.Planner
import org.firstinspires.ftc.teamcode.core.types.Command
import org.firstinspires.ftc.teamcode.core.types.DriveReference
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import org.firstinspires.ftc.teamcode.core.types.UserInput

@TeleOp(name = "Full Manual")
class Manual : StandardOpMode() {
    enum class GripState { OPEN, CLOSED }
    enum class TwistState { HORIZONTAL, VERTICAL }

    override fun createPlanner() = object : Planner {
        var gripState = GripState.CLOSED
        var twistState = TwistState.HORIZONTAL

        override fun process(input: UserInput, state: StateEstimate): Command {
            /* Unused for now, since PID params are all 0. */
            val driveReference = DriveReference.Velocity(0.0, 0.0, 0.0)
            val liftHeightMeters = 0.0
            val armAngleRads = 0.0

            /* Manual drive using feed-forward for now. */
            val driveLatFF = input.gamepad1.leftStickX
            val driveLongFF = -input.gamepad1.leftStickY
            val driveRotFF = -input.gamepad1.rightStickX
            val liftFFMag = input.gamepad1.leftTrigger
            val liftFFSign = if (input.gamepad1.leftBumper) +1.0 else -1.0
            val liftFF = liftFFMag * liftFFSign
            val armFFMag = input.gamepad1.rightTrigger
            val armFFSign = if (input.gamepad1.rightBumper) +1.0 else -1.0
            val armFF = armFFMag * armFFSign

            /* Control servos using toggles. */
            if (input.gamepad1.a) gripState = GripState.CLOSED
            if (input.gamepad1.b) gripState = GripState.OPEN
            val clawGripDutyCycle = when (gripState) {
                GripState.OPEN -> CLAW_GRIP_OPEN
                GripState.CLOSED -> CLAW_GRIP_CLOSED
            }
            if (input.gamepad1.x) twistState = TwistState.HORIZONTAL
            if (input.gamepad1.y) twistState = TwistState.VERTICAL
            val clawTwistDutyCycle = when (twistState) {
                TwistState.HORIZONTAL -> CLAW_TWIST_HORIZONTAL
                TwistState.VERTICAL -> CLAW_TWIST_VERTICAL
            }

            return Command(
                driveReference = driveReference,
                driveLatFF = driveLatFF,
                driveLongFF = driveLongFF,
                driveRotFF = driveRotFF,
                liftHeightMeters = liftHeightMeters,
                liftFF = liftFF,
                armAngleRads = armAngleRads,
                armFF = armFF,
                clawTwistDutyCycle = clawTwistDutyCycle,
                clawGripDutyCycle = clawGripDutyCycle,
            )
        }
    }
}