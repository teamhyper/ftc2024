package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.TouchSensor
import kotlin.math.min

enum class State { INIT, RESET, HOME, AWAY }

@TeleOp(name = "Arm Test")
class ArmTest : OpMode() {
    private lateinit var motor: DcMotor
    private lateinit var switch: TouchSensor

    private var state: State = State.INIT

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "arm")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        switch = hardwareMap.get(TouchSensor::class.java, "arm_limit")
        telemetry.addData("switch manufacturer", switch.manufacturer)
        telemetry.addData("switch connection", switch.connectionInfo)
    }

    override fun loop() {
        val command = -gamepad1.left_stick_y.toDouble()

        when (state) {
            State.INIT -> {
                if (switch.isPressed) {
                    state = State.RESET
                    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                } else {
                    // stay in INIT state
                    motor.power = 0.4
                }
            }
            State.RESET -> {
                state = State.HOME
                motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
            State.HOME -> {
                if (switch.isPressed) {
                    // stay in HOME state
                    motor.power = min(command, 0.0)
                } else {
                    state = State.AWAY
                    motor.power = command
                }
            }
            State.AWAY -> {
                if (switch.isPressed) {
                    state = State.RESET
                    motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                } else {
                    // stay in AWAY state
                    motor.power = command
                }
            }
        }

        telemetry.addData("command", command)
        telemetry.addData("power", motor.power)
        telemetry.addData("position", motor.currentPosition)
        telemetry.addData("switch pressed", switch.isPressed)
    }
}