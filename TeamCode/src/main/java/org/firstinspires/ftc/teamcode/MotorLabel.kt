package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "front_left")
class FrontLeft : OpMode() {
    private lateinit var motor: DcMotor

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "front_left")
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", motor.power)
    }
}

@TeleOp(name = "front_right")
class FrontRight : OpMode() {
    private lateinit var motor: DcMotor

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "front_right")
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", motor.power)
    }
}

@TeleOp(name = "back_left")
class BackLeft : OpMode() {
    private lateinit var motor: DcMotor

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "back_left")
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", motor.power)
    }
}

@TeleOp(name = "back_right")
class BackRight : OpMode() {
    private lateinit var motor: DcMotor

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "back_right")
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", motor.power)
    }
}