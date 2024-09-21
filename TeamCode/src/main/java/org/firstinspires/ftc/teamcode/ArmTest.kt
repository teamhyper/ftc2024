package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "Arm Test")
class ArmTest : OpMode() {
    private lateinit var motor: DcMotor

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "arm")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", motor.power)
        telemetry.addData("position", motor.currentPosition)
    }
}