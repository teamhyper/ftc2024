package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotor

@OpModeRegistrar
fun registerMotors(manager: OpModeManager) {
    val motors = listOf("front_left", "front_right", "back_left", "back_right", "arm")
    for (m in motors) {
        manager.register("$m test", MotorTest(m))
    }
}

class MotorTest(val motorName: String) : OpMode() {
    private lateinit var motor: DcMotor

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, motorName)
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()
        telemetry.addData("power", motor.power)
    }
}