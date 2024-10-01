package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

@OpModeRegistrar
fun registerMotors(manager: OpModeManager) {
    val motors = listOf("front_left", "front_right", "back_left", "back_right", "arm")
    for (m in motors) {
        manager.register("$m test", MotorTest(m))
    }
}

class MotorTest(val motorName: String) : OpMode() {
    private lateinit var motor: DcMotorEx

    override fun init() {
        motor = hardwareMap.get(DcMotorEx::class.java, motorName)
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble()

        telemetry.addData("power (%)", motor.power)
        telemetry.addData("position (ticks)", motor.currentPosition)
        telemetry.addData("current (A)", motor.getCurrent(CurrentUnit.AMPS))
    }
}