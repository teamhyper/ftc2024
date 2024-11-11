/*
 * This file contains extension methods.  Extensions are a feature of Kotlin
 * that let you add methods and properties to other people's classes.  These
 * are just convenient shorthands.
 */
package org.firstinspires.ftc.teamcode.core.ext

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.core.types.GamepadState
import org.firstinspires.ftc.teamcode.core.types.UserInput

fun HardwareMap.getMotor(name: String): DcMotorEx =
    get(DcMotorEx::class.java, name)

fun HardwareMap.getIMU(name: String): IMU =
    get(IMU::class.java, name)

fun HardwareMap.getCamera(name: String): CameraName =
    get(CameraName::class.java, name)

fun HardwareMap.getTouchSensor(name: String): TouchSensor =
    get(TouchSensor::class.java, name)

fun HardwareMap.getServo(name: String): Servo =
    get(Servo::class.java, name)

fun HardwareMap.getAllHubs(): List<LynxModule> =
    getAll(LynxModule::class.java)

val Gamepad.state get() = GamepadState(
    leftStickX = left_stick_x.toDouble(),
    leftStickY = left_stick_y.toDouble(),
    rightStickX = right_stick_x.toDouble(),
    rightStickY = right_stick_y.toDouble(),
    dPadUp = dpad_up,
    dPadDown = dpad_down,
    dPadLeft = dpad_left,
    dPadRight = dpad_right,
    a = a,
    b = b,
    x = x,
    y = y,
    start = start,
    back = back,
    leftBumper = left_bumper,
    rightBumper = right_bumper,
    leftStickButton = left_stick_button,
    rightStickButton = right_stick_button,
    leftTrigger = left_trigger.toDouble(),
    rightTrigger = right_trigger.toDouble(),
)

val OpMode.userInput get() = UserInput(
    gamepad1 = gamepad1.state,
    gamepad2 = gamepad2.state,
)