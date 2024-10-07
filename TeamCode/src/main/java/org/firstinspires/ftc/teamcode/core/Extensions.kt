/*
 * This file contains extension methods.  Extensions are a feature of Kotlin
 * that let you add methods and properties to other people's classes.  These
 * are just convenient shorthands.
 */
package org.firstinspires.ftc.teamcode.core

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName

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