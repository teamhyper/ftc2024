/*
 * This file describes the type of control data sent to actuators.
 */

package org.firstinspires.ftc.teamcode.core.types

data class Control(
    val frontLeftDrivePower: Double,
    val frontRightDrivePower: Double,
    val backLeftDrivePower: Double,
    val backRightDrivePower: Double,
    val liftPower: Double,
    val armPower: Double,
    val clawTwistDutyCycle: Double,
    val clawGripDutyCycle: Double,
)