/*
 * This file describes the type of control data sent to actuators.
 */

package org.firstinspires.ftc.teamcode.core.types

data class DriveControl(
    val latitudePower: Double,
    val longitudePower: Double,
    val rotationPower: Double,
)

data class ClawControl(
    val liftPower: Double,
    val clawAngleRads: Double,
)

data class Control(
    val drive: DriveControl,
    val claw: ClawControl,
)