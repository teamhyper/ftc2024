package org.firstinspires.ftc.teamcode.core.types

data class DriveStateEstimate(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
    val latMetersPerCycle: Double,
    val longMetersPerCycle: Double,
    val rotRadsPerCycle: Double,
)

data class ClawStateEstimate(
    val heightMeters: Double,
)

data class StateEstimate(
    val drive: DriveStateEstimate,
    val claw: ClawStateEstimate,
)