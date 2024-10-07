package org.firstinspires.ftc.teamcode.core.types

data class DriveStateEstimate(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
    val dXMetersPerCycle: Double,
    val dYMetersPerCycle: Double,
    val dYawRadsPerCycle: Double,
)

data class ClawStateEstimate(
    val heightMeters: Double,
)

data class StateEstimate(
    val drive: DriveStateEstimate,
    val claw: ClawStateEstimate,
)