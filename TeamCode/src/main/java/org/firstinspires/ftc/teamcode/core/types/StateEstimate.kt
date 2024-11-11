package org.firstinspires.ftc.teamcode.core.types

data class StateEstimate(
    val xMeters: Double,
    val yMeters: Double,
    val yawRads: Double,
    val latMetersPerCycle: Double,
    val longMetersPerCycle: Double,
    val rotRadsPerCycle: Double,
    val liftHeightMeters: Double,
    val armAngleRads: Double,
)