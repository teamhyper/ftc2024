package org.firstinspires.ftc.teamcode.core.types

sealed class DriveReference {
    data class Position(
        val xMeters: Double,
        val yMeters: Double,
        val yawRads: Double,
    ) : DriveReference()

    data class Velocity(
        val latMetersPerCycle: Double,
        val longMetersPerCycle: Double,
        val rotRadsPerCycle: Double,
    ) : DriveReference()
}

data class Command(
    val driveReference: DriveReference,
    val driveLatFF: Double,
    val driveLongFF: Double,
    val driveRotFF: Double,
    val liftHeightMeters: Double,
    val liftFF: Double,
    val armAngleRads: Double,
    val armFF: Double,
    val clawTwistDutyCycle: Double,
    val clawGripDutyCycle: Double,
)