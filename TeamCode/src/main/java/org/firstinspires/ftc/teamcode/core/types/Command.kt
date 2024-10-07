package org.firstinspires.ftc.teamcode.core.types

sealed class DriveCommand {
    data class DrivePositionCommand(
        val xMeters: Double,
        val yMeters: Double,
        val yawRads: Double,
        val latFeedForward: Double,
        val longFeedForward: Double,
        val rotFeedForward: Double,
    ) : DriveCommand()

    data class DriveVelocityCommand(
        val dXMetersPerCycle: Double,
        val dYMetersPerCycle: Double,
        val dYawRadsPerCycle: Double,
        val latFeedForward: Double,
        val longFeedForward: Double,
        val rotFeedForward: Double,
    ) : DriveCommand()
}

data class ClawCommand(
    val heightMeters: Double,
    val angleRads: Double,
    val liftFeedForward: Double,
)

data class Command(
    val drive: DriveCommand,
    val claw: ClawCommand
)