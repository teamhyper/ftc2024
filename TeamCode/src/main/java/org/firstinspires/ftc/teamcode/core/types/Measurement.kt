/*
 * This file defines the type of data read from sensors.
 */

package org.firstinspires.ftc.teamcode.core.types

data class DriveEncoderMeasurement(
    val leftMeters: Double,
    val rightMeters: Double,
    val centerMeters: Double,
)

data class IMUMeasurement(
    val yawRads: Double
)

data class AprilTagMeasurement(
    val id: Int,
    val xMeters: Double,
    val yMeters: Double,
    val zMeters: Double,
)

data class DriveCameraMeasurement(
    val aprilTags: List<AprilTagMeasurement>
)

data class ClawCameraMeasurement(
    val todo: Unit,
)

data class DriveMeasurement(
    val encoders: DriveEncoderMeasurement,
    val imu: IMUMeasurement,
    val camera: DriveCameraMeasurement,
)

data class ClawMeasurement(
    val heightMeters: Double,
    val isHome: Boolean,
    val camera: ClawCameraMeasurement,
)

data class Measurement(
    val drive: DriveMeasurement,
    val claw: ClawMeasurement,
)