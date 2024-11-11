/*
 * This file defines the type of data read from sensors.
 */

package org.firstinspires.ftc.teamcode.core.types

data class AprilTagMeasurement(
    val id: Int,
    val xMeters: Double,
    val yMeters: Double,
    val zMeters: Double,
)

data class Measurement(
    val leftDriveEncTicks: Double,
    val rightDriveEncTicks: Double,
    val centerDriveEncTicks: Double,
    val leftLiftEncTicks: Double,
    val rightLiftEncTicks: Double,
    val armEncTicks: Double,
    val liftIsHome: Boolean,
    val imuYawRads: Double,
    val aprilTags: List<AprilTagMeasurement>,
)