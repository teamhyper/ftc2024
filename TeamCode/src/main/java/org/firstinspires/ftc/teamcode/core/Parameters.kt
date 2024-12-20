/*
 * This file collects together numerical constants that must be measured/tuned.
 */

package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import kotlin.math.PI

const val CYCLE_PERIOD_SECONDS = 0.05 /* 50 ms -> 20 Hz */
/* see: https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/ */
const val ODOMETRY_PPR = 2000
const val ODOMETRY_WHEEL_DIAMETER_METERS = 0.032
const val DRIVE_METERS_PER_TICK = PI * ODOMETRY_WHEEL_DIAMETER_METERS / ODOMETRY_PPR
const val DRIVE_RADIUS_METERS = 0.156
const val DRIVE_YAW_RADS_PER_TICK = DRIVE_METERS_PER_TICK / DRIVE_RADIUS_METERS
const val DRIVE_ENC_LEFT_POS_TICKS = 0.156 / DRIVE_METERS_PER_TICK
const val DRIVE_ENC_RIGHT_POS_TICKS = 0.156 / DRIVE_METERS_PER_TICK
const val DRIVE_ENC_CENTER_POS_TICKS = 0.144 / DRIVE_METERS_PER_TICK
const val LIFT_METERS_PER_TICK = 1.0
const val ARM_RADS_PER_TICK = 1.0
const val CLAW_GRIP_OPEN = 0.6
const val CLAW_GRIP_CLOSED = 1.0
const val CLAW_TWIST_HORIZONTAL = 0.0
const val CLAW_TWIST_VERTICAL = 0.35
const val DRIVE_CAMERA_X_METERS = 0.0
const val DRIVE_CAMERA_Y_METERS = 0.0
const val DRIVE_CAMERA_Z_METERS = 0.0
const val DRIVE_CAMERA_YAW_RADS = 0.0
const val DRIVE_CAMERA_PITCH_RADS = 0.0
const val DRIVE_CAMERA_ROLL_RADS = 0.0

val driveCameraPosition get() = Position(
    DistanceUnit.METER,
    DRIVE_CAMERA_X_METERS,
    DRIVE_CAMERA_Y_METERS,
    DRIVE_CAMERA_Z_METERS,
    0,
)

val driveCameraOrientation get() = YawPitchRollAngles(
    AngleUnit.RADIANS,
    DRIVE_CAMERA_YAW_RADS,
    DRIVE_CAMERA_PITCH_RADS,
    DRIVE_CAMERA_ROLL_RADS,
    0,
)