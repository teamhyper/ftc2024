/*
 * This file defines the low-level details of how measurements are read from
 * sensors and how control signals are sent to actuators.  It also handles
 * initialization of hardware.
 *
 * To use it, call `hardware()` during initialization, passing it the hardware
 * map.  It returns a `Hardware` object.  During each cycle, call `measure()`
 * to read data from all sensors, and `control()` to send a signal to all
 * actuators.
 *
 * Not much processing logic is done here, besides unit conversions and
 * normalization (e.g. if a motor only accepts inputs in the range [0, 1], but
 * your program sends 1.5, it is clamped down to 1).
 */

package org.firstinspires.ftc.teamcode.core

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.core.types.AprilTagMeasurement
import org.firstinspires.ftc.teamcode.core.types.ClawCameraMeasurement
import org.firstinspires.ftc.teamcode.core.types.ClawControl
import org.firstinspires.ftc.teamcode.core.types.ClawMeasurement
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.DriveCameraMeasurement
import org.firstinspires.ftc.teamcode.core.types.DriveControl
import org.firstinspires.ftc.teamcode.core.types.DriveEncoderMeasurement
import org.firstinspires.ftc.teamcode.core.types.DriveMeasurement
import org.firstinspires.ftc.teamcode.core.types.IMUMeasurement
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.min

/*
 * The interface that hardware presents to the rest of the control logic.
 */
interface Hardware {
    fun measure(): Measurement
    fun control(control: Control)
}

/*
 * The canonical implementation of the Hardware interface.  This should be
 * called during initialization.  The rest of this file is just the definition
 * of HardwareImpl.
 */
fun hardware(hw: HardwareMap): Hardware = HardwareImpl(hw)

private class HardwareImpl(hw: HardwareMap) : Hardware {
    /*
     * Drive motors.  These are RUN_WITHOUT_ENCODER because our program produces
     * the power signal, not the position signal.  We use the encoders on the
     * dead wheels ourselves to decide this signal.
     */
    val frontLeftDriveMotor = hw.getMotor("drive_front_left").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val frontRightDriveMotor = hw.getMotor("drive_front_right").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val backLeftDriveMotor = hw.getMotor("drive_back_left").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val backRightDriveMotor = hw.getMotor("drive_back_right").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    /*
     * The encoders on the dead wheels are plugged into the motor encoder
     * ports.
     */
    val leftDriveEncoder = frontLeftDriveMotor
    val rightDriveEncoder = frontRightDriveMotor
    val centerDriveEncoder = backRightDriveMotor

    /*
     * We report only the number of ticks since the last cycle, to get velocity
     * in units of "ticks per cycle time".  To make this work we keep track of
     * the last read encoder position.  Control code can integrate this if it
     * needs an absolute measurement.  This also means we never need to ask
     * the hardware to reset itself.
     */
    var leftDriveLastPosition = leftDriveEncoder.currentPosition
    var rightDriveLastPosition = rightDriveEncoder.currentPosition
    var centerDriveLastPosition = centerDriveEncoder.currentPosition

    /* The IMU is additionally used by the drivetrain for state estimation. */
    val imu = hw.getIMU("imu").apply {
        val orientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT,
        )
        initialize(IMU.Parameters(orientation))
        resetYaw()
    }

    /*
     * We also report only the delta yaw measured from the IMU.  This is
     * initialized to zero since we just called resetYaw().
     */
    var imuLastYawRads = 0.0

    /* The drive camera is used for detecting AprilTags. */
    val driveAprilTagProcessor = AprilTagProcessor.Builder()
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
        .setDrawCubeProjection(true)
        .setCameraPose(driveCameraPosition(), driveCameraOrientation())
        .build()
    init {
        VisionPortal.Builder()
            .setCamera(hw.getCamera("drive_camera"))
            .addProcessor(driveAprilTagProcessor)
            .setAutoStartStreamOnBuild(true)
            .build()
    }

    /* Claw hardware.  MAKE SURE to set the direction right before running! */
    val leftClawLiftMotor = hw.getMotor("claw_lift_left").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val rightClawLiftMotor = hw.getMotor("claw_lift_right").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val clawLiftLimit = hw.getTouchSensor("claw_lift_limit")
    val clawGripServo = hw.getServo("claw_grip").apply {
        direction = Servo.Direction.FORWARD
        scaleRange(CLAW_GRIP_MIN_POSITION, CLAW_GRIP_MAX_POSITION)
    }

    /* The last recorded positions of each claw lift encoder. */
    var leftClawLiftLastPosition = leftClawLiftMotor.currentPosition
    var rightClawLiftLastPosition = rightClawLiftMotor.currentPosition

    /*
     * Set all control hubs to use MANUAL mode.  This improves performance, but
     * means that the cache must manually be cleared on each iteration.  See
     * ConceptMotorBulkRead.
     */
    val controlHubs = hw.getAllHubs().onEach {
        it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
    }

    /* Called at the start of each control cycle. */
    fun clearCache() {
        controlHubs.forEach { it.clearBulkCache() }
    }

    fun measureDriveEncoders(): DriveEncoderMeasurement {
        val leftDriveDelta =
            leftDriveEncoder.currentPosition - leftDriveLastPosition
        leftDriveLastPosition += leftDriveDelta
        val rightDriveDelta =
            rightDriveEncoder.currentPosition - rightDriveLastPosition
        rightDriveLastPosition += rightDriveDelta
        val centerDriveDelta =
            centerDriveEncoder.currentPosition - centerDriveLastPosition
        centerDriveLastPosition += centerDriveDelta

        return DriveEncoderMeasurement(
            leftMetersPerCycle = leftDriveDelta * ODOMETRY_METERS_PER_TICK,
            rightMetersPerCycle = rightDriveDelta * ODOMETRY_METERS_PER_TICK,
            centerMetersPerCycle = centerDriveDelta * ODOMETRY_METERS_PER_TICK,
        )
    }

    fun measureIMU(): IMUMeasurement {
        val yawDeltaRads = imu.robotYawPitchRollAngles.yaw - imuLastYawRads
        imuLastYawRads += yawDeltaRads

        return IMUMeasurement(
            yawRadsPerCycle = yawDeltaRads
        )
    }

    fun measureDriveCamera(): DriveCameraMeasurement {
        val tags = driveAprilTagProcessor.freshDetections
            ?.map {
                AprilTagMeasurement(
                    id = it.id,
                    xMeters = it.ftcPose.x,
                    yMeters = it.ftcPose.y,
                    zMeters = it.ftcPose.z,
                )
            }
            ?: emptyList()

        return DriveCameraMeasurement(tags)
    }

    fun measureDrive(): DriveMeasurement =
        DriveMeasurement(
            measureDriveEncoders(),
            measureIMU(),
            measureDriveCamera(),
        )

    fun measureClawCamera(): ClawCameraMeasurement =
        ClawCameraMeasurement(todo = Unit)

    fun measureClaw(): ClawMeasurement {
        val leftDelta =
            leftClawLiftMotor.currentPosition - leftClawLiftLastPosition
        leftClawLiftLastPosition += leftDelta
        val rightDelta =
            rightClawLiftMotor.currentPosition - rightClawLiftLastPosition
        rightClawLiftLastPosition += rightDelta
        val averageDelta = (leftDelta + rightDelta) / 2.0

        return ClawMeasurement(
            heightMetersPerCycle = averageDelta * CLAW_LIFT_METERS_PER_TICK,
            isHome = clawLiftLimit.isPressed,
            camera = measureClawCamera(),
        )
    }

    /*
     * Called at the start of each control cycle.  Read data from all sensors.
     */
    override fun measure(): Measurement {
        clearCache()
        return Measurement(
            measureDrive(),
            measureClaw(),
        )
    }

    fun controlDrive(ctl: DriveControl) {
        val norm = abs(ctl.latitudePower) + abs(ctl.longitudePower) +
                abs(ctl.rotationPower)
        val scale = min(1.0, 1.0 / norm)
        frontLeftDriveMotor.power =
            scale * (ctl.longitudePower + ctl.latitudePower - ctl.rotationPower)
        frontRightDriveMotor.power =
            scale * (ctl.longitudePower - ctl.latitudePower + ctl.rotationPower)
        backLeftDriveMotor.power =
            scale * (ctl.longitudePower - ctl.latitudePower - ctl.rotationPower)
        backRightDriveMotor.power =
            scale * (ctl.longitudePower + ctl.latitudePower + ctl.rotationPower)
    }

    fun controlClaw(ctl: ClawControl) {
        leftClawLiftMotor.power = ctl.liftPower
        rightClawLiftMotor.power = ctl.liftPower
        clawGripServo.position = ctl.clawAngleRads / CLAW_GRIP_MAX_ANGLE_RADS
    }

    /*
     * Called later during each control cycle.  Write all data to actuators.
     */
    override fun control(ctl: Control) {
        controlDrive(ctl.drive)
        controlClaw(ctl.claw)
    }
}