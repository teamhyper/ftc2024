/*
 * This file defines the low-level details of how measurements are read from
 * sensors and how control signals are sent to actuators.  It also handles
 * initialization of hardware.
 *
 * To use it, call `hardware()` during initialization, passing it the hardware
 * map.  It returns a `Hardware` object.  During each cycle, call `measure()`
 * to read data from all sensors, and `control()` to send a signal to all
 * actuators.
 */

package org.firstinspires.ftc.teamcode.core

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.core.ext.getAllHubs
import org.firstinspires.ftc.teamcode.core.ext.getCamera
import org.firstinspires.ftc.teamcode.core.ext.getIMU
import org.firstinspires.ftc.teamcode.core.ext.getMotor
import org.firstinspires.ftc.teamcode.core.ext.getServo
import org.firstinspires.ftc.teamcode.core.ext.getTouchSensor
import org.firstinspires.ftc.teamcode.core.types.AprilTagMeasurement
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

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
fun hardware(hw: HardwareMap) = object : Hardware {
    /*
     * Drive motors.  These are RUN_WITHOUT_ENCODER because our program produces
     * the power signal, not the position signal.  We use the encoders on the
     * dead wheels ourselves to decide this signal.
     */
    val frontLeftDriveMotor = hw.getMotor("Front_Left").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val frontRightDriveMotor = hw.getMotor("Front_Right").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val backLeftDriveMotor = hw.getMotor("Back_Left").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val backRightDriveMotor = hw.getMotor("Back_Right").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    /*
     * The encoders on the dead wheels are plugged into the motor encoder
     * ports, so we define aliases here.
     */
    val leftDriveEncoder = frontLeftDriveMotor
    val rightDriveEncoder = frontRightDriveMotor
    val centerDriveEncoder = backRightDriveMotor

    /*
     * Lifter/arm/claw hardware. Again we use run_without_encoder since we
     * have two motors stuck together, and we want to make sure they don't run
     * against each other by always feeding them the same power signal.
     */
    val leftLiftMotor = hw.getMotor("Right_Lift").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val rightLiftMotor = hw.getMotor("Left_Lift").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val liftLimit = hw.getTouchSensor("Lift_Sensor")
    val armMotor = hw.getMotor("Arm").apply {
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val clawTwistServo = hw.getServo("Wrist").apply {
        direction = Servo.Direction.FORWARD
    }
    val clawGripServo = hw.getServo("Claw").apply {
        direction = Servo.Direction.FORWARD
    }

    /* The IMU is additionally used by the drivetrain for state estimation. */
    /*
    val imu = hw.getIMU("imu").apply {
        initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT,
        )))
        resetYaw()
    }
    */

    /* The drive camera is used for detecting AprilTags. */
    val driveAprilTagProcessor = AprilTagProcessor.Builder()
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
        .setDrawCubeProjection(true)
        .setCameraPose(driveCameraPosition, driveCameraOrientation)
        .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
        .build()
    init {
        VisionPortal.Builder()
            .setCamera(hw.getCamera("drive_camera"))
            .addProcessor(driveAprilTagProcessor)
            .setAutoStartStreamOnBuild(true)
            .build()
    }

    /*
     * Set all control hubs to use MANUAL mode.  This improves performance, but
     * means that the cache must manually be cleared on each iteration.  We do
     * this in our `measure()` override.
     */
    val controlHubs = hw.getAllHubs().onEach {
        it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
    }

    val timer = ElapsedTime()

    /*
     * Called at the start of each control cycle.  Read data from all sensors.
     */
    override fun measure(): Measurement {
        /* Update the bulk cache so we read new values. */
        controlHubs.forEach { it.clearBulkCache() }
        return Measurement(
            leftDriveEncTicks = leftDriveEncoder.currentPosition.toDouble(),
            rightDriveEncTicks = rightDriveEncoder.currentPosition.toDouble(),
            centerDriveEncTicks = centerDriveEncoder.currentPosition.toDouble(),
            leftLiftEncTicks = leftLiftMotor.currentPosition.toDouble(),
            rightLiftEncTicks = rightLiftMotor.currentPosition.toDouble(),
            liftIsHome = liftLimit.isPressed,
            imuYawRads = 0.0 /*imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)*/,
            aprilTags = /*driveAprilTagProcessor.freshDetections?.map {
                AprilTagMeasurement(
                    id = it.id,
                    xMeters = it.ftcPose.x,
                    yMeters = it.ftcPose.y,
                    zMeters = it.ftcPose.z,
                )
            } ?:*/ emptyList(),
            armEncTicks = armMotor.currentPosition.toDouble(),
            timeSeconds = timer.seconds(),
        )
    }

    /*
     * Called later during each control cycle.  Write all data to actuators.
     */
    override fun control(ctl: Control) {
        frontLeftDriveMotor.power = ctl.frontLeftDrivePower
        frontRightDriveMotor.power = ctl.frontRightDrivePower
        backLeftDriveMotor.power = ctl.backLeftDrivePower
        backRightDriveMotor.power = ctl.backRightDrivePower
        leftLiftMotor.power = ctl.liftPower
        rightLiftMotor.power = ctl.liftPower
        armMotor.power = ctl.armPower
        clawTwistServo.position = ctl.clawTwistDutyCycle
        clawGripServo.position = ctl.clawGripDutyCycle
    }
}