package org.firstinspires.ftc.teamcode.samples

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

/*
 * In order to develop a model of the robot's dynamics, we need to collect data
 * about how the various sensors respond to control inputs.  This is easiest if
 * we can feed large amounts of mostly random input into the motors and collect
 * the data automatically.  In order for the robot to move autonomously for long
 * periods of time without crashing, we use the camera and a proportional
 * feedback loop, and send a desired position signal.  The desired position will
 * always be close to a fixed spot on the field with no obstacles around and a
 * clear view of the camera.  The robot will not rotate more than a small angle,
 * so we don't have to worry about nonlinearity.  Since it's only used for
 * testing, the control loop does not have to be particularly fast, but it does
 * have to be stable.  In fact, we want the gain to be as low as we can get away
 * with, to avoid saturating the motors.
 *
 * This file provides an OpMode for running these sorts of experiments.  The
 * recorded data is stored on the SD card, in CSV format.
 *
 * We want to collect data from all sensors at the same sample rate.  The
 * current value is just a guess.  For this experiment, we want this to be as
 * high as possible while still maintaining consistent timing.  The camera is
 * slower than this, so its samples will be repeated.
 *
 * 10,000,000 ns = 10 ms = (100 Hz)^-1
 * 1,000 samples / 100 Hz = 10 seconds total
 */
private const val SAMPLE_PERIOD_NS: Long = 10_000_000
private const val NUM_SAMPLES: Int = 1_000
private const val TARGET_ID: Int = 11

private class SampleTimer {
    private val startNs: Long = System.nanoTime()
    private var lastAckedTick: Long = -1

    /*
     * Wait for a new, unACKnowledged, tick to happen. We busy-wait here instead
     * of sleeping because we're only waiting for short amounts of time and
     * accuracy is more important than efficiency.
     */
    fun tick() {
        var ack = false
        while (!ack) {
            val lastTick = (System.nanoTime() - startNs) / SAMPLE_PERIOD_NS
            if (lastTick > lastAckedTick) {
                lastAckedTick = lastTick
                ack = true
            }
        }
    }
}

private class PRBS

/* This lets me use `frontLeft.current`, which is comfy. */
private val DcMotorEx.current: Double
    get() = getCurrent(CurrentUnit.AMPS)
private val IMU.angularVelocity: AngularVelocity
    get() = getRobotAngularVelocity(AngleUnit.RADIANS)

class DriveExperiment : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("status", "initializing")
        val frontLeft = hardwareMap.get(DcMotorEx::class.java, "front_left")
        val frontRight = hardwareMap.get(DcMotorEx::class.java, "front_right")
        val backLeft = hardwareMap.get(DcMotorEx::class.java, "back_left")
        val backRight = hardwareMap.get(DcMotorEx::class.java, "back_right")
        val imu = hardwareMap.get(IMU::class.java, "imu")
        val camera = hardwareMap.get(WebcamName::class.java, "camera")
        val aprilTags = AprilTagProcessor.Builder()
            .build()
        val visionPortal = VisionPortal.Builder()
            .setCamera(camera)
            .addProcessors(aprilTags)
            .build()

        telemetry.addData("status", "ready")
        waitForStart()

        telemetry.addData("status", "running")
        val timer = SampleTimer()
        var nticks = 0
        while (!isStopRequested && nticks++ < NUM_SAMPLES) {
            timer.tick()

            /* measure inputs */
            frontLeft.current
            frontLeft.currentPosition
            frontRight.current
            frontRight.currentPosition
            backLeft.current
            backLeft.currentPosition
            backRight.current
            backRight.currentPosition
            imu.angularVelocity.xRotationRate
            imu.angularVelocity.yRotationRate
            imu.angularVelocity.zRotationRate
            val pose = aprilTags.detections
                ?.firstOrNull { d -> d.id == TARGET_ID }
                ?.robotPose
            pose?.position?.x
            pose?.position?.y
            pose?.position?.z
            pose?.orientation?.yaw
            pose?.orientation?.pitch
            pose?.orientation?.roll

            /* if pose is null, we're lost! */
            if (pose == null) {
                requestOpModeStop()
            }

            /* calculate control signal */

            /* set outputs */

        }
    }
}