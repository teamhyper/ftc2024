package org.firstinspires.ftc.teamcode

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionPortal.StreamFormat
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

// in meters
const val CAMERA_X = 0.0
const val CAMERA_Y = 0.0
const val CAMERA_Z = 0.0

// in radians
const val CAMERA_YAW = 0.0
const val CAMERA_PITCH = 0.0
const val CAMERA_ROLL = 0.0

// in pixels
const val CAMERA_WIDTH = 1
const val CAMERA_HEIGHT = 1

// trade-off between speed and accuracy, this is default
const val DECIMATION = 3.0f

val CAMERA_POSITION = Position(
    DistanceUnit.METER,
    CAMERA_X,
    CAMERA_Y,
    CAMERA_Z,
    0
)
val CAMERA_ORIENTATION = YawPitchRollAngles(
    AngleUnit.RADIANS,
    CAMERA_YAW,
    CAMERA_PITCH,
    CAMERA_ROLL,
    0,
)

// these are reasonable defaults
fun makeAprilTagProcessor(): AprilTagProcessor =
    AprilTagProcessor.Builder()
        .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
        .setDrawAxes(false)
        .setDrawCubeProjection(true)
        .setDrawTagID(true)
        .setDrawTagOutline(false)
        .setNumThreads(AprilTagProcessor.THREADS_DEFAULT)
        .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
        .setSuppressCalibrationWarnings(false)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
        .build()

fun makeVisionPortal(webcam: CameraName, vararg processors: VisionProcessor): VisionPortal =
    VisionPortal.Builder()
        .addProcessors(*processors)
        .setAutoStartStreamOnBuild(false)
        .setAutoStopLiveView(false)
        .setCamera(webcam)
        .setCameraResolution(Size(CAMERA_WIDTH, CAMERA_HEIGHT))
        .setLiveViewContainerId(1)
        .setShowStatsOverlay(true)
        .setStreamFormat(StreamFormat.MJPEG)
        .build()

@TeleOp(name = "AprilTag Demo")
class AprilTagDemo : OpMode() {
    private val aprilTags: AprilTagProcessor = makeAprilTagProcessor()
    private lateinit var vision: VisionPortal

    override fun init() {
        aprilTags.setDecimation(DECIMATION)

        val camera = hardwareMap.get(CameraName::class.java, "front_camera")
        vision = makeVisionPortal(camera, aprilTags)
        vision.resumeStreaming()
        vision.resumeLiveView()
    }

    override fun loop() {
        val tags = aprilTags.detections
        telemetry.addLine("----- BEGIN TAGS -----")
        telemetry.addLine("found ${tags.count()} tags")
        for (tag in tags) {
            telemetry.addLine("----- BEGIN TAG ${tag.id} -----")
            telemetry.addLine("range (m): ${tag.ftcPose.range}")
            telemetry.addLine("bearing (rad): ${tag.ftcPose.bearing}")
            telemetry.addLine("elevation (rad): ${tag.ftcPose.elevation}")
            telemetry.addLine("----- END TAG ${tag.id} -----")
        }
        telemetry.addLine("----- END TAGS -----")
    }
}