/*
 * This file implements mecanum drive.  It also aims to be an example of how to
 * program FTC robots in Kotlin, so it is written in a literate style, intended
 * to be read top to bottom.
 *
 * Full disclosure: I learned this language a few hours ago.  For what it's
 * worth, my first impression is that it is a much more friendly and fun version
 * of Java, that integrates very easily with existing Java code.
 *
 * Every file starts with a package declaration.  This matches the path to where
 * the file is stored.
 */

package org.firstinspires.ftc.teamcode.samples

/*
 * Later, we will refer to concepts from the FIRST-provided libraries.  Kotlin
 * requires us to list any names we want to import from other files up front, so
 * we list them here.  These will be explained when we get to them.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.min

/*
 * A mecanum drive train has four wheels with rollers.  They look like an "X"
 * pattern when viewed from above.
 *
 * front \\            // front
 * left  \\-+--------+-// right
 *       \\ |        | //
 *          | ^    ^ |
 *          |   __   |
 * back  // |        | \\ back
 * left  //-+--------+-\\ right
 *       //            \\
 *
 * Each wheel is connected to a motor that can run in either direction, at
 * variable speed.  Our program can set the power sent to each motor to a number
 * in the range [-1, 1].  Sending 1 will cause the motor to run forward at full
 * power, and sending -1 will make it run backward.  Similarly, 0 will make it
 * stop, and 0.5 will make it run at half power.
 *
 * Our program's output is four numbers, representing the desired speed of each
 * wheel.  We define a new type, WheelSpeeds, which holds all this information.
 */

private data class WheelSpeeds(
    val frontLeft: Double,
    val frontRight: Double,
    val backLeft: Double,
    val backRight: Double,
)

/*
 * Here's a quick translation of what each of these words means, in order of
 * importance:
 *
 * - `data class`: We're defining a new type of data.
 * - `WheelSpeeds`: This is the name of the new type.
 * - `val`: Short for value.  A `WheelSpeeds` object should contain four values,
 *   one for each wheel, so we repeat it four times.
 * - `frontLeft`, `frontRight`, etc.: These are the names of the four
 *   properties.
 * - `Double`: Short for "double-precision floating-point number".  This is the
 *   go-to type for representing (approximate) real numbers.  They are accurate
 *   to about 16 significant digits, which is more than enough for our purposes.
 * - `private`: We don't intend for code outside this file to use this type.
 *
 * With mecanum wheels, the robot effectively has three degrees of freedom: it
 * can move forward and back (longitudinal), move side-to-side (transverse), and
 * rotate in place (yaw).  We define another type to hold the drive speeds in
 * this format.
 */

private data class DriveSpeeds(
    val longitudinal: Double,
    val transverse: Double,
    val yaw: Double,
)

/*
 * We'll use the convention that these numbers are also in the range [-1, 1],
 * with 0 representing no motion.  The direction that 1 means is shown in the
 * picture below:
 *
 * longitudinal=1                      <-----
 *       ^                                   \
 *       |                           | yaw=1 |
 *  <----+----> transverse=1         \
 *       |                            ----->
 *       v
 *
 * Now we define a function which takes DriveSpeeds as input, and produces
 * WheelSpeeds as output.  Each wheel speed is calculated by adding or
 * subtracting the components of the drive speed.  To figure out where the
 * plus and minus signs go, use the following picture.  It shows arrows in the
 * direction that each wheel pushes the robot when it is driven forwards.
 *
 *  front ↗️        ↖️ front
 *  left  -+--------+- right
 *         |        |
 *         | ^    ^ |
 *         |   __   |
 * back ↖️ |        | ↗️ back
 * left   -+--------+-   right
 */

private fun driveSpeedsToWheelSpeeds(ds: DriveSpeeds): WheelSpeeds =
    WheelSpeeds(
        frontLeft = ds.longitudinal + ds.transverse - ds.yaw,
        frontRight = ds.longitudinal - ds.transverse + ds.yaw,
        backLeft = ds.longitudinal - ds.transverse - ds.yaw,
        backRight = ds.longitudinal + ds.transverse + ds.yaw,
    )

/*
 * We use the `fun` keyword to define a new function, and again we declare it
 * `private`, since we don't expect to use it outside this file.  The syntax
 * `ds: DriveSpeeds` means that the function takes a single input, of type
 * `DriveSpeeds`, and that in the body of the function, we'll call the input
 * `ds`.  The following `: WheelSpeeds` says that our functions output is of
 * type `WheelSpeeds`.  Then comes `=`, and we give the formula for computing
 * the output.
 *
 * Now, if you've been paying really close attention, you'll notice there's a
 * problem with these formulas.  Each of the DriveSpeed components is in the
 * range [-1, 1], and each WheelSpeed is computed by adding 3 of them up.  The
 * possible range of outputs is then [-3, 3], but we said earlier that the
 * motors expect a number in [-1, 1].  If we used this as-is, the robot wouldn't
 * behave correctly (this would be particularly visible if you try to move in
 * all three degrees of freedom at once).
 *
 * There are several ways we could fix this.  We could just divide everything
 * by 3, and we'd have the numbers in [-1, 1] again.  Unfortunately, that would
 * mean that driving the robot forward at "full power" would only run each
 * motor at 1/3 power.  Instead, we'll start by calculating the "raw" speeds as
 * above, and then "normalize" them, by scaling them down only as much as
 * necessary.  This process of normalization takes raw wheel speeds (any real
 * numbers) as input, and produces wheel speeds in the range [-1, 1] as output.
 * We take care to scale each component by the same amount, so that we don't
 * change what direction the robot is moving, only its speed.
 */

private fun normalizeWheelSpeeds(ws: WheelSpeeds): WheelSpeeds {
    val maxSpeed = maxOf(
        abs(ws.frontLeft),
        abs(ws.frontRight),
        abs(ws.backLeft),
        abs(ws.backRight),
    )
    /* We only want to scale down, not up! */
    val scale = min(1.0 / maxSpeed, 1.0)
    return WheelSpeeds(
        frontLeft = ws.frontLeft * scale,
        frontRight = ws.frontRight * scale,
        backLeft = ws.backLeft * scale,
        backRight = ws.backRight * scale,
    )
}

/*
 * Since this is a longer function, notice that we don't write "=", but instead
 * write curly braces, and write a list of statements.  We use `val` again to
 * define temporary values in the middle of our computation.  To signal we're
 * done, we say "return", and provide the output value.
 *
 * Next, we specify how the driver inputs the desired speed.  This is done using
 * a pair of joysticks.  Each joystick can move in two axes, x and y.
 */

private data class Joystick(
    val x: Double,
    val y: Double,
)

private data class Input(
    val left: Joystick,
    val right: Joystick,
)

/*
 * The convention that our joysticks use is that x and y are numbers in the
 * range [-1, 1], with 0 being in the center.  The directions look like this:
 *
 *       ^
 *       |
 *  <----+----> x=1
 *       |
 *       v
 *      y=1
 *
 * We define another function that converts driver inputs into our DriveSpeed
 * type from earlier.  Notice that y=1 should drive the robot backwards, and
 * y=-1 should drive it forwards, so we need a minus sign to convert from y to
 * longitudinal.
 */

private fun inputToDriveSpeeds(input: Input): DriveSpeeds =
    DriveSpeeds(
        longitudinal = -input.left.y,
        transverse = input.left.x,
        yaw = -input.right.x,
    )

/*
 * Next, we need to specify how to read our inputs and write out outputs.  The
 * FTC library provides a Gamepad type, which holds the state of all buttons
 * and joysticks on a single game pad.  We define a function that takes a
 * Gamepad as input, and packs the relevant data into our Input type.
 */

private fun readInput(gamepad: Gamepad): Input =
    Input(
        left = Joystick(
            x = gamepad.left_stick_x.toDouble(),
            y = gamepad.left_stick_y.toDouble(),
        ),
        right = Joystick(
            x = gamepad.right_stick_x.toDouble(),
            y = gamepad.right_stick_y.toDouble(),
        ),
    )

/*
 * The FTC library also provides a type `DcMotor`, for each motor on the robot.
 * Each motor has a `power` property, which we can set to change the current
 * speed.
 */

private fun writeWheelSpeeds(
    ws: WheelSpeeds,
    frontLeft: DcMotor,
    frontRight: DcMotor,
    backLeft: DcMotor,
    backRight: DcMotor
) {
    frontLeft.power = ws.frontLeft
    frontRight.power = ws.frontRight
    backLeft.power = ws.backLeft
    backRight.power = ws.backRight
}

/*
 * Finally, we are ready to assemble these components together into a complete
 * program.  The top-level thing we are providing to the robot is an `OpMode`,
 * or "mode of operation".
 */

@TeleOp(name = "Drive Test")
@Disabled /* remove this line to enable! */
class Drive : OpMode() {

    /*
     * This defines a new type, called `Drive`.  Unlike our previous types,
     * which used `data class`, this uses just `class`.  This is because we are
     * not defining "plain old data", but rather something that includes a set
     * of behaviours.  The `: OpMode()` syntax indicates that we are inheriting
     * all the features of the type `OpMode`, and extending it with our own
     * additions.  The `@TeleOp(name = ...)` is an annotation.  It doesn't do
     * anything on it's own, but the FTC library code scans for classes with
     * these annotations when it displays a menu for the operator to choose
     * from.
     *
     * Inside this type, we're going to define two functions: init() and loop().
     * init() is called once on startup, and is where we're going to get our
     * hands on any hardware and sensors we want to use, and configure them.
     * loop() is called periodically while the robot is running, and is where
     * we're going to read our inputs, do our calculations, and then write our
     * outputs.
     *
     * Since we're going to get handles to the motors in one function but then
     * write to them in another, we need a place to store those handles.  We add
     * some private properties to Drive, for our own internal use.
     */

    private lateinit var frontLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var backRight: DcMotor

    private lateinit var vision: VisionPortal
    private lateinit var aprilTags: AprilTagProcessor

    /*
     * Here we see two new words.  `var` stands for variable, and is a bit like
     * val, except we are allowed to change it after we declare it.  I never
     * mentioned it before, but `val` values are immutable: they cannot be
     * changed once they have been set.  To prevent programmer confusion, it's
     * best to only declare things `var` if they actually need to change over
     * time.  The `lateinit` keyword is less interesting.  It means that we
     * don't have a value to assign just yet.  One will come later, when our
     * init() method is called by the FTC system.
     *
     * Now we can fill in init().  Since init() is a function that OpMode
     * (part of the FTC libraries) declared, and we want to fill it in with our
     * own definition, we say we are overriding it.
     */

    override fun init() {
        /*
         * Every OpMode comes with a property called hardwareMap.  This is where
         * we get handles to motors and sensors.  It's also where we configure
         * settings for hardware.  In particular, some motors may be wired
         * backwards, so we flip them around here.  We also set it so the robot
         * breaks, instead of coasting, when we set its power to 0.
         */
        frontLeft = hardwareMap.get(DcMotor::class.java, "front_left")
        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        frontRight = hardwareMap.get(DcMotor::class.java, "front_right")
        frontRight.direction = DcMotorSimple.Direction.FORWARD
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backLeft = hardwareMap.get(DcMotor::class.java, "back_left")
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backRight = hardwareMap.get(DcMotor::class.java, "back_right")
        backRight.direction = DcMotorSimple.Direction.FORWARD
        backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val camera = hardwareMap.get(CameraName::class.java, "front_camera")

        aprilTags = AprilTagProcessor.Builder()
            .setDrawCubeProjection(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
            .build()

        vision = VisionPortal.Builder()
            .setCamera(camera)
            .addProcessors(aprilTags)
            .build()

        vision.resumeStreaming()
    }

    /*
     * Last, we fill in loop() with a combination of all our hard work from
     * earlier.
     */

    override fun loop() {
        /* Read our input.  `gamepad1` is provided by OpMode. */
        val input = readInput(gamepad1)

        /* Use our inputs to calculate outputs. */
        val driveSpeeds = inputToDriveSpeeds(input)
        val rawWheelSpeeds = driveSpeedsToWheelSpeeds(driveSpeeds)
        val wheelSpeeds = normalizeWheelSpeeds(rawWheelSpeeds)

        /* Write our output to the motors we got in init(). */
        writeWheelSpeeds(
            wheelSpeeds,
            frontLeft = frontLeft,
            frontRight = frontRight,
            backLeft = backLeft,
            backRight = backRight
        )

        /*
         * Write some diagnostic information to the screen, so we can understand
         * what's going on if we run into issues.
         */
        telemetry.addData("input", input)
        telemetry.addData("drive speeds", driveSpeeds)
        telemetry.addData("raw wheel speeds", rawWheelSpeeds)
        telemetry.addData("wheel speeds", wheelSpeeds)

        val leftPosition = frontLeft.currentPosition
        val rightPosition = -frontRight.currentPosition
        val centerPosition = backRight.currentPosition

        telemetry.addData("left encoder", leftPosition)
        telemetry.addData("right encoder", rightPosition)
        telemetry.addData("center encoder", centerPosition)

        telemetry.addData("motor mode", frontLeft.mode)
    }
}
