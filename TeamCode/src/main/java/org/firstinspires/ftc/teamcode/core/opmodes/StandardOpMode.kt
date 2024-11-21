package org.firstinspires.ftc.teamcode.core.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.core.CYCLE_PERIOD_SECONDS
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.Hardware
import org.firstinspires.ftc.teamcode.core.Logger
import org.firstinspires.ftc.teamcode.core.Planner
import org.firstinspires.ftc.teamcode.core.controller
import org.firstinspires.ftc.teamcode.core.estimator.StateEstimator
import org.firstinspires.ftc.teamcode.core.estimator.stateEstimator
import org.firstinspires.ftc.teamcode.core.ext.userInput
import org.firstinspires.ftc.teamcode.core.hardware
import org.firstinspires.ftc.teamcode.core.logger

abstract class StandardOpMode: LinearOpMode() {
    open fun createStateEstimator(): StateEstimator = stateEstimator()
    abstract fun createPlanner(): Planner
    open fun createController(): Controller = controller()
    open fun createHardware(): Hardware = hardware(hardwareMap)
    open fun createLogger(): Logger = logger(telemetry)

    override fun runOpMode() {
        val hw = createHardware()
        val stateEstimator = createStateEstimator()
        val planner = createPlanner()
        val controller = createController()
        val cycleTimer = ElapsedTime()
        createLogger().use { logger ->
            waitForStart()
            while (opModeIsActive()) {
                cycleTimer.reset()

                /* Measure sensors. */
                val measurement = hw.measure()

                /* Perform measurement update on state estimate. */
                val oldState = stateEstimator.estimate
                stateEstimator.measure(measurement)
                val state = stateEstimator.estimate

                /* Compute actuator output. */
                val command = planner.process(userInput, state)
                val control = controller.follow(command, state)

                /* Send outputs to actuators. */
                hw.control(control)

                /* Perform time update on state estimate. */
                stateEstimator.predict(control)

                /* Update telemetry and data logs. */
                logger.logMeasurement(measurement)
                logger.logPrior(oldState)
                logger.logPosterior(state)
                logger.logCommand(command)
                logger.logControl(control)
                logger.flush()

                /* Sleep until next cycle. */
                while (cycleTimer.seconds() < CYCLE_PERIOD_SECONDS) sleep(1)
            }
        }
    }
}