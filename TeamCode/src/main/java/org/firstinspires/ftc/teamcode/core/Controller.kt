package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.types.Command
import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.StateEstimate

fun interface Controller {
    /*
     * Decide what control signal to send to actuators.  This decision is based
     * on the command, as well as our current estimate of the state.  The
     * command consists of the desired position and velocity of the robot, as
     * well as feed-forward terms (the control signal that we expect to need
     * to use to follow that command).  The job of this function is to modify
     * this control signal to compensate for errors, as measured by the state
     * estimate.
     */
    fun follow(
        command: Command,
        state: StateEstimate,
    ): Control
}
