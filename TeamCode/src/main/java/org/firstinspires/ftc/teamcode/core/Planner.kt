package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.types.Command
import org.firstinspires.ftc.teamcode.core.types.StateEstimate
import org.firstinspires.ftc.teamcode.core.types.UserInput

fun interface Planner {
    /*
     * Decide what command the robot should currently be following, based on
     * user input and state estimates.  The output of this function contains
     * the desired position or velocity of the robot, as well as feed-forward
     * values for actuators.
     *
     * For simple op-modes, this may just set the command based on the current
     * value of the controller.  For more complex op-modes, it may plan a path
     * in advanced ant attempt to follow it.
     */
    fun process(
        input: UserInput,
        state: StateEstimate,
    ): Command
}