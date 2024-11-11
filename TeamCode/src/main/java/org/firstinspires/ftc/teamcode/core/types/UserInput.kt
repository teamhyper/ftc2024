package org.firstinspires.ftc.teamcode.core.types

data class GamepadState(
    val leftStickX: Double,
    val leftStickY: Double,
    val rightStickX: Double,
    val rightStickY: Double,
    val dPadUp: Boolean,
    val dPadDown: Boolean,
    val dPadLeft: Boolean,
    val dPadRight: Boolean,
    val a: Boolean,
    val b: Boolean,
    val x: Boolean,
    val y: Boolean,
    val start: Boolean,
    val back: Boolean,
    val leftBumper: Boolean,
    val rightBumper: Boolean,
    val leftStickButton: Boolean,
    val rightStickButton: Boolean,
    val leftTrigger: Double,
    val rightTrigger: Double,
)

data class UserInput(
    val gamepad1: GamepadState,
    val gamepad2: GamepadState,
)
