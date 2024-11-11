package org.firstinspires.ftc.teamcode.core.math

import kotlin.math.max
import kotlin.math.min

interface SISOController {
    fun follow(target: Double, measured: Double): Double
    fun reset()
}

fun pid(
    kp: Double = 0.0,
    ki: Double = 0.0,
    kd: Double = 0.0,
    maxAbsIntegral: Double = Double.POSITIVE_INFINITY,
) = object : SISOController {
    init {
        if (maxAbsIntegral.isNaN()) throw IllegalArgumentException()
        if (maxAbsIntegral <= 0.0) throw IllegalArgumentException()
    }
    var errorIntegral = 0.0
    var errorDelay = 0.0
    var firstTick = true

    override fun follow(target: Double, measured: Double): Double {
        val error = target - measured

        errorIntegral += error
        errorIntegral = min(errorIntegral, maxAbsIntegral)
        errorIntegral = max(errorIntegral, -maxAbsIntegral)

        val errorDeriv = if (firstTick) 0.0 else error - errorDelay
        errorDelay = error
        firstTick = false

        return kp * error + ki * errorIntegral + kd * errorDeriv
    }

    override fun reset() {
        errorIntegral = 0.0
        errorDelay = 0.0
        firstTick = true
    }
}