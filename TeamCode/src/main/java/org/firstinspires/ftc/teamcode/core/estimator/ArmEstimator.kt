package org.firstinspires.ftc.teamcode.core.estimator

import org.firstinspires.ftc.teamcode.core.ARM_RADS_PER_TICK

interface ArmEstimator {
    val angleRads: Double
    fun measure(encoderTicks: Double)
}

sealed class ArmStateInternal {
    object Init: ArmStateInternal()
    data class Running(
        val homeTicks: Double,
        val currentTicks: Double,
    ): ArmStateInternal()
}

fun armEstimator() = object : ArmEstimator {
    var state: ArmStateInternal = ArmStateInternal.Init

    override val angleRads: Double get() {
        val s = state
        when (s) {
            is ArmStateInternal.Init -> return 0.0
            is ArmStateInternal.Running -> {
                val angleTicks = s.currentTicks - s.homeTicks
                val angleRads = ARM_RADS_PER_TICK * angleTicks
                return angleRads
            }
        }
    }

    override fun measure(encoderTicks: Double) {
        val old = state
        state = when (old) {
            is ArmStateInternal.Init -> ArmStateInternal.Running(
                homeTicks = encoderTicks,
                currentTicks = encoderTicks,
            )
            is ArmStateInternal.Running -> ArmStateInternal.Running(
                homeTicks = old.homeTicks,
                currentTicks = encoderTicks,
            )
        }
    }
}