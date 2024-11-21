package org.firstinspires.ftc.teamcode.core.estimator

import org.firstinspires.ftc.teamcode.core.LIFT_METERS_PER_TICK
import org.firstinspires.ftc.teamcode.core.types.LiftState

interface LiftEstimator {
    val state: LiftState
    fun measure(
        leftEncoderTicks: Double,
        rightEncoderTicks: Double,
        isHome: Boolean,
    )
}

sealed class LiftStateInternal {
    object Init : LiftStateInternal()
    data class Running(
        val homeTicks: Double,
        val currentTicks: Double,
    ) : LiftStateInternal()
}

fun liftEstimator() = object : LiftEstimator {
    var stateInternal: LiftStateInternal = LiftStateInternal.Init

    override val state: LiftState get() {
        val state = stateInternal
        when (state) {
            is LiftStateInternal.Init -> return LiftState.Unknown
            is LiftStateInternal.Running -> {
                val heightTicks = state.currentTicks - state.homeTicks
                val heightMeters = LIFT_METERS_PER_TICK * heightTicks
                return LiftState.AtHeight(heightMeters)
            }
        }
    }

    override fun measure(
        leftEncoderTicks: Double,
        rightEncoderTicks: Double,
        isHome: Boolean,
    ) {
        val currentTicks = (leftEncoderTicks + rightEncoderTicks) / 2.0
        if (isHome) {
            stateInternal = LiftStateInternal.Running(
                homeTicks = currentTicks,
                currentTicks = currentTicks,
            )
        } else {
            val old = stateInternal
            if (old is LiftStateInternal.Running) {
                stateInternal = LiftStateInternal.Running(
                    homeTicks = old.homeTicks,
                    currentTicks = currentTicks,
                )
            }
        }
    }
}