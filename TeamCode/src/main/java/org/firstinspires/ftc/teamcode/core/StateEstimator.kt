package org.firstinspires.ftc.teamcode.core

import org.firstinspires.ftc.teamcode.core.types.Control
import org.firstinspires.ftc.teamcode.core.types.Measurement
import org.firstinspires.ftc.teamcode.core.types.StateEstimate

interface StateEstimator {
    /**
     * Update the state estimate based on new measurements.
     */
    fun update(measurement: Measurement)

    /**
     * Predict the future state for the next time step.
     */
    fun predict(control: Control)

    /**
     * Our current best estimate of the state.
     */
    val estimate: StateEstimate
}

