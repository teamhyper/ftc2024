package org.firstinspires.ftc.teamcode.core.math

import org.jetbrains.kotlinx.multik.api.*
import org.jetbrains.kotlinx.multik.api.linalg.*
import org.jetbrains.kotlinx.multik.ndarray.data.*
import org.jetbrains.kotlinx.multik.ndarray.operations.*

sealed class EKFVar() {
    data class StateVar(val name: Int): EKFVar()
    data class NoiseVar(val name: Int): EKFVar()
}

typealias EKFExpr = Expr<EKFVar>

interface Noise {
    fun noise(): EKFExpr
}

interface SetVar {
    operator fun set(i: Int, expr: EKFExpr)
}

interface GetVar {
    operator fun get(i: Int): EKFExpr
}

interface Measured {
    operator fun set(expr: EKFExpr, value: Double)
}

interface InitEKF: Noise {
    val init: SetVar
}

interface PredictEKF: Noise {
    val new: SetVar
    val old: GetVar
}

interface MeasureEKF: Noise {
    val state: GetVar
    val measured: Measured
}

interface Mean {
    operator fun get(i: Int): Double
}

interface Variance {
    operator fun get(i: Int): Double
    operator fun get(i: Int, j: Int): Double
}

interface Estimate {
    val mean: Mean
    val variance: Variance
}

interface EKF {
    fun predict(model: PredictEKF.() -> Unit)
    fun measure(model: MeasureEKF.() -> Unit)
    val estimate: Estimate
}

fun ekf(init: InitEKF.() -> Unit) = object : EKF {
    val dimension: Int
    var stateMean: D1Array<Double>
    var stateVar: D2Array<Double>
    init {
        /*
         * First, run the user-supplied initializer to figure out how many state
         * variables there are.
         */
        val equations = mutableMapOf<Int, EKFExpr>()
        var noiseCount = 0
        object : InitEKF {
            override val init = object : SetVar {
                override fun set(i: Int, expr: EKFExpr) {
                    if (i < 0) throw IndexOutOfBoundsException()
                    equations[i] = expr
                }
            }
            override fun noise(): EKFExpr =
                variable(EKFVar.NoiseVar(noiseCount++), 0.0)
        }.init()

        /* Now we can use that to allocate our state. */
        dimension = equations.size
        stateMean = mk.zeros(dimension)
        stateVar = mk.zeros(dimension, dimension)

        /* And update the state with default values. */
        predict {
            for (i in 0..<dimension) {
                new[i] = equations[i] ?: constant(0.0)
            }
        }
    }

    override fun predict(model: PredictEKF.() -> Unit) {
        /* Collect information from the user-supplied model. */
        val equations = mutableMapOf<Int, EKFExpr>()
        var noiseCount = 0
        object : PredictEKF {
            override val new = object : SetVar {
                override fun set(i: Int, expr: EKFExpr) {
                    if (i < 0) throw IndexOutOfBoundsException()
                    if (i >= dimension) throw IndexOutOfBoundsException()
                    equations[i] = expr
                }
            }
            override val old = object : GetVar {
                override fun get(i: Int): EKFExpr {
                    if (i < 0) throw IndexOutOfBoundsException()
                    if (i >= dimension) throw IndexOutOfBoundsException()
                    return variable(EKFVar.StateVar(i), stateMean[i])
                }
            }
            override fun noise() : EKFExpr =
                variable(EKFVar.NoiseVar(noiseCount++), 0.0)
        }.model()

        /* Update the mean estimate for those that changed. */
        for ((i, expr) in equations) {
            stateMean[i] = expr.nominal
        }

        /* Also update the covariance matrix. */
        val f = mk.d2arrayIndices(dimension, dimension) { i, j ->
            val eqn = equations[i]
            if (eqn == null) {
                if (i == j) 1.0 else 0.0
            } else {
                eqn.partials[EKFVar.StateVar(j)] ?: 0.0
            }
        }
        val l = mk.d2arrayIndices(dimension, noiseCount) { i, j ->
            val eqn = equations[i]
            if (eqn == null) {
                if (i == j) 1.0 else 0.0
            } else {
                eqn.partials[EKFVar.NoiseVar(j)] ?: 0.0
            }
        }
        stateVar = f dot stateVar dot f.transpose() + l dot l.transpose()
    }

    override fun measure(model: MeasureEKF.() -> Unit) {
        /* Collect the list of measured expressions and values. */
        val measurements = mutableListOf<Pair<EKFExpr, Double>>()
        var noiseCount = 0
        object : MeasureEKF {
            override val state = object : GetVar {
                override fun get(i: Int): EKFExpr {
                    if (i < 0) throw IndexOutOfBoundsException()
                    if (i >= dimension) throw IndexOutOfBoundsException()
                    return variable(EKFVar.StateVar(i), stateMean[i])
                }
            }
            override val measured = object : Measured {
                override fun set(expr: EKFExpr, value: Double) {
                    measurements += Pair(expr, value)
                }
            }
            override fun noise(): EKFExpr =
                variable(EKFVar.NoiseVar(noiseCount++), 0.0)
        }.model()

        /* Pack the data into matrices. */
        val e = mk.d1array(measurements.size) { i ->
            measurements[i].second - measurements[i].first.nominal
        }
        val h = mk.d2arrayIndices(measurements.size, dimension) { i, j ->
            measurements[i].first.partials[EKFVar.StateVar(j)] ?: 0.0
        }
        val m = mk.d2arrayIndices(measurements.size, noiseCount) { i, j ->
            measurements[i].first.partials[EKFVar.NoiseVar(j)] ?: 0.0
        }

        /* Perform the measurement update. */
        val v = (h dot stateVar dot h.transpose()) + (m dot m.transpose())
        val k = stateVar dot h.transpose() dot mk.linalg.inv(v)
        stateMean = stateMean + (k dot e)
        stateVar = (mk.identity<Double>(dimension) - (k dot h)) dot stateVar
    }

    override val estimate = object : Estimate {
        override val mean = object : Mean {
            override fun get(i: Int) = stateMean[i]
        }
        override val variance = object : Variance {
            override fun get(i: Int) = stateVar[i, i]
            override fun get(i: Int, j: Int) = stateVar[i, j]
        }
    }
}