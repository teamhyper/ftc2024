package org.firstinspires.ftc.teamcode.core.math

sealed class EKFVar() {
    class StateVar(val name: Int): EKFVar()
    class NoiseVar(val name: Int): EKFVar()
}

typealias EKFExpr = Expr<EKFVar>

interface Noise {
    fun noise(sd: Double): EKFExpr
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
    init {
        object : InitEKF {
            override val init = object : SetVar {
                override fun set(i: Int, expr: EKFExpr) {
                    TODO("Not yet implemented")
                }
            }
            override fun noise(sd: Double): EKFExpr {
                TODO("Not yet implemented")
            }
        }.init()
    }

    override fun predict(model: PredictEKF.() -> Unit) {
        object : PredictEKF {
            override val new = object : SetVar {
                override fun set(i: Int, expr: EKFExpr) {
                    TODO()
                }
            }
            override val old = object : GetVar {
                override fun get(i: Int): EKFExpr {
                    TODO()
                }
            }
            override fun noise(sd: Double) : EKFExpr {
                TODO()
            }
        }.model()
    }

    override fun measure(model: MeasureEKF.() -> Unit) {
        object : MeasureEKF {
            override val state = object : GetVar {
                override fun get(i: Int) : EKFExpr {
                    TODO()
                }
            }
            override val measured = object : Measured {
                override fun set(expr: EKFExpr, value: Double) {
                    TODO("Not yet implemented")
                }

            }
            override fun noise(sd: Double): EKFExpr {
                TODO("Not yet implemented")
            }
        }
    }

    override val estimate = object : Estimate {
        override val mean = object : Mean {
            override fun get(i: Int): Double {
                TODO("Not yet implemented")
            }
        }
        override val variance = object : Variance {
            override fun get(i: Int): Double {
                TODO("Not yet implemented")
            }
            override fun get(i: Int, j: Int): Double {
                TODO("Not yet implemented")
            }
        }
    }
}