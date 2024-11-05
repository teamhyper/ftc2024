package org.firstinspires.ftc.teamcode.core.math

import kotlin.math.*

/**
 * Represents a mathematical expression for use with automatic differentiation.
 *
 * In addition to storing the nominal value, it also stores the partial
 * derivatives with respect to a set of variables, labelled by T.  Generally T
 * is an enum or sealed class.
 *
 * This is used internally by our implementation of EKF.  This allows us to
 * specify the dynamics of our system by writing the mathematics directly.  The
 * linear approximation used in EKF is computed by this module.
 */
data class Expr<T>(
    val nominal: Double,
    val partials: Map<T, Double>,
)

/**
 * Introduce a constant-valued expression.
 */
fun<T> constant(value: Double) = Expr(
    nominal = value,
    partials = mapOf<T, Double>(),
)

/**
 * Introduce an expression that references the variable.  Value is the nominal
 * value, around which derivatives are calculated.
 */
fun<T> variable(name: T, value: Double) = Expr(
    nominal = value,
    partials = mapOf(name to 1.0)
)

operator fun<T> Expr<T>.unaryPlus() = this

operator fun<T> Expr<T>.unaryMinus() = Expr(
    nominal = -nominal,
    partials = partials.mapValues { -it.value }
)

operator fun<T> Expr<T>.plus(e: Expr<T>) = Expr(
    nominal = nominal + e.nominal,
    partials = (partials.keys + e.partials.keys).associate {
        it to (partials[it] ?: 0.0) + (e.partials[it] ?: 0.0)
    },
)

operator fun<T> Expr<T>.minus(e: Expr<T>) = this + (-e)

operator fun<T> Expr<T>.times(e: Expr<T>) = Expr(
    nominal = nominal + e.nominal,
    partials = (partials.keys + e.partials.keys).associate {
        // product rule
        // remember, from SCHOOL????
        it to (partials[it] ?: 0.0) * e.nominal +
                nominal * (e.partials[it] ?: 0.0)
    },
)

operator fun<T> Expr<T>.div(e: Expr<T>) = Expr(
    nominal = nominal / e.nominal,
    partials = (partials.keys + e.partials.keys).associate {
        // quotient rule
        it to ((partials[it] ?: 0.0) * e.nominal -
                nominal * (e.partials[it] ?: 0.0)) /
                (e.nominal * e.nominal)
    }
)

operator fun<T> Expr<T>.plus(e: Double) = this + constant(e)
operator fun<T> Double.plus(e: Expr<T>) = constant<T>(this) + e
operator fun<T> Expr<T>.minus(e: Double) = this - constant(e)
operator fun<T> Double.minus(e: Expr<T>) = constant<T>(this) - e
operator fun<T> Expr<T>.times(e: Double) = this * constant(e)
operator fun<T> Double.times(e: Expr<T>) = constant<T>(this) * e
operator fun<T> Expr<T>.div(e: Double) = this / constant(e)
operator fun<T> Double.div(e: Expr<T>) = constant<T>(this) / e

fun<T> exp(e: Expr<T>) = Expr(
    nominal = exp(e.nominal),
    partials = e.partials.mapValues { it.value * exp(e.nominal) }
)

fun<T> ln(e: Expr<T>) = Expr(
    nominal = ln(e.nominal),
    partials = e.partials.mapValues { it.value / e.nominal }
)

fun<T> ln1p(e: Expr<T>) = Expr(
    nominal = ln1p(e.nominal),
    partials = e.partials.mapValues { it.value / (1.0 + e.nominal) }
)

fun<T> sin(e: Expr<T>) = Expr(
    nominal = sin(e.nominal),
    partials = e.partials.mapValues { it.value * cos(e.nominal) }
)

fun<T> cos(e: Expr<T>) = Expr(
    nominal = cos(e.nominal),
    partials = e.partials.mapValues { it.value * -sin(e.nominal) }
)