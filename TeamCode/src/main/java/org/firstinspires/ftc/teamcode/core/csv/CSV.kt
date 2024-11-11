package org.firstinspires.ftc.teamcode.core.csv

import java.io.Writer

/*
 * Super simple CSV writer.  Does not perform any quoting or escaping. This is
 * used by the logger.
 */

interface CSVInit<out Row> {
    fun column(name: String, value: (Row) -> Any)
}

interface CSV<in Row> {
    fun header(): String
    fun row(row: Row): String
}

fun<Row> Writer.writeCsvHeader(csv: CSV<Row>) = appendLine(csv.header())
fun<Row> Writer.writeCsvRow(csv: CSV<Row>, row: Row) = appendLine(csv.row(row))

fun<Row> csv(init: CSVInit<Row>.() -> Unit) = object : CSV<Row> {
    val names: List<String>
    val values: List<(Row) -> Any>
    init {
        val namesMut = mutableListOf<String>()
        val valuesMut = mutableListOf<(Row) -> Any>()
        object : CSVInit<Row> {
            override fun column(name: String, value: (Row) -> Any) {
                namesMut += name
                valuesMut += value
            }
        }.init()
        names = namesMut.toList()
        values = valuesMut.toList()
    }

    override fun header() = names.joinToString(",")
    override fun row(row: Row) = values.map { it(row) }.joinToString(",")
}