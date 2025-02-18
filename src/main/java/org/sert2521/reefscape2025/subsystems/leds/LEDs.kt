package org.sert2521.reefscape2025.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

object LEDs: SubsystemBase() {

    private val io = LEDsIOSomething()
    private val ioInputs = LoggedLEDsIOInputs()

    init{
        defaultCommand = offCommand()
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("LEDs", ioInputs)
        io.ledStrip.setData(io.ledBuffer)
    }

    fun setAll(color: Color) {
        var fillColor = LEDPattern.solid(color)
        fillColor.applyTo(io.ledBuffer)
    }

    fun ledCurveCalculate(percent: Double): Color {
        return Color(
            -(0.0255) * percent.pow(2) + 255,
            (0.0255) * percent.pow(2),
            0.0
        )

        /* Curves for Miku colors (If robot is named Miku):
        Red:    r = (-.0241) * percent.pow(2) + 255
        Green:  g =  (.0234) * percent.pow(2)
        Blue:   b =  (.0223) * percent.po2(2) */
    }

    fun off() {
        io.off()
    }

    fun offCommand(): Command {
        return run{
            off()
        }
    }

}