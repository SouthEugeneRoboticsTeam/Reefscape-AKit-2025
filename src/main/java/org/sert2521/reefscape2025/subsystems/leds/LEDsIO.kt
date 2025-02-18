package org.sert2521.reefscape2025.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLEDBuffer
import org.team9432.annotation.Logged
import edu.wpi.first.wpilibj.util.Color
import org.sert2521.reefscape2025.PhysicalConstants


interface LEDsIO {
    @Logged
    open class LEDsIOInputs {
        var buffer = AddressableLEDBuffer(PhysicalConstants.LED_STRIP_LENGTH)
    }

    fun updateInputs(inputs: LEDsIOInputs) {}

    fun setLED(int: Int, color: Color) {}

    fun setAll(color: Color) {}

    fun setRainbow() {}

    fun ledCurveCalculate(percent: Double) {}

    fun off() {}
}