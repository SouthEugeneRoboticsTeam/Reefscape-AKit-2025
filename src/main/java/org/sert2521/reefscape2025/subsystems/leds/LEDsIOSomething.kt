package org.sert2521.reefscape2025.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import org.sert2521.reefscape2025.ElectronicIDs
import org.sert2521.reefscape2025.PhysicalConstants

class LEDsIOSomething: LEDsIO {

    val ledStrip = AddressableLED(ElectronicIDs.LED_PORT)
    val ledBuffer = AddressableLEDBuffer(PhysicalConstants.LED_STRIP_LENGTH)

    init{}

    override fun updateInputs(inputs: LEDsIO.LEDsIOInputs) {
        inputs.buffer = ledBuffer
    }

    override fun off() {
        ledStrip.setData()
    }
}