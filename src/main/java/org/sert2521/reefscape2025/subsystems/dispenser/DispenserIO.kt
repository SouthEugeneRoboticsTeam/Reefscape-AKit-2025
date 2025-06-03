package org.sert2521.reefscape2025.subsystems.dispenser

import org.team9432.annotation.Logged

interface DispenserIO {

    @Logged
    open class DispenserIOInputs {
        var appliedVolts = 0.0
        var currentAmps = 0.0
        var velocityRPM = 0.0

        var beambreakDispenserClear = true
        var beambreakRampClear = true
    }

    fun updateInputs(inputs: DispenserIOInputs) {}

    fun setMotor(speed: Double) {}

    fun setVoltage(voltage: Double) {}
}