package org.sert2521.reefscape2025.subsystems.elevator

import org.team9432.annotation.Logged

interface ElevatorIO {

    @Logged
    open class ElevatorIOInputs {
        var appliedVolts = 0.0
        var currentAmps = 0.0

        var positionMeters = 0.0
        var velocityMetersPerSec = 0.0
    }

    fun updateInputs(inputs: ElevatorIOInputs) {}

    fun setVoltage(voltage: Double) {}

    fun setReference(setpointPosition: Double, setpointVelocity: Double) {}

}