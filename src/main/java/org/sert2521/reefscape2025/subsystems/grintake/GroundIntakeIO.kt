package org.sert2521.reefscape2025.subsystems.grintake


import org.team9432.annotation.Logged

interface GroundIntakeIO {
    @Logged
    open class GroundIntakeIOInputs {
        var appliedVolts = 0.0
        var currentAmps = 0.0
        var velocityRPM = 0.0
    }

    fun updateInputs(inputs: GroundIntakeIOInputs){}

    fun setMotor(speed:Double){}

    fun setVoltage(voltage:Double){}
}