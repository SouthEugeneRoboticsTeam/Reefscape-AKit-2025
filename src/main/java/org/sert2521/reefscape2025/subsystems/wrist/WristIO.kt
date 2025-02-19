package org.sert2521.reefscape2025.subsystems.wrist

import org.team9432.annotation.Logged

interface WristIO {

    @Logged
    open class WristIOInputs{
        var wristAppliedVolts = 0.0
        var wristCurrentAmps = 0.0
        var wristVelocityRadPerSec = 0.0
        var wristPosition = 0.0
    }

    fun updateInputs(inputs:WristIOInputs)

    fun setVoltage(voltage:Double)

    fun setReference(targetPosition:Double)
}