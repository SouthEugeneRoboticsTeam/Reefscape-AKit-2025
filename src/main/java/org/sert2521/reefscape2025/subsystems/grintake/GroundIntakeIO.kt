package org.sert2521.reefscape2025.subsystems.grintake


import org.team9432.annotation.Logged

interface GroundIntakeIO {
    @Logged
    open class GroundIntakeIOInputs {
        var intakeAppliedVolts = 0.0
        var intakeCurrentAmps = 0.0
        var intakeVelocityRPM = 0.0

        var wristAppliedVolts = 0.0
        var wristCurrentAmps = 0.0
        var wristVelocityRPM = 0.0
    }

    fun updateInputs(inputs: GroundIntakeIOInputs) {}

    fun setIntakeMotor(speed: Double) {}

    fun setIntakeVoltage(voltage: Double) {}

    fun setWristMotor(speed: Double) {}

    fun setWristVoltage(voltage: Double) {}
}