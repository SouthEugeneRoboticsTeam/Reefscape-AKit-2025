package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.trajectory.TrapezoidProfile
import org.team9432.annotation.Logged

interface ElevatorIO {

    @Logged
    open class ElevatorIOInputs{
        var appliedVolts = 0.0
        var currentAmps = 0.0

        var motorsPosition = 0.0
        var motorsVelocity = 0.0

        var laserPosition = 0.0
    }

    fun updateInputs(inputs:ElevatorIOInputs)

    fun setVoltage(voltage:Double)

    fun setReference(setpointPosition:Double, setpointVelocity:Double)

}