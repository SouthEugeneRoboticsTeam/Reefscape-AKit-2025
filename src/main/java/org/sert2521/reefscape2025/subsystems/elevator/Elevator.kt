package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.reefscape2025.SetpointConstants

object Elevator : SubsystemBase() {
    private val io = ElevatorIOSpark()
    private val ioInputs = LoggedElevatorIOInputs()

    var goal = SetpointConstants.ELEVATOR_STOW

    override fun periodic() {
        io.updateInputs(ioInputs)
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun getVelocity():Double {
        return ioInputs.laserVelocity
    }

    fun getPosition():Double{
        return ioInputs.laserPosition
    }

    fun stop() {
        io.setVoltage(0.0)
    }
}