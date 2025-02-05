package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.wpilibj2.command.SubsystemBase

object Elevator : SubsystemBase() {
    private val io = ElevatorIOSpark()
    private val ioInputs = LoggedElevatorIOInputs()

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
}