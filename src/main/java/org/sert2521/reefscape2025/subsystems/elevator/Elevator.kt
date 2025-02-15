package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.commands.elevator.HoldElevator

object Elevator : SubsystemBase() {
    private val io = ElevatorIOSpark()
    private val ioInputs = LoggedElevatorIOInputs()
    private val downFilter = Debouncer(0.4)

    var goal = SetpointConstants.ELEVATOR_STOW

    init{
        defaultCommand = HoldElevator()
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Elevator", ioInputs)

        if (downFilter.calculate(ioInputs.laserPosition<0.03)){
            io.setEncoder(0.0)
        }
    }

    fun setVoltage(voltage:Double){
        io.setVoltage(voltage)
    }

    fun getVelocity():Double {
        return ioInputs.laserVelocity
    }

    fun getPosition():Double{
        return ioInputs.motorPosition//ioInputs.laserPosition
    }

    fun getMotorPosition():Double{
        return ioInputs.motorPosition
    }

    fun stop() {
        io.setVoltage(0.0)
    }
}