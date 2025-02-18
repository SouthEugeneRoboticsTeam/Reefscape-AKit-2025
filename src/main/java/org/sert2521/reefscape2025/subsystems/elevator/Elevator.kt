package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_PROFILE
import org.sert2521.reefscape2025.commands.elevator.HoldElevator

object Elevator : SubsystemBase() {
    private val io = ElevatorIOSpark()
    private val ioInputs = LoggedElevatorIOInputs()
    private val downFilter = Debouncer(0.4)

    val rollingAverage = LinearFilter.movingAverage(4)
    var rollingAverageOutput = 0.0

    val profile = TrapezoidProfile(ELEVATOR_PROFILE)

    var goal  = TrapezoidProfile.State(0.0, 0.0)
    var currentState = TrapezoidProfile.State(0.0, 0.0)

    init{
        defaultCommand = holdElevatorCommand()
    }

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Elevator", ioInputs)

        rollingAverageOutput = rollingAverage.calculate(ioInputs.laserPosition*2 - 0.15)
        Logger.recordOutput("Elevator/Processed Laser", getPosition())

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
        return ioInputs.laserPosition-0.07
    }

    fun getMotorPosition():Double{
        return ioInputs.motorPosition
    }

    fun stop() {
        io.setVoltage(0.0)
    }

    fun setElevatorCommand(goalMeters:Double): Command {
        goal = TrapezoidProfile.State(goalMeters, 0.0)
        return run{
            currentState = profile.calculate(0.02, currentState, goal)
            io.setReference(currentState)

            Logger.recordOutput("Elevator/At Setpoint",
                MathUtil.isNear(currentState.position, getPosition(), 0.05))
        }.until{
            profile.isFinished(0.0)
        }
    }

    fun holdElevatorCommand():Command{
        return run{
            io.setReference(currentState)
            Logger.recordOutput("Elevator/At Setpoint",
                MathUtil.isNear(currentState.position, getPosition(), 0.05))
        }
    }
}