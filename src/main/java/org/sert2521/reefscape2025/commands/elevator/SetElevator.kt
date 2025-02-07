package org.sert2521.reefscape2025.commands.elevator

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator

class SetElevator(val goal:Double) : Command() {
    private val elevator = Elevator
    private val motorPID = ProfiledPIDController(TuningConstants.ELEVATOR_P, TuningConstants.ELEVATOR_I, TuningConstants.ELEVATOR_D, TuningConstants.ELEVATOR_PROFILE)
    private val feedforward = ElevatorFeedforward(
        TuningConstants.ELEVATOR_S, TuningConstants.ELEVATOR_G,
        TuningConstants.ELEVATOR_V, TuningConstants.ELEVATOR_A)
    var elevatorPosition = elevator.getPosition()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(elevator)
        motorPID.setTolerance(0.05)
    }

    override fun initialize() {
    }

    override fun execute() {
        elevatorPosition = elevator.getPosition()
        elevator.setVoltage(motorPID.calculate(elevatorPosition, goal) + feedforward.calculate(motorPID.setpoint.velocity))
    }

    override fun isFinished(): Boolean {
        return motorPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        elevator.stop()
    }
}
