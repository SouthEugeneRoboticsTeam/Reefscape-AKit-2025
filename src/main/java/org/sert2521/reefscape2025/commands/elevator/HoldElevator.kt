package org.sert2521.reefscape2025.commands.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_G
import org.sert2521.reefscape2025.subsystems.elevator.Elevator

class HoldElevator : Command() {
    private val motorPID = PIDController(TuningConstants.ELEVATOR_P, TuningConstants.ELEVATOR_I, TuningConstants.ELEVATOR_D)

    private var elevatorSetPoint = Elevator.goal
    private var elevatorPosition = Elevator.getPosition()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Elevator)
        motorPID.setTolerance(0.05)
    }

    override fun initialize() {
        elevatorSetPoint = Elevator.getPosition()
    }

    override fun execute() {
        elevatorPosition = Elevator.getPosition()
        Elevator.setVoltage(motorPID.calculate(Elevator.getPosition(), elevatorSetPoint) + ELEVATOR_G)
    }
}
