package org.sert2521.reefscape2025.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator

class RemoveAlgae : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Elevator)
    }

    override fun initialize() {}

    override fun execute() {
        Elevator.setVoltage(0.15)
    }

    override fun isFinished(): Boolean {
        return Elevator.getPosition() < 0.15
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_STOW).schedule()
        }
    }
}
