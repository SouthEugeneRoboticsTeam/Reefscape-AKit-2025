package org.sert2521.reefscape2025.commands.dispenser

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

class DispenserReverse : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements()
    }

    override fun initialize() {
        Dispenser.setMotor(-0.4)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        Dispenser.stop()
    }
}
