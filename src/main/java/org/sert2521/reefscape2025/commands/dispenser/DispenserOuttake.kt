package org.sert2521.reefscape2025.commands.dispenser

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

class DispenserOuttake : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Dispenser)
    }

    override fun initialize() {
        Dispenser.setMotor(0.7)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Dispenser.stop()
    }
}
