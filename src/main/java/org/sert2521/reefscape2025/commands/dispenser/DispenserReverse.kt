package org.sert2521.reefscape2025.commands.dispenser

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_INTAKE_SPEED
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

class DispenserReverse : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Dispenser)
    }

    override fun initialize() {
        Dispenser.setMotor(-DISPENSER_INTAKE_SPEED)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        Dispenser.stop()
    }
}
