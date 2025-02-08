package org.sert2521.reefscape2025.commands.dispenser

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.SetpointConstants.DISPENSER_INTAKE_SPEED
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser

class DispenserIntake : Command() {

    private var triggered = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Dispenser)
    }

    override fun initialize() {
        Dispenser.setMotor(DISPENSER_INTAKE_SPEED)
    }

    override fun execute() {
        if (Dispenser.getBlocked()){
            triggered = true
        }
    }

    override fun isFinished(): Boolean {
        return !Dispenser.getBlocked() && triggered
    }

    override fun end(interrupted: Boolean) {
        Dispenser.stop()
    }
}
