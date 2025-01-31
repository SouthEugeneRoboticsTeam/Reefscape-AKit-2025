package org.sert2521.reefscape2025.commands.grintake

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.subsystems.grintake.GroundIntake

class Intake : Command() {
    private val groundIntake = GroundIntake


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(groundIntake)
    }

    override fun initialize() {}

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
