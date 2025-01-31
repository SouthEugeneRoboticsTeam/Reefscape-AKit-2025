package org.sert2521.reefscape2025.commands.grintake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.subsystems.grintake.GroundIntake

class RunWrist : Command() {
    private val groundIntake = GroundIntake
    private val motorPID = PIDController(TuningConstants.WRIST_P, TuningConstants.WRIST_I, TuningConstants.WRIST_D)
    private val feedforward = ArmFeedforward(TuningConstants.WRIST_S, TuningConstants.WRIST_G,
        TuningConstants.WRIST_V, TuningConstants.WRIST_A)
    private var wristAngle = GroundIntake.getWristRadians()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(groundIntake)
    }

    override fun initialize() {

    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
