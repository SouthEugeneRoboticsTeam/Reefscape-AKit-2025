package org.sert2521.reefscape2025.commands.grintake

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.subsystems.grintake.GroundIntake

class RunWrist(val wristSetPoint:Double) : Command() {
    private val groundIntake = GroundIntake

    //Smooth motion stuff
    private val motorPID = PIDController(TuningConstants.WRIST_P, TuningConstants.WRIST_I, TuningConstants.WRIST_D)
    private val feedforward = ArmFeedforward(TuningConstants.WRIST_S, TuningConstants.WRIST_G,
        TuningConstants.WRIST_V, TuningConstants.WRIST_A)
    private var wristAngle = GroundIntake.getWristRadians()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(groundIntake)

    }

    override fun initialize() {
        motorPID.setTolerance(0.05)
    }

    override fun execute() {
        wristAngle = groundIntake.getWristRadians()

        groundIntake.setWristVoltage(motorPID.calculate(wristAngle, wristSetPoint) + feedforward.calculate(wristAngle, 0.0))
    }

    override fun isFinished(): Boolean {
        return motorPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        groundIntake.stopWrist()
    }
}
