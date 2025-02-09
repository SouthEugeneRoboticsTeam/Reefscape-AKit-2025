package org.sert2521.reefscape2025.commands.wrist

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.TuningConstants
import org.sert2521.reefscape2025.subsystems.wrist.Wrist

class SetWrist(private val wristSetPoint:Double) : Command() {
    //Smooth motion stuff
    private val motorPID = PIDController(TuningConstants.WRIST_P, TuningConstants.WRIST_I, TuningConstants.WRIST_D)
    private val feedforward = ArmFeedforward(TuningConstants.WRIST_S, TuningConstants.WRIST_G,
        TuningConstants.WRIST_V, TuningConstants.WRIST_A)
    private var wristAngle = Wrist.getRadians()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Wrist)
    }

    override fun initialize() {
        motorPID.setTolerance(0.05)
        motorPID.reset()
        Wrist.goal = wristSetPoint
    }

    override fun execute() {
        wristAngle = Wrist.getRadians()

        Wrist.setVoltage(motorPID.calculate(wristAngle, wristSetPoint) + feedforward.calculate(wristAngle, 0.0))
    }

    override fun isFinished(): Boolean {
        return motorPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        Wrist.stop()
    }
}
