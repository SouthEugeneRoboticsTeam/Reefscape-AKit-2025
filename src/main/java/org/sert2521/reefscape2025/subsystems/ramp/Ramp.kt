package org.sert2521.reefscape2025.subsystems.ramp

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.SetpointConstants.RAMP_INTAKE_SPEED

object Ramp : SubsystemBase() {
    private val io = RampIOSpark()
    private val ioInputs = LoggedRampIOInputs()

    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Ramp", ioInputs)
    }

    private fun setSpeed(speed:Double){ io.setSpeed(speed) }

    fun intakeCommand(): Command {
        return run{
            setSpeed(RAMP_INTAKE_SPEED)
        }
    }
}