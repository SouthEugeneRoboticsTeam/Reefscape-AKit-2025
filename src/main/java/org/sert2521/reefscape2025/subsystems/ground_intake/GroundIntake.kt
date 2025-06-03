package org.sert2521.reefscape2025.subsystems.ground_intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.SetpointConstants.GROUND_HOLD_ALGAE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.GROUND_INTAKE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.GROUND_OUTTAKE_SPEED_ALGAE
import org.sert2521.reefscape2025.SetpointConstants.GROUND_OUTTAKE_SPEED_CORAL
import org.sert2521.reefscape2025.subsystems.wrist.Wrist

object GroundIntake : SubsystemBase() {

    private val io = when (MetaConstants.currentMode) {
        MetaConstants.Mode.REAL -> GroundIntakeIOSpark()
        else -> object : GroundIntakeIO {}
    }
    private val ioInputs = LoggedGroundIntakeIOInputs()

    init {
        defaultCommand = stopCommand()
    }

    //Updates inputs ig
    override fun periodic() {
        io.updateInputs(ioInputs)
        Logger.processInputs("Wrist/Rollers", ioInputs)
    }

    //Intake functions
    fun setVoltage(voltage: Double) {
        io.setIntakeVoltage(voltage)
    }

    fun setMotor(speed: Double) {
        io.setIntakeMotor(speed)
    }

    fun stop() {
        io.setIntakeMotor(0.0)
    }

    fun stopCommand(): Command {
        return run { stop() }
    }

    fun intakeCommand(): Command {
        return run {
            setMotor(GROUND_INTAKE_SPEED)
        }
    }

    fun outtakeCommand(): Command {
        return run {
            setMotor(GROUND_OUTTAKE_SPEED_ALGAE)
        }
    }

    fun outtakeCoralCommand(): Command {
        return run {
            setMotor(GROUND_OUTTAKE_SPEED_CORAL)
        }
    }

    fun holdAlgaeCommand(): Command {
        return run {
            setMotor(GROUND_HOLD_ALGAE_SPEED)
        }
    }
}