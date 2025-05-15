package org.sert2521.reefscape2025.subsystems.ramp

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.MetaConstants
import org.sert2521.reefscape2025.SetpointConstants.RAMP_INTAKE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.RAMP_RECENTER_SPEED
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain

object Ramp : SubsystemBase() {
    private val io = when (MetaConstants.currentMode){
        MetaConstants.Mode.REAL -> RampIOSpark()
        MetaConstants.Mode.SIM -> RampIOSim(Drivetrain.swerveDriveSimulation!!)
        MetaConstants.Mode.REPLAY -> object:RampIO{}
    }

    private val ioInputs = LoggedRampIOInputs()

    init{
        defaultCommand = idleCommand()
    }

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

    fun idleCommand():Command{
        return run{
            if (Dispenser.getBlocked()){
                setSpeed(RAMP_INTAKE_SPEED)
            } else {
                setSpeed(0.0)
            }
        }
    }

    fun recenterCommand():Command{
        return run{
            setSpeed(RAMP_RECENTER_SPEED)
        }.until {
            !Dispenser.getBlocked()
        }
    }
}