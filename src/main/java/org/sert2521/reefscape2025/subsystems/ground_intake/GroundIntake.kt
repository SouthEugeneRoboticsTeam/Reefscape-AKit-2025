package org.sert2521.reefscape2025.subsystems.ground_intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.reefscape2025.SetpointConstants.GROUND_INTAKE_SPEED
import org.sert2521.reefscape2025.SetpointConstants.GROUND_OUTTAKE_SPEED

object GroundIntake : SubsystemBase() {

    private val io = GroundIntakeIOSpark()
    private val ioInputs = LoggedGroundIntakeIOInputs()

    init {
        defaultCommand = stopCommand()
    }

    //Updates inputs ig
    override fun periodic() {
        io.updateInputs(ioInputs)
    }

    //Intake functions
    fun setVoltage(voltage:Double){
        io.setIntakeVoltage(voltage)
    }

    fun setMotor(speed:Double){
        io.setIntakeMotor(speed)
    }

    fun stop() {
        io.setIntakeMotor(0.0)
    }

    fun getVelocity():Double{
        return ioInputs.intakeVelocityRPM
    }

    fun stopCommand():Command {
        return run{ stop() }
    }

    fun intakeCommand():Command{
        return run{
            setMotor(GROUND_INTAKE_SPEED)
        }
    }

    fun outtakeCommand():Command{
        return run{
            setMotor(GROUND_OUTTAKE_SPEED)
        }
    }
}