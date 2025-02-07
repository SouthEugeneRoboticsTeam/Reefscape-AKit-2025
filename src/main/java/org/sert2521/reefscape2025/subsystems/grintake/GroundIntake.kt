package org.sert2521.reefscape2025.subsystems.grintake

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.reefscape2025.commands.grintake.HoldWrist

object GroundIntake : SubsystemBase() {

    private val io = GroundIntakeIOSpark()
    private val ioInputs = LoggedGroundIntakeIOInputs()

    init {
        defaultCommand = HoldWrist()
    }

    //Updates inputs ig
    override fun periodic() {
        io.updateInputs(ioInputs)
    }

    //Intake functions
    fun setIntakeVoltage(voltage:Double){
        io.setIntakeVoltage(voltage)
    }

    fun setIntakeMotor(speed:Double){
        io.setIntakeMotor(speed)
    }

    fun stopIntake() {
        io.setIntakeMotor(0.0)
    }

    fun getIntakeVelocity():Double{
        return ioInputs.intakeVelocityRPM
    }

    // Wrist Functions
    fun setWristVoltage(voltage:Double) {
        io.setWristVoltage(voltage)
    }

    fun setWristMotor(speed:Double) {
        io.setWristMotor(speed)
    }

    fun stopWrist() {
        io.setWristMotor(0.0)
    }

    fun getWristVelocity():Double {
        return ioInputs.wristVelocityRadPerSec
    }

    fun getWristRadians(): Double {
        return ioInputs.wristPosition
    }
}