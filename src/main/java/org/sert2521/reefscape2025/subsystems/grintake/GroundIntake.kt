package org.sert2521.reefscape2025.subsystems.grintake

import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.reefscape2025.ElectronicIDs
import org.sert2521.reefscape2025.PhysicalConstants

object GroundIntake : SubsystemBase() {

    private val io = GroundIntakeIOSpark()
    private val ioInputs = LoggedGroundIntakeIOInputs()
    val trueEncoder = DutyCycleEncoder(ElectronicIDs.WRIST_TRUE_ENCODER)
    var wristSetPoint = 0.0

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
        return ioInputs.wristVelocityRPM
    }

    fun getWristRadians(): Double {
        val wristAngle = (trueEncoder.get()) * PhysicalConstants.WRIST_ENCODER_MULTIPLIER + PhysicalConstants.WRIST_ENCODER_TRANSFORM
        return wristAngle
    }
}