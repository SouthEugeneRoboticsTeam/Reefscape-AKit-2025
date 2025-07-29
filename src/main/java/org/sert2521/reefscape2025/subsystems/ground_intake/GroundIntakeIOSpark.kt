package org.sert2521.reefscape2025.subsystems.ground_intake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import org.sert2521.reefscape2025.ElectronicIDs.GROUND_INTAKE_MOTOR_ID

class GroundIntakeIOSpark:GroundIntakeIO {
    //Intake
    val intakeMotor = SparkMax(GROUND_INTAKE_MOTOR_ID, MotorType.kBrushless)
    val intakeConfig = SparkMaxConfig()

    init {
        //Intake settings
        intakeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)

        //Configuration
        intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
        //Update Intake Inputs
        inputs.intakeCurrentAmps = intakeMotor.outputCurrent
        inputs.intakeAppliedVolts = intakeMotor.busVoltage * intakeMotor.appliedOutput
        inputs.intakeVelocityRPM = intakeMotor.encoder.velocity
    }

    override fun setIntakeMotor(speed: Double) {
        intakeMotor.set(speed)
    }

    override fun setIntakeVoltage(voltage: Double) {
        intakeMotor.setVoltage(voltage)
    }
}