package org.sert2521.reefscape2025.subsystems.ground_intake

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.sert2521.reefscape2025.ElectronicIDs.GROUNDINTAKE_MOTOR_ID
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_ABS_ENCODER
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_MOTOR_ID
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ENCODER_MULTIPLIER
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ENCODER_TRANSFORM
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_MOTOR_MULTIPLIER
import org.sert2521.reefscape2025.TuningConstants.WRIST_CURRENT_LIMIT

class GroundIntakeIOSpark:GroundIntakeIO {
    //Intake
    val intakeMotor = SparkMax(GROUNDINTAKE_MOTOR_ID, MotorType.kBrushless)
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