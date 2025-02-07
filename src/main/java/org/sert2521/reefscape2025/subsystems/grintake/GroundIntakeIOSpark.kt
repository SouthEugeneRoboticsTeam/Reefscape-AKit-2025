package org.sert2521.reefscape2025.subsystems.grintake

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

    //Wrist
    val wristMotor = SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless)
    val wristConfig = SparkMaxConfig()
    val absEncoder = DutyCycleEncoder(WRIST_ABS_ENCODER)

    init {
        //Intake settings
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)

        //Wrist settings
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(WRIST_CURRENT_LIMIT)
            .encoder.positionConversionFactor(WRIST_MOTOR_MULTIPLIER)
                .velocityConversionFactor(WRIST_MOTOR_MULTIPLIER)

        //Configuration
        intakeMotor.configure(intakeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
        //Update Intake Inputs
        inputs.intakeCurrentAmps = intakeMotor.outputCurrent
        inputs.intakeAppliedVolts = intakeMotor.busVoltage * intakeMotor.appliedOutput
        inputs.intakeVelocityRPM = intakeMotor.encoder.velocity

        //Update Wrist Inputs
        inputs.wristAppliedVolts = wristMotor.outputCurrent
        inputs.wristCurrentAmps = wristMotor.busVoltage * wristMotor.appliedOutput
        inputs.wristVelocityRadPerSec = wristMotor.encoder.velocity
        inputs.wristPosition = absEncoder.get() * WRIST_ENCODER_MULTIPLIER + WRIST_ENCODER_TRANSFORM
    }

    override fun setIntakeMotor(speed: Double) {
        intakeMotor.set(speed)
    }

    override fun setIntakeVoltage(voltage: Double) {
        intakeMotor.setVoltage(voltage)
    }

    override fun setWristMotor(speed: Double) {
        wristMotor.set(speed)
    }

    override fun setWristVoltage(voltage:Double){
        wristMotor.setVoltage(voltage)
    }

}