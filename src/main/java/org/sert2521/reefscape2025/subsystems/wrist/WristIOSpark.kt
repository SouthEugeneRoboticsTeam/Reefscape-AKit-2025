package org.sert2521.reefscape2025.subsystems.wrist

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_ABS_ENCODER
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_MOTOR_ID
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ENCODER_MULTIPLIER
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ENCODER_TRANSFORM
import org.sert2521.reefscape2025.TuningConstants.WRIST_CURRENT_LIMIT

class WristIOSpark:WristIO {
    private val wristMotor = SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless)
    private val wristConfig = SparkMaxConfig()
    private val absEncoder = DutyCycleEncoder(WRIST_ABS_ENCODER)

    init {
        //Wrist settings
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(WRIST_CURRENT_LIMIT)

        wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun setVoltage(voltage:Double) {
        wristMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: WristIO.WristIOInputs) {
        //Update Wrist Inputs
        inputs.wristAppliedVolts = wristMotor.outputCurrent
        inputs.wristCurrentAmps = wristMotor.busVoltage * wristMotor.appliedOutput
        inputs.wristVelocityRadPerSec = wristMotor.encoder.velocity
        inputs.wristPosition = absEncoder.get() * WRIST_ENCODER_MULTIPLIER + WRIST_ENCODER_TRANSFORM
    }
}