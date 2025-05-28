package org.sert2521.reefscape2025.subsystems.wrist

import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_MOTOR_ID
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ABS_ENCODER_ZERO
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_MOTOR_ENCODER_MULTIPLIER
import org.sert2521.reefscape2025.TuningConstants.WRIST_CURRENT_LIMIT
import org.sert2521.reefscape2025.TuningConstants.WRIST_P_FAST
import org.sert2521.reefscape2025.TuningConstants.WRIST_D_FAST
import org.sert2521.reefscape2025.TuningConstants.WRIST_D_SLOW
import org.sert2521.reefscape2025.TuningConstants.WRIST_P_SLOW
import kotlin.math.cos

class WristIOSpark:WristIO {
    private val wristMotor = SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless)


    init {
        // Put the spark config into the init, so that java can garbage collect it
        // It's a small optimization because now it doesn't have to store the value the whole code
        val wristConfig = SparkMaxConfig()

        // Wrist settings
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(WRIST_CURRENT_LIMIT)

        wristConfig.absoluteEncoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0/60)
            .zeroOffset(WRIST_ABS_ENCODER_ZERO)
            .zeroCentered(true)
            .inverted(true)

        wristConfig.encoder
            .positionConversionFactor(WRIST_MOTOR_ENCODER_MULTIPLIER)
            .velocityConversionFactor(WRIST_MOTOR_ENCODER_MULTIPLIER/60)

        wristConfig.closedLoop
            .pidf(
                WRIST_P_FAST, 0.0,
                WRIST_D_FAST, 0.0, ClosedLoopSlot.kSlot0
            )
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

        wristConfig.closedLoop
            .pidf(
                WRIST_P_SLOW, 0.0,
                WRIST_D_SLOW, 0.0, ClosedLoopSlot.kSlot1
            )
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

        wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun setVoltage(voltage:Double) {
//        wristMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: WristIO.WristIOInputs) {
        inputs.wristAppliedVolts = wristMotor.busVoltage * wristMotor.appliedOutput
        inputs.wristCurrentAmps = wristMotor.outputCurrent
        inputs.wristVelocityRadPerSec = wristMotor.absoluteEncoder.velocity
        inputs.wristAbsPosition = wristMotor.absoluteEncoder.position
        inputs.wristMotorPosition = wristMotor.encoder.position
    }

    override fun setReferenceFast(targetPosition: Double) {
        wristMotor.closedLoopController.setReference(
            targetPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )
    }

    override fun setReferenceSlow(targetPosition: Double) {
        wristMotor.closedLoopController.setReference(
            targetPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot1
        )
    }

    override fun resetMotorEncoder() {
        wristMotor.encoder.position = wristMotor.absoluteEncoder.position
    }
}