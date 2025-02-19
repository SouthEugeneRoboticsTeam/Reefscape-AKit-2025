package org.sert2521.reefscape2025.subsystems.wrist

import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_ABS_ENCODER
import org.sert2521.reefscape2025.ElectronicIDs.WRIST_MOTOR_ID
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ENCODER_MULTIPLIER
import org.sert2521.reefscape2025.PhysicalConstants.WRIST_ENCODER_TRANSFORM
import org.sert2521.reefscape2025.TuningConstants.WRIST_CURRENT_LIMIT
import org.sert2521.reefscape2025.TuningConstants.WRIST_G
import org.sert2521.reefscape2025.TuningConstants.WRIST_P
import org.sert2521.reefscape2025.TuningConstants.WRIST_D
import kotlin.math.PI
import kotlin.math.cos

class WristIOSpark:WristIO {
    private val wristMotor = SparkMax(WRIST_MOTOR_ID, MotorType.kBrushless)
    private val wristConfig = SparkMaxConfig()

    init {
        //Wrist settings
        wristConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(WRIST_CURRENT_LIMIT)

        wristConfig
            .absoluteEncoder
            .positionConversionFactor(WRIST_ENCODER_MULTIPLIER)
            .velocityConversionFactor(WRIST_ENCODER_MULTIPLIER)
            .zeroOffset(WRIST_ENCODER_TRANSFORM)
            .zeroCentered(true)
            .inverted(true)

        wristConfig.closedLoop
            .pidf(
                WRIST_P, 0.0,
                WRIST_D, 0.0
            )
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)

        wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun setVoltage(voltage:Double) {
//        wristMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: WristIO.WristIOInputs) {
        //Update Wrist Inputs
        inputs.wristAppliedVolts = wristMotor.outputCurrent
        inputs.wristCurrentAmps = wristMotor.busVoltage * wristMotor.appliedOutput
        inputs.wristVelocityRadPerSec = wristMotor.absoluteEncoder.velocity
        inputs.wristPosition = wristMotor.absoluteEncoder.position
    }

    override fun setReference(targetPosition: Double) {
        val arbFF = WRIST_G * cos(targetPosition)
        wristMotor.closedLoopController.setReference(
            targetPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            arbFF
        )
    }
}