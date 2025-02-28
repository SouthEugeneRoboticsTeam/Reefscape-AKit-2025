package org.sert2521.reefscape2025.subsystems.ramp

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.sert2521.reefscape2025.ElectronicIDs.RAMP_MOTOR_ID

class RampIOSpark:RampIO {
    private val motor = SparkMax(RAMP_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    init{
        val config = SparkMaxConfig()
        config
            .inverted(false)
            .smartCurrentLimit(20)
            .idleMode(SparkBaseConfig.IdleMode.kCoast)

        motor.configure(config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: RampIO.RampIOInputs) {
        inputs.speedRPM = motor.encoder.velocity
        inputs.appliedVolts = motor.busVoltage * motor.appliedOutput
        inputs.currentAmps = motor.outputCurrent
    }

    override fun setSpeed(speed: Double) { motor.set(speed) }
}