package org.sert2521.reefscape2025.subsystems.grintake

import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import org.sert2521.reefscape2025.ElectronicIDs.GROUNDINTAKE_MOTOR_ID
import org.sert2521.reefscape2025.subsystems.dispenser.DispenserIO

class GroundIntakeIOSpark {
    val motor = SparkMax(GROUNDINTAKE_MOTOR_ID, MotorType.kBrushless)
    val config = SparkMaxConfig()
    init {
        config.inverted(false)
        config.idleMode(IdleMode.kBrake)
        config.smartCurrentLimit(30)
    }

    fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
        inputs.currentAmps = motor.outputCurrent
        inputs.appliedVolts = motor.busVoltage * motor.appliedOutput
        inputs.velocityRPM = motor.encoder.velocity
    }
}