package org.sert2521.reefscape2025.subsystems.dispenser

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DigitalInput
import org.sert2521.reefscape2025.ElectronicIDs.BEAMBREAK_DISPENSER
import org.sert2521.reefscape2025.ElectronicIDs.BEAMBREAK_RAMP
import org.sert2521.reefscape2025.ElectronicIDs.DISPENSER_MOTOR_ID

class DispenserIOSpark:DispenserIO {
    private val motor = SparkMax(DISPENSER_MOTOR_ID, MotorType.kBrushless)
    private val beambreakDispenser = DigitalInput(BEAMBREAK_DISPENSER)
    private val beambreakRamp = DigitalInput(BEAMBREAK_RAMP)
    private val config = SparkMaxConfig()


    init {
        config
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)


        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: DispenserIO.DispenserIOInputs) {
        inputs.currentAmps = motor.outputCurrent
        inputs.appliedVolts = motor.busVoltage * motor.appliedOutput
        inputs.velocityRPM = motor.encoder.velocity

        inputs.beambreakDispenserClear = beambreakDispenser.get()
        inputs.beambreakRampClear = beambreakRamp.get()
    }

    override fun setMotor(speed: Double) {
        motor.set(speed)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }
}