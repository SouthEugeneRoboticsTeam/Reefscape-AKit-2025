package org.sert2521.reefscape2025.subsystems.elevator

import com.ctre.phoenix6.hardware.CANrange
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import org.sert2521.reefscape2025.ElectronicIDs.ELEVATOR_LEFT_ID
import org.sert2521.reefscape2025.ElectronicIDs.ELEVATOR_RIGHT_ID
import org.sert2521.reefscape2025.ElectronicIDs.LASER_ID
import org.sert2521.reefscape2025.PhysicalConstants.ELEVATOR_MOTOR_ENCODER_MULTIPLIER
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_D
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_G
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_P
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_V

class ElevatorIOSpark:ElevatorIO {
    val leftMotor = SparkMax(ELEVATOR_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
    val rightMotor = SparkMax(ELEVATOR_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)

    val leftConfig = SparkMaxConfig()
    val rightConfig = SparkMaxConfig()

    val distanceSensor = CANrange(LASER_ID)


    init{
        leftConfig
            .inverted(false)
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .encoder
            .positionConversionFactor(ELEVATOR_MOTOR_ENCODER_MULTIPLIER)
            .velocityConversionFactor(ELEVATOR_MOTOR_ENCODER_MULTIPLIER/60)

        leftConfig.closedLoop
            .pidf(
                ELEVATOR_P, 0.0,
                ELEVATOR_D, 0.0
            )

        rightConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(true)
        .encoder
            .positionConversionFactor(ELEVATOR_MOTOR_ENCODER_MULTIPLIER)
            .velocityConversionFactor(ELEVATOR_MOTOR_ENCODER_MULTIPLIER/60)

        rightConfig.closedLoop
            .pidf(
                ELEVATOR_P, 0.0,
                ELEVATOR_D, 0.0
            )

        leftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        rightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        inputs.currentAmps = (leftMotor.outputCurrent + rightMotor.outputCurrent) / 2
        inputs.appliedVolts = (leftMotor.busVoltage * leftMotor.appliedOutput + rightMotor.busVoltage * rightMotor.appliedOutput)/2
        inputs.laserPosition = distanceSensor.distance.value.`in`(Units.Meters)
        inputs.motorsVelocity = (leftMotor.encoder.velocity + rightMotor.encoder.velocity) / 2
        inputs.motorsPosition = (leftMotor.encoder.position + rightMotor.encoder.position) / 2
    }

    override fun setVoltage(voltage: Double) {
        leftMotor.setVoltage(voltage)
        rightMotor.setVoltage(voltage)
    }

    override fun setEncoder(encoderValue: Double) {
        leftMotor.encoder.setPosition(encoderValue)
        rightMotor.encoder.setPosition(encoderValue)
    }

    override fun setReference(setpoint: TrapezoidProfile.State) {
        val arbFF = setpoint.velocity * ELEVATOR_V + ELEVATOR_G

        leftMotor.closedLoopController.setReference(
            setpoint.position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            arbFF
        )
        rightMotor.closedLoopController.setReference(
            setpoint.position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            arbFF
        )
    }
}