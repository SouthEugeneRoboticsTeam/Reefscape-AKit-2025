package org.sert2521.reefscape2025.subsystems.elevator

import com.ctre.phoenix6.hardware.CANrange
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.units.Units
import org.sert2521.reefscape2025.ElectronicIDs.ELEVATOR_LEFT_ID
import org.sert2521.reefscape2025.ElectronicIDs.ELEVATOR_RIGHT_ID
import org.sert2521.reefscape2025.ElectronicIDs.LASER_ID
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_D
import org.sert2521.reefscape2025.TuningConstants.ELEVATOR_P

class ElevatorIOSpark:ElevatorIO {
    val leftMotor = SparkMax(ELEVATOR_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
    val rightMotor = SparkMax(ELEVATOR_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)

    val leftConfig = SparkMaxConfig()
    val rightConfig = SparkMaxConfig()

    val distanceSensor = CANrange(LASER_ID)
    val rollingAverage = LinearFilter.movingAverage(4)

    init{
        leftConfig
            .inverted(false)
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .encoder
            .positionConversionFactor(1.0)//0.02328333333)
            .velocityConversionFactor(1.0)//0.02328333333)

        rightConfig
            .smartCurrentLimit(40)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .follow(leftMotor, true)
        .encoder
            .positionConversionFactor(1.0)//0.02328333333)
            .velocityConversionFactor(1.0)//0.02328333333)

        leftMotor.configure(leftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        rightMotor.configure(rightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        inputs.currentAmps = (leftMotor.outputCurrent + rightMotor.outputCurrent) / 2
        inputs.appliedVolts = (leftMotor.busVoltage * leftMotor.appliedOutput + rightMotor.busVoltage * rightMotor.appliedOutput)/2
        inputs.laserPosition = rollingAverage.calculate(distanceSensor.distance.value.`in`(Units.Meters) * 2 - 0.18)
        inputs.laserVelocity = (leftMotor.encoder.velocity + rightMotor.encoder.velocity) / 2
        //inputs.motorPosition = (leftMotor.encoder.position) + rightMotor.encoder.position//) / 2
        inputs.motorPosition = leftMotor.encoder.position
    }

    override fun setVoltage(voltage: Double) {
        leftMotor.setVoltage(voltage)
    }

    override fun setEncoder(encoderValue: Double) {
        leftMotor.encoder.setPosition(encoderValue)
        rightMotor.encoder.setPosition(encoderValue)
    }

}