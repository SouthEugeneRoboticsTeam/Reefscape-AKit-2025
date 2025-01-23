package org.sert2521.reefscape2025.subsystems.drive

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import org.sert2521.reefscape2025.utils.SparkUtil
import org.sert2521.reefscape2025.utils.SparkUtil.ifOk
import org.sert2521.reefscape2025.utils.SparkUtil.sparkStickyFault
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.sign

//It was quite the fun exercise to translate this into kotlin

class ModuleIOSpark(module:Int):ModuleIO {
    val zeroRotation = SwerveConstants.moduleZeroRotations[module]

    val driveMotor = SparkMax(SwerveConstants.driveIDs[module], MotorType.kBrushless)
    val turnMotor = SparkMax(SwerveConstants.turnIDs[module], MotorType.kBrushless)

    val driveEncoder = driveMotor.encoder
    val turnEncoder = turnMotor.encoder

    val absEncoder = CANcoder(SwerveConstants.encoderIDs[module])

    val driveController = driveMotor.closedLoopController
    val turnController = turnMotor.closedLoopController

    val timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue()
    val drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(
        driveMotor,
        driveEncoder::getPosition
    )

    val turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(
        turnMotor, turnEncoder::getPosition
    )

    val driveConnectedDebounce = Debouncer(0.5)
    val turnConnectedDebounce = Debouncer(0.5)


    init {
        val driveConfig = SparkMaxConfig()

        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT)
            .voltageCompensation(12.0)

        driveConfig.encoder
            .positionConversionFactor(SwerveConstants.DRIVE_CONVERSION_POSITION)
            .velocityConversionFactor(SwerveConstants.DRIVE_CONVERSION_VELOCITY)
            //does NOT say uwuMeasurementPeriod (Kona, I mean YOU)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2)

        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                SwerveConstants.DRIVE_P,
                SwerveConstants.DRIVE_I,
                SwerveConstants.DRIVE_D,
                0.0,
            )

        driveConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(SwerveConstants.ODOMETRY_PERIOD)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20)

        SparkUtil.tryUntilOk(driveMotor, 5) {
            driveMotor.configure(
                driveConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        }

        SparkUtil.tryUntilOk(driveMotor, 5) {
            driveEncoder.setPosition(0.0)
        }

        val turnConfig = SparkMaxConfig()
        turnConfig
            .inverted(SwerveConstants.TURN_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveConstants.TURN_CURRENT_LIMIT)
            .voltageCompensation(12.0)

        turnConfig.encoder
            .inverted(SwerveConstants.TURN_REL_ENCODER_INVERTED)
            .positionConversionFactor(SwerveConstants.TURN_REL_CONVERSION_POSITION)
            .velocityConversionFactor(SwerveConstants.TURN_REL_CONVERSION_VELOCITY)
            .uvwMeasurementPeriod(20)
            .uvwAverageDepth(2)

        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-PI, PI)
            .pidf(
                SwerveConstants.TURN_P,
                SwerveConstants.TURN_I,
                SwerveConstants.TURN_D,
                0.0
            )

        turnConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(SwerveConstants.ODOMETRY_PERIOD)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20)

        SparkUtil.tryUntilOk(turnMotor, 5) {
            turnMotor.configure(
                turnConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        }
    }

    //MIGHT WANT TO CHANGE THIS TO MAKE SENSE

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        sparkStickyFault = false
        ifOk(driveMotor, driveEncoder::getPosition) {
            inputs.drivePositionRad = it
        }
        ifOk(driveMotor, driveEncoder::getVelocity) {
            inputs.driveVelocityRadPerSec = it
        }
        ifOk(driveMotor, driveEncoder::getPosition) {
            inputs.drivePositionRad = it
        }
        ifOk(driveMotor, arrayOf(DoubleSupplier{driveMotor.appliedOutput}, DoubleSupplier{driveMotor.busVoltage})) {
            inputs.driveAppliedVolts = it[0]*it[1]
        }
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault)

        sparkStickyFault=false
        ifOk(turnMotor, turnEncoder::getPosition) {
            inputs.turnPosition=Rotation2d(it).minus(zeroRotation)
        }
        ifOk(turnMotor, turnEncoder::getVelocity) {
            inputs.turnVelocityRadPerSec = it
        }
        ifOk(turnMotor, arrayOf(DoubleSupplier(turnMotor::getAppliedOutput), DoubleSupplier(turnMotor::getBusVoltage))){
            inputs.turnAppliedVolts = it[0]*it[1]
        }
        ifOk(turnMotor, turnMotor::getOutputCurrent){
            inputs.turnCurrentAmps = it
        }
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault)
    }

    override fun setDriveOpenLoop(output: Double) {
        driveMotor.setVoltage(output)
    }

    override fun setTurnOpenLoop(output: Double) {
        turnMotor.setVoltage(output)
    }

    override fun setDriveVelocity(velocityRadPerSec: Double) {
        val ffVolts = SwerveConstants.DRIVE_KS*velocityRadPerSec.sign + SwerveConstants.DRIVE_FF*velocityRadPerSec
        driveController.setReference(
            velocityRadPerSec,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage
        )
    }

    override fun setTurnPosition(rotation: Rotation2d) {
        val setpoint = MathUtil.inputModulus(
            rotation.plus(zeroRotation).radians,
            SwerveConstants.TURN_PID_MIN_INPUT,
            SwerveConstants.TURN_PID_MAX_INPUT
        )

        turnController.setReference(setpoint, SparkBase.ControlType.kPosition)
    }

    override fun updateTurnEncoder() {
        if (SwerveConstants.TURN_INVERTED) {
            turnEncoder.setPosition(
                SwerveConstants.TURN_ABS_ENCODER_CONVERSION_POSITION * absEncoder.absolutePosition.valueAsDouble
            )
        } else {
            turnEncoder.setPosition(
                -SwerveConstants.TURN_ABS_ENCODER_CONVERSION_POSITION*absEncoder.absolutePosition.valueAsDouble
            )
        }
    }
}