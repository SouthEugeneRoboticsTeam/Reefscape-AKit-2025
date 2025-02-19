package org.sert2521.reefscape2025.subsystems.drivetrain

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import org.sert2521.reefscape2025.utils.SparkUtil
import org.sert2521.reefscape2025.utils.SparkUtil.ifOk
import org.sert2521.reefscape2025.utils.SparkUtil.tryUntilOk
import java.util.function.DoubleSupplier
import kotlin.math.PI

//It was quite the fun exercise to translate this into kotlin

class ModuleIOSpark(module:Int):ModuleIO {
    private val zeroRotation = SwerveConstants.moduleZeroRotations[module]

    private val driveMotor = SparkMax(SwerveConstants.driveIDs[module], MotorType.kBrushless)
    private val turnMotor = SparkMax(SwerveConstants.turnIDs[module], MotorType.kBrushless)

    private val driveEncoder = driveMotor.encoder
    private val turnEncoder = turnMotor.encoder

    private val driveFeedforward = SimpleMotorFeedforward(
        SwerveConstants.DRIVE_KS,
        SwerveConstants.DRIVE_KV,
        SwerveConstants.DRIVE_KA
    )

    private var lastVelocity = 0.0

    private val absEncoder = CANcoder(SwerveConstants.encoderIDs[module])

    private val driveController = driveMotor.closedLoopController
    private val turnController = turnMotor.closedLoopController

    private val timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue()
    private val drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(
        driveMotor,
        driveEncoder::getPosition
    )

    private val turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(
        turnMotor, turnEncoder::getPosition
    )

    private val driveConnectedDebounce = Debouncer(0.5)
    private val turnConnectedDebounce = Debouncer(0.5)

    private val driveConfig = SparkMaxConfig()
    private val turnConfig = SparkMaxConfig()


    init {
        driveConfig
            .idleMode(SwerveConstants.moduleIdleMode)
            .smartCurrentLimit(SwerveConstants.DRIVE_CURRENT_LIMIT_TELE)
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

        turnConfig
            .inverted(SwerveConstants.TURN_INVERTED)
            .idleMode(SwerveConstants.moduleIdleMode)
            .smartCurrentLimit(SwerveConstants.TURN_CURRENT_LIMIT_TELE)
            .voltageCompensation(12.0)

        turnConfig.encoder
            //.inverted(SwerveConstants.TURN_REL_ENCODER_INVERTED)
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
        SparkUtil.sparkStickyFault = false
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
        inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault)

        SparkUtil.sparkStickyFault=false
        ifOk(turnMotor, turnEncoder::getPosition) {
            inputs.turnPositionMotor=Rotation2d(it).minus(zeroRotation)
        }
        inputs.turnPositionAbsolute=Rotation2d.fromRotations(absEncoder.position.valueAsDouble)
        ifOk(turnMotor, turnEncoder::getVelocity) {
            inputs.turnVelocityRadPerSec = it
        }
        ifOk(turnMotor, arrayOf(DoubleSupplier(turnMotor::getAppliedOutput), DoubleSupplier(turnMotor::getBusVoltage))){
            inputs.turnAppliedVolts = it[0]*it[1]
        }
        ifOk(turnMotor, turnMotor::getOutputCurrent){
            inputs.turnCurrentAmps = it
        }
        inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault)


        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble { it }.toArray()
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble { it }.toArray()
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map { Rotation2d(it).minus(zeroRotation) }
                    .toArray().filterIsInstance<Rotation2d>().toTypedArray()
        timestampQueue.clear()
        drivePositionQueue.clear()
        turnPositionQueue.clear()
    }

    override fun setDriveOpenLoop(output: Double) {
        driveMotor.setVoltage(output)
    }

    override fun setTurnOpenLoop(output: Double) {
        turnMotor.setVoltage(output)
    }

    override fun setDriveVelocity(velocityRadPerSec: Double) {
        val ffVolts = driveFeedforward.calculate(velocityRadPerSec)
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

    override fun updateTurnEncoder(rotation: Rotation2d) {
        turnEncoder.setPosition(rotation.radians)
    }

    override fun setCurrentLimit(limit: Int) {
        driveConfig.smartCurrentLimit(limit)
        /* Not resetting or persisting parameters
           because we want it to go back to default when we lose power
           and also its goofy to save to flash multiple times a match */
        tryUntilOk(driveMotor, 5){
            driveMotor.configure(driveConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        }
    }
}