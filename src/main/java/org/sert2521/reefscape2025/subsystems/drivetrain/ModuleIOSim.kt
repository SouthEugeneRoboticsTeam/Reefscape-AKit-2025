package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DutyCycle
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.sert2521.reefscape2025.subsystems.drivetrain.ModuleIO.ModuleIOInputs
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_D
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_P
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_SIM_KS
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_SIM_KV
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.TURN_D
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.TURN_P
import org.sert2521.reefscape2025.utils.SparkUtil
import java.util.*
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign
import kotlin.time.times


class ModuleIOSim(private val moduleSimulation:SwerveModuleSimulation):ModuleIO {

    private var driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(SwerveConstants.DRIVE_CURRENT_LIMIT_TELE.toDouble()))
    private var turnMotor = moduleSimulation.useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(SwerveConstants.TURN_CURRENT_LIMIT_TELE.toDouble()))

    private var driveClosedLoop = false
    private var turnClosedLoop = false
    private val drivePIDController = PIDController(DRIVE_P, 0.0, DRIVE_D)
    private val turnPIDController = PIDController(32.0, 0.0, 0.0)

    private var driveFFVolts = 0.0
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0

    init{
        turnPIDController.enableContinuousInput(-PI, PI)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = (driveFFVolts
                    + drivePIDController.calculate(
                moduleSimulation.driveWheelFinalSpeed.`in`(RadiansPerSecond)
            ) * SimulatedBattery.getBatteryVoltage().`in`(Volts))
        } else {
            drivePIDController.reset()
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnPIDController.calculate(
                moduleSimulation.steerAbsoluteFacing.radians
            ) * SimulatedBattery.getBatteryVoltage().`in`(Volts)
        } else {
            turnPIDController.reset()
        }

        // Update simulation state
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts))
        turnMotor.requestVoltage(Volts.of(turnAppliedVolts))

        // Update drive inputs
        inputs.driveConnected = true
        inputs.drivePositionRad = moduleSimulation.driveWheelFinalPosition.`in`(Radians)
        inputs.driveVelocityRadPerSec =
            moduleSimulation.driveWheelFinalSpeed.`in`(RadiansPerSecond)
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrentAmps = abs(moduleSimulation.driveMotorStatorCurrent.`in`(Amps))

        // Update turn inputs
        inputs.turnConnected = true
        inputs.turnPositionMotor = moduleSimulation.steerAbsoluteFacing
        inputs.turnPositionAbsolute = moduleSimulation.steerAbsoluteFacing
        inputs.turnVelocityRadPerSec =
            moduleSimulation.steerAbsoluteEncoderSpeed.`in`(RadiansPerSecond)
        inputs.turnAppliedVolts = turnAppliedVolts
        inputs.turnCurrentAmps = abs(moduleSimulation.steerMotorStatorCurrent.`in`(Amps))
    }

    override fun setDriveOpenLoop(output: Double) {
        driveClosedLoop = false
        driveAppliedVolts = output
    }

    override fun setTurnOpenLoop(output: Double) {
        turnClosedLoop = false
        turnAppliedVolts = output
    }

    override fun setDriveVelocity(velocityRadPerSec: Double, withPID: Boolean) {
        driveClosedLoop = true
        driveFFVolts = DRIVE_SIM_KS * sign(velocityRadPerSec) + DRIVE_SIM_KV * velocityRadPerSec
        drivePIDController.setSetpoint(velocityRadPerSec)
    }

    override fun setTurnPosition(rotation: Rotation2d) {
        turnClosedLoop = true
        turnPIDController.setSetpoint(rotation.radians)
    }

    override fun setCurrentLimit(limit: Int) {
        driveMotor = driveMotor.withCurrentLimit(Amps.of(limit.toDouble()))
        turnMotor = turnMotor.withCurrentLimit(Amps.of(limit.toDouble()))
    }

    override fun updateTurnEncoder(rotation: Rotation2d) {}
}