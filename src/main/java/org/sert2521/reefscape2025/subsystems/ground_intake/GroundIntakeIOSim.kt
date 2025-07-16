package org.sert2521.reefscape2025.subsystems.ground_intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly
import org.sert2521.reefscape2025.GroundIntakeSimConstants
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.wrist.Wrist

class GroundIntakeIOSim: GroundIntakeIO {
    private val intakeMotorSimulation = MapleMotorSim(GroundIntakeSimConstants.motorConfigs)
    private val intakeMotor = intakeMotorSimulation.useSimpleDCMotorController()
        .withCurrentLimit(Amps.of(30.0))
    private val intakeCoralSimulation = IntakeSimulation.OverTheBumperIntake(
        "Coral",
        Drivetrain.swerveDriveSimulation,
        Meters.of(0.5715),
        Meters.of(0.294795),
        IntakeSimulation.IntakeSide.BACK,
        1
    )
    private val intakeAlgaeSimulation = IntakeSimulation.OverTheBumperIntake(
        "Algae",
        Drivetrain.swerveDriveSimulation,
        Meters.of(0.5715),
        Meters.of(0.294795),
        IntakeSimulation.IntakeSide.BACK,
        1
    )

    init {
        intakeCoralSimulation.register()
        intakeAlgaeSimulation.register()
    }

    private var requestedVoltage = Volts.zero()

    override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
        intakeMotor.requestVoltage(requestedVoltage)

        intakeMotorSimulation.update(Seconds.of(0.02))

        inputs.intakeCurrentAmps = intakeMotorSimulation.supplyCurrent.`in`(Amps)
        inputs.intakeAppliedVolts = intakeMotorSimulation.appliedVoltage.`in`(Volts)
        inputs.intakeVelocityRPM = intakeMotorSimulation.encoderVelocity.`in`(RPM)

        if (MathUtil.isNear(SetpointConstants.WRIST_GROUND, Wrist.getRotations(), 0.1)
            && inputs.intakeVelocityRPM > 1000.0){
            intakeCoralSimulation.startIntake()
        } else {
            intakeCoralSimulation.stopIntake()
        }

        if (MathUtil.isNear(SetpointConstants.WRIST_ALGAE_LOW, Wrist.getRotations(), 0.1)
            && inputs.intakeVelocityRPM < -1000.0){
            intakeAlgaeSimulation.startIntake()
        } else {
            intakeAlgaeSimulation.stopIntake()
        }

        if (MathUtil.isNear(SetpointConstants.WRIST_L1, Wrist.getRotations(), 0.1)
            && inputs.intakeVelocityRPM < -1000.0){
            if (intakeCoralSimulation.obtainGamePieceFromIntake()){
                SimulatedArena.getInstance().addGamePieceProjectile(
                    ReefscapeCoralOnFly(
                    Drivetrain.swerveDriveSimulation!!.simulatedDriveTrainPose.translation,
                    Translation2d(0.0, -0.6),
                    Drivetrain.swerveDriveSimulation.driveTrainSimulatedChassisSpeedsFieldRelative,
                    Rotation2d.kCW_90deg+Drivetrain.swerveDriveSimulation.simulatedDriveTrainPose.rotation,
                    Meters.of(0.635499),
                    MetersPerSecond.of(0.0),
                    Degrees.of(0.0)
                    )
                )
            }
        }


        if (MathUtil.isNear(SetpointConstants.WRIST_ALGAE_LOW, Wrist.getRotations(), 0.1)
            && inputs.intakeVelocityRPM > 1000.0){
            if (intakeAlgaeSimulation.obtainGamePieceFromIntake()){
                SimulatedArena.getInstance().addGamePieceProjectile(
                    ReefscapeAlgaeOnFly(
                        Drivetrain.swerveDriveSimulation!!.simulatedDriveTrainPose.translation,
                        Translation2d(0.0, -0.6),
                        Drivetrain.swerveDriveSimulation.driveTrainSimulatedChassisSpeedsFieldRelative,
                        Rotation2d.kCW_90deg+Drivetrain.swerveDriveSimulation.simulatedDriveTrainPose.rotation,
                        Meters.of(0.635499),
                        MetersPerSecond.of(1.5),
                        Degrees.of(0.0)
                    )
                )
            }
        }
    }

    override fun setIntakeMotor(speed: Double) {
        requestedVoltage = SimulatedBattery.getBatteryVoltage() * speed
    }

    override fun setIntakeVoltage(voltage: Double) {
        requestedVoltage = Volts.of(voltage)
    }
}