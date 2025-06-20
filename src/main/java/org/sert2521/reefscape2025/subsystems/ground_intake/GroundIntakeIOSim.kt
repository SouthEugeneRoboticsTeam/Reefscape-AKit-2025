package org.sert2521.reefscape2025.subsystems.ground_intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.dyn4j.geometry.Rectangle
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
import org.ironmaple.simulation.gamepieces.GamePieceProjectile
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide
import org.sert2521.reefscape2025.GroundIntakeSimConstants
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.wrist.Wrist

class GroundIntakeIOSim: GroundIntakeIO {
    private val intakeMotorSimulation = MapleMotorSim(GroundIntakeSimConstants.motorConfigs)
    private val intakeMotor = intakeMotorSimulation.useSimpleDCMotorController()
        .withCurrentLimit(Amps.of(30.0))
    private val intakeGamepieceSimulation = IntakeSimulation.OverTheBumperIntake(
        "Coral",
        Drivetrain.swerveDriveSimulation,
        Meters.of(0.5715),
        Meters.of(0.294795),
        IntakeSimulation.IntakeSide.BACK,
        1
    )

    init {
        intakeGamepieceSimulation.register()
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
            intakeGamepieceSimulation.startIntake()
        } else {
            intakeGamepieceSimulation.stopIntake()
        }

        if (MathUtil.isNear(SetpointConstants.WRIST_L1, Wrist.getRotations(), 0.1)
            && inputs.intakeVelocityRPM < -1000.0){
            if (intakeGamepieceSimulation.obtainGamePieceFromIntake()){
                ReefscapeCoralOnFly(
                    Drivetrain.swerveDriveSimulation!!.simulatedDriveTrainPose.translation,
                    Translation2d(-0.255899, 0.0),
                    Drivetrain.swerveDriveSimulation.driveTrainSimulatedChassisSpeedsFieldRelative,
                    Rotation2d.k180deg,
                    Meters.of(0.635499),
                    MetersPerSecond.of(0.25),
                    Rotations.of(Wrist.getRotations())
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