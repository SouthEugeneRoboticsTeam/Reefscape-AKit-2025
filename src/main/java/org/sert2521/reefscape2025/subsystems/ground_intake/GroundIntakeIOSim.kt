package org.sert2521.reefscape2025.subsystems.ground_intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.dyn4j.geometry.Rectangle
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.sert2521.reefscape2025.GroundIntakeSimConstants
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

    private val requestedVoltage = Volts.zero()

    override fun updateInputs(inputs: GroundIntakeIO.GroundIntakeIOInputs) {
        intakeMotor.requestVoltage(requestedVoltage)

        intakeMotorSimulation.update(Seconds.of(0.02))

        inputs.intakeCurrentAmps = intakeMotorSimulation.supplyCurrent.`in`(Amps)
        inputs.intakeAppliedVolts = intakeMotorSimulation.appliedVoltage.`in`(Volts)
        inputs.intakeVelocityRPM = intakeMotorSimulation.encoderVelocity.`in`(RPM)

        if (Wrist.getRotations()<0.0){
            intakeGamepieceSimulation.startIntake()
        } else {
            intakeGamepieceSimulation.stopIntake()
        }
    }


}