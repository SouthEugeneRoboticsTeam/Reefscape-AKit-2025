package org.sert2521.reefscape2025.subsystems.ramp

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.ironmaple.simulation.motorsims.SimMotorConfigs
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.motorsims.SimulatedMotorController

class RampIOSim(drivetrainSim:SwerveDriveSimulation):RampIO {
    private val motorSimulation = MapleMotorSim(SimMotorConfigs(
        DCMotor.getNeo550(1),
        8.0/3.0,
        Units.KilogramSquareMeters.of(0.01),
        Units.Volts.of(0.05)
    ))

    private val intakeSimulation = IntakeSimulation.InTheFrameIntake(
        "Coral",
        drivetrainSim,
        Units.Inches.of(12.6),
        IntakeSimulation.IntakeSide.BACK,
        1
    )

    private val rampRollerMotor = motorSimulation.useSimpleDCMotorController()
        .withCurrentLimit(Units.Amps.of(20.0))

    init{
        intakeSimulation.register()
    }

    override fun updateInputs(inputs: RampIO.RampIOInputs) {
        motorSimulation.update(Units.Seconds.of(0.02))

        inputs.speedRPM = motorSimulation.velocity.`in`(Units.RotationsPerSecond)
        inputs.currentAmps = motorSimulation.supplyCurrent.`in`(Units.Amps)
        inputs.appliedVolts = rampRollerMotor.appliedVoltage.`in`(Units.Volts)

        if (inputs.speedRPM > 200.0){
            intakeSimulation.startIntake()
        } else {
            intakeSimulation.stopIntake()
        }
    }

    override fun setSpeed(speed: Double) {
        rampRollerMotor.requestVoltage(SimulatedBattery.getBatteryVoltage()*speed)
    }
}