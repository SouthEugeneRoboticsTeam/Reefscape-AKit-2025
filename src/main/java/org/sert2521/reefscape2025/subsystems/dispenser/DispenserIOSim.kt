package org.sert2521.reefscape2025.subsystems.dispenser

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.ironmaple.simulation.motorsims.SimMotorConfigs
import org.ironmaple.simulation.motorsims.SimulatedBattery

class DispenserIOSim:DispenserIO {
    private val motorSimulation = MapleMotorSim(
        SimMotorConfigs(
            DCMotor.getNEO(1),
            16.0/23.0,
            Units.KilogramSquareMeters.of(0.01),
            Units.Volts.of(0.01)
        )
    )

    private val dispenserRollerMotor = motorSimulation.useSimpleDCMotorController()
        .withCurrentLimit(Units.Amps.of(40.0))

    override fun updateInputs(inputs: DispenserIO.DispenserIOInputs) {
        motorSimulation.update(Units.Seconds.of(0.02))

        inputs.velocityRPM = motorSimulation.velocity.`in`(Units.RotationsPerSecond)
        inputs.currentAmps = motorSimulation.supplyCurrent.`in`(Units.Amps)
        inputs.appliedVolts = dispenserRollerMotor.appliedVoltage.`in`(Units.Volts)
    }

    override fun setMotor(speed: Double) {
        dispenserRollerMotor.requestVoltage(SimulatedBattery.getBatteryVoltage() * speed)
    }

    override fun setVoltage(voltage: Double) {
        dispenserRollerMotor.requestVoltage(Volts.of(voltage))
    }
}