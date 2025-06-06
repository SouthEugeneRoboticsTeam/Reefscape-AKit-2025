package org.sert2521.reefscape2025.subsystems.wrist

import com.revrobotics.spark.ClosedLoopSlot
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units.*
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.WristSimConstants
import org.sert2521.reefscape2025.WristSimConstants.WRIST_D_SIM_FAST
import org.sert2521.reefscape2025.WristSimConstants.WRIST_D_SIM_SLOW
import org.sert2521.reefscape2025.WristSimConstants.WRIST_I_SIM_FAST
import org.sert2521.reefscape2025.WristSimConstants.WRIST_I_SIM_SLOW
import org.sert2521.reefscape2025.WristSimConstants.WRIST_P_SIM_FAST
import org.sert2521.reefscape2025.WristSimConstants.WRIST_P_SIM_SLOW
import org.sert2521.reefscape2025.utils.sim.SimulatedSparkMax

class WristIOSim : WristIO {
    private val wristSimulation = MapleMotorSim(WristSimConstants.motorConfigs)
    private val motor = wristSimulation.useSimpleDCMotorController()
        .withCurrentLimit(Amps.of(30.0))

    private val startingPosition = Rotations.of(0.25)

    private val pidController = PIDController(WRIST_P_SIM_FAST, WRIST_I_SIM_FAST, WRIST_D_SIM_FAST)

    private var closedLoop = true
    private var targetPosition = 0.0

    private var requestedVoltage = 0.0

    override fun updateInputs(inputs: WristIO.WristIOInputs) {
        Logger.recordOutput("Sim target position", targetPosition)
        if (closedLoop) {
            requestedVoltage =
                pidController.calculate((wristSimulation.angularPosition + startingPosition).`in`(Rotations)) *
                        SimulatedBattery.getBatteryVoltage()
                            .`in`(Volts) // multiply by battery voltage because pid is in duty cycle
        } else {
            pidController.calculate(inputs.wristAbsPosition) // Even if we don't use the output, we still need to update pid
        }

        Logger.recordOutput("Sim wrist requested voltage", requestedVoltage)

        motor.requestVoltage(Volts.of(requestedVoltage))

        wristSimulation.update(Seconds.of(0.02))

        //Update again to give back updated inputs
        inputs.wristAbsPosition = (wristSimulation.angularPosition + startingPosition).`in`(Rotations)
        inputs.wristMotorPosition = inputs.wristAbsPosition
        inputs.wristCurrentAmps = wristSimulation.supplyCurrent.`in`(Amps)
        inputs.wristAppliedVolts = wristSimulation.appliedVoltage.`in`(Volts)
        inputs.wristVelocityRadPerSec = wristSimulation.velocity.`in`(RadiansPerSecond)
    }

    override fun setReferenceFast(targetPosition: Double) {
        closedLoop = true

        pidController.setPID(WRIST_P_SIM_FAST, WRIST_I_SIM_FAST, WRIST_D_SIM_FAST)
        pidController.setpoint = targetPosition
    }

    override fun setReferenceSlow(targetPosition: Double) {
        closedLoop = true

        pidController.setPID(WRIST_P_SIM_SLOW, WRIST_I_SIM_SLOW, WRIST_D_SIM_SLOW)
        pidController.setpoint = targetPosition
    }

    override fun setVoltage(voltage: Double) {
        closedLoop = false
        requestedVoltage = voltage
    }
}