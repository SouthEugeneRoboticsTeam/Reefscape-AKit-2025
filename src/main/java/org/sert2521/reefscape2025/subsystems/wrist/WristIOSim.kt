package org.sert2521.reefscape2025.subsystems.wrist

import com.revrobotics.spark.ClosedLoopSlot
import edu.wpi.first.units.Units.*
import org.ironmaple.simulation.motorsims.MapleMotorSim
import org.sert2521.reefscape2025.WristSimConstants
import org.sert2521.reefscape2025.WristSimConstants.WRIST_D_SIM_FAST
import org.sert2521.reefscape2025.WristSimConstants.WRIST_D_SIM_SLOW
import org.sert2521.reefscape2025.WristSimConstants.WRIST_I_SIM_FAST
import org.sert2521.reefscape2025.WristSimConstants.WRIST_I_SIM_SLOW
import org.sert2521.reefscape2025.WristSimConstants.WRIST_P_SIM_FAST
import org.sert2521.reefscape2025.WristSimConstants.WRIST_P_SIM_SLOW
import org.sert2521.reefscape2025.utils.sim.SimulatedSparkMax

class WristIOSim:WristIO {
    private val wristSimulation = MapleMotorSim(WristSimConstants.motorConfigs)
    private val motor = wristSimulation.useMotorController(SimulatedSparkMax(WristSimConstants.motorConfigs.motor)
        .withCurrentLimit(Amps.of(30.0))
        .withPIDController(ClosedLoopSlot.kSlot0, WRIST_P_SIM_FAST, WRIST_I_SIM_FAST, WRIST_D_SIM_FAST, 0.0)
        .withPIDController(ClosedLoopSlot.kSlot1, WRIST_P_SIM_SLOW, WRIST_I_SIM_SLOW, WRIST_D_SIM_SLOW, 0.0))

    private val startingPosition = Rotations.of(0.25)

    private var closedLoop = true
    private var fast = true
    private var targetPosition = 0.0

    private var requestedVoltage = 0.0

    override fun updateInputs(inputs: WristIO.WristIOInputs) {
        if (closedLoop){
            if (fast){
                motor.setReference(targetPosition, SimulatedSparkMax.ControlMode.POSITION, ClosedLoopSlot.kSlot0)
            } else {
                motor.setReference(targetPosition, SimulatedSparkMax.ControlMode.POSITION, ClosedLoopSlot.kSlot1)
            }
        } else {
            motor.requestVoltage(Volts.of(requestedVoltage))
        }

        wristSimulation.update(Seconds.of(0.02))

        inputs.wristMotorPosition = (wristSimulation.angularPosition + startingPosition).`in`(Rotations)
        inputs.wristAbsPosition = (wristSimulation.angularPosition + startingPosition).`in`(Rotations)
        inputs.wristCurrentAmps = wristSimulation.supplyCurrent.`in`(Amps)
        inputs.wristAppliedVolts = wristSimulation.appliedVoltage.`in`(Volts)
        inputs.wristVelocityRadPerSec = wristSimulation.velocity.`in`(RadiansPerSecond)
    }

    override fun setReferenceFast(targetPosition: Double) {
        closedLoop = true
        fast = true
        this.targetPosition = targetPosition - startingPosition.`in`(Rotations)
    }

    override fun setReferenceSlow(targetPosition: Double) {
        closedLoop = true
        fast = false
        this.targetPosition = targetPosition - startingPosition.`in`(Rotations)
    }

    override fun setVoltage(voltage: Double) {
        closedLoop = false
        requestedVoltage = voltage
    }
}