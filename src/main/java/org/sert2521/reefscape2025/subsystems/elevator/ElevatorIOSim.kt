package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.ElevatorSimConstants
import org.sert2521.reefscape2025.ElevatorSimConstants.ELEVATOR_SIM_D
import org.sert2521.reefscape2025.ElevatorSimConstants.ELEVATOR_SIM_G
import org.sert2521.reefscape2025.ElevatorSimConstants.ELEVATOR_SIM_I
import org.sert2521.reefscape2025.ElevatorSimConstants.ELEVATOR_SIM_P
import org.sert2521.reefscape2025.ElevatorSimConstants.ELEVATOR_SIM_S
import org.sert2521.reefscape2025.ElevatorSimConstants.ELEVATOR_SIM_V

class ElevatorIOSim : ElevatorIO {
    private val elevatorSim = ElevatorSim(
        ElevatorSimConstants.motorGearbox,
        ElevatorSimConstants.GEAR_RATIO,
        ElevatorSimConstants.CARRIAGE_MASS_KG,
        ElevatorSimConstants.DRUM_RADIUS_METERS,
        ElevatorSimConstants.MIN_HEIGHT,
        ElevatorSimConstants.MAX_HEIGHT,
        true,
        (ElevatorSimConstants.MAX_HEIGHT + ElevatorSimConstants.MIN_HEIGHT) / 2
    )

    private val pid = PIDController(ELEVATOR_SIM_P, ELEVATOR_SIM_I, ELEVATOR_SIM_D)
    private val feedforward = ElevatorFeedforward(ELEVATOR_SIM_S, ELEVATOR_SIM_G, ELEVATOR_SIM_V)
    private var feedforwardInput = 0.0
    private var inputVoltage = 0.0
    private var closedLoop = true

    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        if (closedLoop) {
            inputVoltage = pid.calculate(elevatorSim.positionMeters) + feedforwardInput
        } else {
            pid.calculate(elevatorSim.positionMeters)
        }

        elevatorSim.setInputVoltage(inputVoltage)
        elevatorSim.update(0.02)

        inputs.positionMeters = elevatorSim.positionMeters
        inputs.currentAmps = elevatorSim.currentDrawAmps
        inputs.appliedVolts = elevatorSim.currentDrawAmps * inputVoltage
        inputs.velocityMetersPerSec = elevatorSim.velocityMetersPerSecond

        Logger.recordOutput("Elevator Error", pid.setpoint - elevatorSim.positionMeters)
    }

    override fun setVoltage(voltage: Double) {
        closedLoop = false
        inputVoltage = voltage
        feedforwardInput = 0.0
    }

    override fun setReference(setpointPosition: Double, setpointVelocity: Double) {
        closedLoop = true
        pid.setpoint = setpointPosition
        Logger.recordOutput("Elevator Setpoint", setpointPosition)
        feedforwardInput = feedforward.calculate(setpointVelocity)
    }
}