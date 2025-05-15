package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import org.sert2521.reefscape2025.ElevatorSimConstants

class ElevatorIOSim:ElevatorIO {
    private val elevatorSim = ElevatorSim(
        ElevatorSimConstants.motorGearbox,
        ElevatorSimConstants.GEAR_RATIO,
        ElevatorSimConstants.CARRIAGE_MASS_KG,
        ElevatorSimConstants.DRUM_RADIUS_METERS,
        ElevatorSimConstants.MIN_HEIGHT,
        ElevatorSimConstants.MAX_HEIGHT,
        true,
        ElevatorSimConstants.MIN_HEIGHT
    )

    private val pid = PIDController(0.0, 0.0, 0.0)
    private val feedforward = ElevatorFeedforward(0.0, 0.0, 0.0)
    private var feedforwardInput = 0.0
    private var inputVoltage = 0.0
    private var closedLoop = true

    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        if (closedLoop){
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
    }

    override fun setVoltage(voltage: Double) {
        closedLoop = false
        inputVoltage = voltage
        feedforwardInput = 0.0
    }

    override fun setReference(setpointPosition: Double, setpointVelocity: Double) {
        closedLoop = true
        pid.setpoint = setpointPosition
        feedforwardInput = feedforward.calculate(setpointVelocity)
    }
}