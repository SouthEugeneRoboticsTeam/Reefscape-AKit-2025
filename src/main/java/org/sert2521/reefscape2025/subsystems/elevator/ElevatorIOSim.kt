package org.sert2521.reefscape2025.subsystems.elevator

import edu.wpi.first.wpilibj.simulation.ElevatorSim
import org.sert2521.reefscape2025.ElevatorSimConstants

class ElevatorIOSim:ElevatorIO {
    val elevatorSim = ElevatorSim(
        ElevatorSimConstants.motorGearbox,
        ElevatorSimConstants.GEAR_RATIO,
        ElevatorSimConstants.CARRIAGE_MASS_KG,
        ElevatorSimConstants.DRUM_RADIUS_METERS,
        ElevatorSimConstants.MIN_HEIGHT,
        ElevatorSimConstants.MAX_HEIGHT,
        true,
        ElevatorSimConstants.MIN_HEIGHT
    )
}