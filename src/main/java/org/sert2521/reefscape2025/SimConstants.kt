package org.sert2521.reefscape2025

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import org.ironmaple.simulation.motorsims.SimMotorConfigs


object ElevatorSimConstants {
    val motorGearbox: DCMotor = DCMotor.getNEO(2)
    const val GEAR_RATIO = 12.0
    const val CARRIAGE_MASS_KG = 16.3962065
    const val DRUM_RADIUS_METERS = 0.019304
    const val MIN_HEIGHT = 0.0
    const val MAX_HEIGHT = 0.660400

    const val ELEVATOR_SIM_P = 800.0
    const val ELEVATOR_SIM_I = 0.0
    const val ELEVATOR_SIM_D = 0.0

    const val ELEVATOR_SIM_S = 0.0
    const val ELEVATOR_SIM_G = 0.6
    const val ELEVATOR_SIM_V = 13.6
}

object WristSimConstants {
    const val GEAR_RATIO_MECH_PER_MOTOR = 3.0 / 200.0

    val motorConfigs = SimMotorConfigs(
        DCMotor.getNEO(1),
        1/GEAR_RATIO_MECH_PER_MOTOR,
        Units.KilogramSquareMeters.of(0.295),
        Volts.of(0.1)
    ) // Add hard limits, figure out pid stuff

    const val WRIST_P_SIM_FAST = 7.0
    const val WRIST_I_SIM_FAST = 0.0
    const val WRIST_D_SIM_FAST = 0.2

    const val WRIST_P_SIM_SLOW = 2.0
    const val WRIST_I_SIM_SLOW = 0.0
    const val WRIST_D_SIM_SLOW = 0.1
}