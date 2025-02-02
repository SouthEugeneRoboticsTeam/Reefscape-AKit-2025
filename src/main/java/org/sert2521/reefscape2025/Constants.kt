package org.sert2521.reefscape2025

import edu.wpi.first.units.BaseUnits.MassUnit
import edu.wpi.first.units.MassUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.wpilibj.RobotBase

enum class BeamState{
    CLEAR,
    BLOCKED
}

object ConfigConstants{
    val POWER_DEADBAND = 0.0
}

object ElectronicIDs{
    const val DISPENSER_MOTOR_ID = -1
    const val BEAMBREAK_ID = -1

    const val ELEVATOR_LEFT_ID = -1
    const val ELEVATOR_RIGHT_ID = -1

    const val GROUNDINTAKE_MOTOR_ID = -1
    const val WRIST_MOTOR_ID = -1
    const val WRIST_TRUE_ENCODER = -1
}

object TuningConstants{
    const val ELEVATOR_P = 0.0
    const val ELEVATOR_D = 0.0

    //Wrist Values
    const val WRIST_CURRENT_LIMIT = 30

    const val WRIST_P = 0.0
    const val WRIST_I = 0.0
    const val WRIST_D = 0.0

    const val WRIST_S = 0.0
    const val WRIST_V = 0.0
    const val WRIST_G = 0.0
    const val WRIST_A = 0.0
}

object PhysicalConstants {
    const val WRIST_ENCODER_MULTIPLIER = 1.0
    const val WRIST_ENCODER_TRANSFORM = 0.1

    val robotMass = Pounds.of(115.0)
    val momentOfInertia = Units.KilogramSquareMeters.of(0.0)

}

object MetaConstants{
    enum class Mode{
        REAL,
        SIM,
        REPLAY
    }

    val simMode = Mode.REPLAY
    val currentMode = if (RobotBase.isReal()) Mode.REAL else simMode
}