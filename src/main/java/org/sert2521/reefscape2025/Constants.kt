package org.sert2521.reefscape2025

import edu.wpi.first.wpilibj.RobotBase

enum class BeamState{
    CLEAR,
    BLOCKED
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

    const val WRIST_CURRENT_LIMIT = 30

    const val WRIST_P = 0.0
    const val WRIST_I = 0.0
    const val WRIST_D = 0.0
}

object PhysicalConstants {
    const val WRIST_ENCODER_MULTIPLIER = 1.0
    const val WRIST_ENCODER_TRANSFORM = 0.1
}