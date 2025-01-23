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
}