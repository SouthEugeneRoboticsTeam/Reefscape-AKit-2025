package org.sert2521.reefscape2025

import edu.wpi.first.wpilibj.RobotBase

object MetaConstants{
    enum class Mode{
        REAL,
        SIM,
        REPLAY
    }

    val simMode = Mode.REPLAY
    val currentMode = if (RobotBase.isReal()) Mode.REAL else simMode


}