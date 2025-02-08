package org.sert2521.reefscape2025

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.wpilibj.RobotBase

object SetpointConstants{
    const val WRIST_STOW = 0.0
    const val WRIST_GROUND = 0.0
    const val WRIST_L1 = 0.0

    const val ELEVATOR_STOW = 0.0
    const val ELEVATOR_L2 = 0.0
    const val ELEVATOR_L3 = 0.0
    const val ELEVATOR_L4 = 0.0
    const val ELEVATOR_ALGAE = 0.0

    const val INTAKE_SPEED = 0.4
}

object ConfigConstants{
    const val POWER_DEADBAND = 0.0
}

object ElectronicIDs{
    const val DISPENSER_MOTOR_ID = -1
    const val BEAMBREAK_DISPENSER = -1
    const val BEAMBREAK_RAMP = -1

    const val ELEVATOR_LEFT_ID = -1
    const val ELEVATOR_RIGHT_ID = -1

    const val GROUNDINTAKE_MOTOR_ID = -1
    const val WRIST_MOTOR_ID = -1
    const val WRIST_ABS_ENCODER = -1

    const val LASER_ID = -1
}

object TuningConstants{
    const val ELEVATOR_P = 0.0
    const val ELEVATOR_I = 0.0
    const val ELEVATOR_D = 0.0
    val ELEVATOR_PROFILE = TrapezoidProfile.Constraints(0.0, 0.0)

    const val ELEVATOR_S = 0.0
    const val ELEVATOR_V = 0.0
    const val ELEVATOR_G = 0.0
    const val ELEVATOR_A = 0.0

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
    const val WRIST_MOTOR_MULTIPLIER = 1.0

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