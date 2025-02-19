package org.sert2521.reefscape2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI

object SetpointConstants{
    const val WRIST_STOW = 0.0
    const val WRIST_GROUND = 0.0
    const val WRIST_L1 = 0.0
    const val WRIST_ALGAE = 0.0

    const val ELEVATOR_STOW = 0.05
    const val ELEVATOR_L2 = 0.17
    const val ELEVATOR_L3 = 0.37
    const val ELEVATOR_L4 = 0.67-0.1
    const val ELEVATOR_ALGAE = 0.25

    const val DISPENSER_INTAKE_SPEED = 0.55
    const val DISPENSER_OUTTAKE_SPEED = 0.65
    const val DISPENSER_OUTTAKE_L4 = 0.5
    const val DISPENSER_OUTTAKE_SLOW_SPEED = 0.2
    const val DISPENSER_STOP_VOLTAGE = -0.01
    const val DISPENSER_RECENTER_SPEED = 0.1

    const val GROUND_INTAKE_SPEED = 0.7
    const val GROUND_OUTTAKE_SPEED = -0.5
}

object ConfigConstants{
    const val POWER_DEADBAND = 0.0
}

object ElectronicIDs{
    const val DISPENSER_MOTOR_ID = 15
    const val BEAMBREAK_DISPENSER = 9
    const val BEAMBREAK_RAMP = 1

    const val ELEVATOR_LEFT_ID = 13
    const val ELEVATOR_RIGHT_ID = 14

    const val GROUNDINTAKE_MOTOR_ID = 17
    const val WRIST_MOTOR_ID = 16
    const val WRIST_ABS_ENCODER = 18

    const val LASER_ID = 19
}

object DrivetrainConstants {
    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0

    const val ANGLE_P = 0.0
    const val ANGLE_I = 0.0
    const val ANGLE_D = 0.0
}

object TuningConstants{
    const val ELEVATOR_P = 1.0
    const val ELEVATOR_I = 0.0
    const val ELEVATOR_D = 0.0
    val ELEVATOR_PROFILE = TrapezoidProfile.Constraints(0.1, 0.5)

    const val ELEVATOR_S = 0.05

    const val ELEVATOR_V = 5.115
    const val ELEVATOR_G = 0.3
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

object VisionTargetPositions {
    val reefPositions = mutableListOf(
        Pose2d(3.2, 4.19, Rotation2d(0.0)),
        Pose2d(3.2, 3.86, Rotation2d(0.0)),

        Pose2d(3.7, 2.99, Rotation2d(PI /3)),
        Pose2d(3.99, 2.83, Rotation2d(PI /3)),

        Pose2d(4.99, 2.83, Rotation2d((2.0* PI)/3.0)),
        Pose2d(5.28, 2.98, Rotation2d((2.0* PI)/3.0)),

        Pose2d(5.78, 3.86, Rotation2d(PI)),
        Pose2d(5.78, 4.19, Rotation2d(PI)),

        Pose2d(5.28, 5.07, Rotation2d((-2.0* PI)/3.0)),
        Pose2d(4.99, 5.23, Rotation2d((-2.0* PI)/3.0)),

        Pose2d(3.99, 5.23, Rotation2d(-PI /3.0)),
        Pose2d(3.70, 5.07, Rotation2d(-PI /3.0)),
    )

}

object MetaConstants{
    enum class Mode{
        REAL,
        SIM,
        REPLAY
    }

    val atEvent = false
    private val simMode = Mode.REPLAY
    val currentMode = if (RobotBase.isReal()) Mode.REAL else simMode
}
