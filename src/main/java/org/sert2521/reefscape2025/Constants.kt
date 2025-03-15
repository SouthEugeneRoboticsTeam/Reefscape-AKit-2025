package org.sert2521.reefscape2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

object SetpointConstants{
    const val WRIST_INIT = 0.28
    const val WRIST_STOW = 0.23
    const val WRIST_GROUND = -0.09
    const val WRIST_L1 = 0.17
    const val WRIST_ALGAE_LOW = 0.04
    const val WRIST_ALGAE_HIGH = 0.12

    const val ELEVATOR_STOW = 0.005
    const val ELEVATOR_L2 = 0.15
    const val ELEVATOR_L3 = 0.345
    const val ELEVATOR_L4 = 0.67-0.01
    const val ELEVATOR_ALGAE_LOW = 0.195
    const val ELEVATOR_ALGAE_HIGH = 0.415

    //algae high 0.42
    //algae low 0.226


    const val DISPENSER_INTAKE_SPEED = 0.5
    const val DISPENSER_OUTTAKE_SPEED = 0.3
    const val DISPENSER_OUTTAKE_L4 = 0.3
    const val DISPENSER_OUTTAKE_SLOW_SPEED = 0.2
    const val DISPENSER_STOP_VOLTAGE = -0.01
    const val DISPENSER_RECENTER_SPEED_FORWARD = 0.1
    const val DISPENSER_RECENTER_SPEED_BACKWARD = -0.55

    const val RAMP_INTAKE_SPEED = 0.8
    const val RAMP_RECENTER_SPEED = -0.2

    const val GROUND_INTAKE_SPEED = 0.9
    const val GROUND_OUTTAKE_SPEED_ALGAE = -0.5
    const val GROUND_OUTTAKE_SPEED_CORAL = -0.2
    const val GROUND_HOLD_ALGAE_SPEED = -0.2
}

object ConfigConstants{
    const val POWER_DEADBAND = 0.0
}

object ElectronicIDs{
    const val DISPENSER_MOTOR_ID = 15
    const val BEAMBREAK_DISPENSER = 9
    const val BEAMBREAK_RAMP = 0

    const val RAMP_MOTOR_ID = 18

    const val ELEVATOR_LEFT_ID = 13
    const val ELEVATOR_RIGHT_ID = 14

    const val GROUNDINTAKE_MOTOR_ID = 17
    const val WRIST_MOTOR_ID = 16
    const val WRIST_ABS_ENCODER = 18

    const val LASER_ID = 19
}

object TuningConstants{
    const val ELEVATOR_P = 4.0
    const val ELEVATOR_I = 0.0
    const val ELEVATOR_D = 0.1
    val ELEVATOR_PROFILE = TrapezoidProfile.Constraints(1.0, 2.0)

    const val ELEVATOR_S = 0.05

    const val ELEVATOR_V = 10.23
    const val ELEVATOR_G = 0.3
    const val ELEVATOR_A = 0.0

    //Wrist Values
    const val WRIST_CURRENT_LIMIT = 30

    const val WRIST_P = 7.0
    const val WRIST_I = 0.0
    const val WRIST_D = 0.2

    const val WRIST_S = 0.0
    const val WRIST_V = 0.0
    const val WRIST_G = 0.0
    const val WRIST_A = 0.0
}

object PhysicalConstants {
    const val WRIST_ABS_ENCODER_ZERO = 0.95
    const val WRIST_MOTOR_ENCODER_MULTIPLIER = 3.0/200

    const val ELEVATOR_MOTOR_ENCODER_MULTIPLIER = 0.02328333333/2.0

    val robotMass = Pounds.of(115.0)
    val momentOfInertia = Units.KilogramSquareMeters.of(0.0)
}

object VisionTargetPositions {
    private const val CX = 4.49
    private const val CY = 4.03
    private const val HD = PI/3
    private const val RADIUS = 1.255
    private const val SHIFT = 0.16

    val reefPositionsLeft = mutableListOf(
        //Left I think
        Pose2d(CX - cos(6*HD) * RADIUS - sin(6*HD) * SHIFT,
            CY - sin(6*HD) * RADIUS + cos(6*HD) * SHIFT, Rotation2d(6* HD)),
        Pose2d(CX - cos(5*HD) * RADIUS - sin(5*HD) * SHIFT,
            CY - sin(5*HD) * RADIUS + cos(5*HD) * SHIFT, Rotation2d(5* HD)),
        Pose2d(CX - cos(4*HD) * RADIUS - sin(4*HD) * SHIFT,
            CY - sin(4*HD) * RADIUS + cos(4*HD) * SHIFT, Rotation2d(4* HD)),
        Pose2d(CX - cos(3*HD) * RADIUS - sin(3*HD) * SHIFT,
            CY - sin(3*HD) * RADIUS + cos(3*HD) * SHIFT, Rotation2d(3* HD)),
        Pose2d(CX - cos(2*HD) * RADIUS - sin(2*HD) * SHIFT,
            CY - sin(2*HD) * RADIUS + cos(2*HD) * SHIFT, Rotation2d(2* HD)),
        Pose2d(CX - cos(1*HD) * RADIUS - sin(1*HD) * SHIFT,
            CY - sin(1*HD) * RADIUS + cos(1*HD) * SHIFT, Rotation2d(1* HD)),
    )

    val reefPositionsRight = mutableListOf(
        //right I think
        Pose2d(CX - cos(1*HD) * RADIUS + sin(1*HD) * SHIFT,
            CY - sin(1*HD) * RADIUS - cos(1*HD) * SHIFT, Rotation2d(1* HD)),
        Pose2d(CX - cos(2*HD) * RADIUS + sin(2*HD) * SHIFT,
            CY - sin(2*HD) * RADIUS - cos(2*HD) * SHIFT, Rotation2d(2* HD)),
        Pose2d(CX - cos(3*HD) * RADIUS + sin(3*HD) * SHIFT,
            CY - sin(3*HD) * RADIUS - cos(3*HD) * SHIFT, Rotation2d(3* HD)),
        Pose2d(CX - cos(4*HD) * RADIUS + sin(4*HD) * SHIFT,
            CY - sin(4*HD) * RADIUS - cos(4*HD) * SHIFT, Rotation2d(4* HD)),
        Pose2d(CX - cos(5*HD) * RADIUS + sin(5*HD) * SHIFT,
            CY - sin(5*HD) * RADIUS - cos(5*HD) * SHIFT, Rotation2d(5* HD)),
        Pose2d(CX - cos(6*HD) * RADIUS + sin(6*HD) * SHIFT,
            CY - sin(6*HD) * RADIUS - cos(6*HD) * SHIFT, Rotation2d(6* HD)),
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
