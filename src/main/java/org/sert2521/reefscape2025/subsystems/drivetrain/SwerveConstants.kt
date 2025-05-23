package org.sert2521.reefscape2025.subsystems.drivetrain

import com.pathplanner.lib.config.PIDConstants
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat.N1
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import org.ejml.simple.SimpleMatrix
import kotlin.math.PI

object SwerveConstants {

    /* set value to IdleMode.kBrake when actually running drivetrain in match or on ground */
    /* set value to IdleMode.kCoast when testing, makes modules easier to turn by hand */
    val moduleIdleMode = IdleMode.kBrake

    val moduleZeroRotations = arrayOf(
        Rotation2d(-2.992),
        Rotation2d(-1.58),
        Rotation2d(-1.35),
        Rotation2d(-0.56)
    )

    val indexToCorner = mapOf(
        0 to "FL",
        1 to "FR",
        2 to "BL",
        3 to "BR"
    )

    val encoderIDs = arrayOf(1, 2, 3, 4)
    val driveIDs = arrayOf(5, 6, 7, 8)
    val turnIDs = arrayOf(9, 10, 11, 12)


    const val DRIVE_GEAR_RATIO = 6.75
    const val TURN_GEAR_RATIO = 21.4285714

    const val TURN_INVERTED = true
    const val TURN_REL_ENCODER_INVERTED = false

    const val DRIVE_CURRENT_LIMIT_TELE = 40
    const val TURN_CURRENT_LIMIT_TELE = 30

    const val DRIVE_CURRENT_LIMIT_AUTO = 0
    const val TURN_CURRENT_LIMIT_AUTO = 0


    const val WHEEL_RADIUS_METERS = 0.0508

    //Drive positions in radians
    const val DRIVE_CONVERSION_POSITION = 2 * PI / DRIVE_GEAR_RATIO
    const val DRIVE_CONVERSION_VELOCITY = DRIVE_CONVERSION_POSITION / 60.0

    const val TURN_REL_CONVERSION_POSITION = (2*PI) / TURN_GEAR_RATIO
    const val TURN_REL_CONVERSION_VELOCITY = TURN_REL_CONVERSION_POSITION

    const val DRIVE_P = 0.02
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.004

    const val DRIVE_KS = 0.1
    const val DRIVE_KV = 0.135
    const val DRIVE_KA = 0.0

    const val TURN_P = 1.0
    const val TURN_I = 0.0
    const val TURN_D = 0.2
    const val TURN_FF = 0.0
    const val TURN_PID_MIN_INPUT = 0.0
    const val TURN_PID_MAX_INPUT = 2 * PI

    val autoTranslationPID = PIDConstants(4.0, 0.0, 0.4)
    val autoRotationPID = PIDConstants(5.5, 0.00, 0.4, 0.7)

    const val VISION_ALIGN_DRIVE_P = 6.0
    const val VISION_ALIGN_DRIVE_I = 0.0
    const val VISION_ALIGN_DRIVE_D = 0.4

    const val VISION_ALIGN_DRIVE_V = 1.0

    const val VISION_ALIGN_ROT_P = 8.0
    const val VISION_ALIGN_ROT_I = 0.0
    const val VISION_ALIGN_ROT_D = 0.47

    const val STATION_ALIGN_ROT_P = 8.0
    const val STATION_ALIGN_ROT_I = 0.0
    const val STATION_ALIGN_ROT_D = 0.47

    val visionAlignProfile = TrapezoidProfile.Constraints(4.0, 2.5)

    const val TURN_ABS_ENCODER_CONVERSION_POSITION = 2 * PI


    const val ODOMETRY_PERIOD = 20
    const val ODOMETRY_FREQUENCY = 1000 / ODOMETRY_PERIOD



    const val MAX_SPEED_MPS = 4.571

    //As opposed to the max speed, this is the speed you actually want the drivetrain to go at
    const val DRIVE_SPEED_FAST = 4.0
    const val DRIVE_SPEED_SLOW = 0.5
    const val ROT_SPEED = 5.0

    const val DRIVE_ACCEL_FAST = 20.0
    const val DRIVE_DECCEL_FAST = 25.0

    const val DRIVE_ACCEL_SLOW = 8.0
    const val DRIVE_DECCEL_SLOW = 8.0


    val moduleTranslations = arrayOf(
        Translation2d(0.28829, 0.28829),
        Translation2d(0.28829, -0.28829),
        Translation2d(-0.28829, 0.28829),
        Translation2d(-0.28829, -0.28829)
    )

    const val WHEEL_COF = 1.54

    val driveMotorGearbox: DCMotor = DCMotor.getNEO(1).withReduction(DRIVE_GEAR_RATIO)

    const val FF_RAMP_RATE = 1.0

    val LIMELIGHT_STDV = VecBuilder.fill(1.0, 1.0, 1.0E99)
    val LIMELIGHT_STDV_YAW_RESET = VecBuilder.fill(0.7, 0.7, 1.0)
}