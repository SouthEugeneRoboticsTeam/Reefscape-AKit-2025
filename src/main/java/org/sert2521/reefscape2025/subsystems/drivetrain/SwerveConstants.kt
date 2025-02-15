package org.sert2521.reefscape2025.subsystems.drivetrain

import com.pathplanner.lib.config.PIDConstants
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import kotlin.math.PI

object SwerveConstants {

    //set value to IdleMode.kBrake when actually running drivetrain
    //set value to IdleMode.kCoast when testing, makes modules easier to turn by hand
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

    //FF is the same as V for SVA controller
    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0
//kS: 0.27859
//kV: 0.13335
    //kS: 0.29108
// kV: 0.13158
    const val DRIVE_KS = 0.29108
    const val DRIVE_KV = 0.13158
    const val DRIVE_KA = 0.0

    const val TURN_P = 1.0
    const val TURN_I = 0.0
    const val TURN_D = 0.2
    const val TURN_FF = 0.0
    const val TURN_PID_MIN_INPUT = 0.0
    const val TURN_PID_MAX_INPUT = 2 * PI

    const val TURN_ABS_ENCODER_CONVERSION_POSITION = 2 * PI


    const val ODOMETRY_PERIOD = 5
    const val ODOMETRY_FREQUENCY = 1000 / ODOMETRY_PERIOD



    const val MAX_SPEED_MPS = 4.571

    const val DRIVE_SPEED = 3.0
    const val ROT_SPEED = 4.0


    val moduleTranslations = arrayOf(
        Translation2d(11.35, 11.35),
        Translation2d(11.35, -11.35),
        Translation2d(-11.35, 11.35),
        Translation2d(-11.35, -11.35)
    )

    val autoTranslationPID = PIDConstants(0.0, 0.0, 0.0)
    val autoRotationPID = PIDConstants(0.0, 0.0, 0.0)

    const val WHEEL_COF = 1.54

    val driveMotorGearbox: DCMotor = DCMotor.getNEO(1).withReduction(DRIVE_GEAR_RATIO)

    const val FF_RAMP_RATE = 1.0

    val LIMELIGHT_STDV = VecBuilder.fill(0.7, 0.7, 9999999.9)
}