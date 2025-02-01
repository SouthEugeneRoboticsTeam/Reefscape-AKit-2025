package org.sert2521.reefscape2025.subsystems.drivetrain

import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import kotlin.math.PI

object SwerveConstants {
    val moduleZeroRotations = arrayOf(Rotation2d(), Rotation2d(), Rotation2d(), Rotation2d())

    val driveIDs = arrayOf(-1, -1, -1, -1)
    val turnIDs = arrayOf(-1, -1, -1, -1)
    val encoderIDs = arrayOf(-1, -1, -1, -1)

    val driveGearRatio = 6.75
    val turnGearRatio = 21.4285714

    const val TURN_INVERTED = false
    const val TURN_REL_ENCODER_INVERTED = false

    const val DRIVE_CURRENT_LIMIT_TELE = 0
    const val TURN_CURRENT_LIMIT_TELE = 0

    const val DRIVE_CURRENT_LIMIT_AUTO = 0
    const val TURN_CURRENT_LIMIT_AUTO = 0



    const val DRIVE_CONVERSION_POSITION = PI * 0.1016 / 6.75
    const val DRIVE_CONVERSION_VELOCITY = DRIVE_CONVERSION_POSITION / 60.0

    const val TURN_REL_CONVERSION_POSITION = (2*PI)/21.4285714
    const val TURN_REL_CONVERSION_VELOCITY = TURN_REL_CONVERSION_POSITION

    //FF is the same as V for SVA controller
    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0
    const val DRIVE_FF = 0.0

    const val DRIVE_KS = 0.0

    const val TURN_P = 0.0
    const val TURN_I = 0.0
    const val TURN_D = 0.0
    const val TURN_FF = 0.0
    const val TURN_PID_MIN_INPUT = 0.0
    const val TURN_PID_MAX_INPUT = 2 * PI

    const val TURN_ABS_ENCODER_CONVERSION_POSITION = 2 * PI

    const val ODOMETRY_FREQUENCY = 50
    const val ODOMETRY_PERIOD = 1000 / ODOMETRY_FREQUENCY


    const val WHEEL_RADIUS_METERS = 0.0508

    const val MAX_SPEED_MPS = 4.571

    const val DRIVE_SPEED = 3.0
    const val ROT_SPEED = 4.0


    val moduleTranslations = arrayOf(
        Translation2d(),
        Translation2d(),
        Translation2d(),
        Translation2d()
    )

    val autoTranslationPID = PIDConstants(0.0, 0.0, 0.0)
    val autoRotationPID = PIDConstants(0.0, 0.0, 0.0)

    const val WHEEL_COF = 1.54

    val driveMotorGearbox = DCMotor.getNEO(1).withReduction(6.75)
}