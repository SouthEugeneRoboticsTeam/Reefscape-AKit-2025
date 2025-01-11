package org.sert2521.reefscape2025.subsystems.drive

import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.PI

object SwerveConstants {
    val moduleZeroRotations = arrayOf(Rotation2d(), Rotation2d(), Rotation2d(), Rotation2d())

    val driveIDs = arrayOf(-1, -1, -1, -1)
    val turnIDs = arrayOf(-1, -1, -1, -1)

    const val TURN_INVERTED = false
    const val TURN_REL_ENCODER_INVERTED = false

    const val DRIVE_CURRENT_LIMIT = 0
    const val TURN_CURRENT_LIMIT = 0



    const val DRIVE_CONVERSION_POSITION = PI * 0.1016 / 6.75
    const val DRIVE_CONVERSION_VELOCITY = DRIVE_CONVERSION_POSITION / 60.0

    const val TURN_REL_CONVERSION_POSITION = (2*PI)/21.4285714
    const val TURN_REL_CONVERSION_VELOCITY = TURN_REL_CONVERSION_POSITION

    //FF is the same as V for SVA controller
    const val DRIVE_P = 0.0
    const val DRIVE_I = 0.0
    const val DRIVE_D = 0.0
    const val DRIVE_FF = 0.0

    const val TURN_P = 0.0
    const val TURN_I = 0.0
    const val TURN_D = 0.0
    const val TURN_FF = 0.0

    const val ODOMETRY_FREQUENCY = 20

    var oil = 0.0

}