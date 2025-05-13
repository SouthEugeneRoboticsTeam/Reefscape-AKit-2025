package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.ConfigConstants
import org.sert2521.reefscape2025.Input
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import kotlin.math.*

object AccelLimiterUtil {
    /*
    This math just works, trust.
    It's a two dimensional slew rate limiter with a changing rate based on the
    value of [currAccelLimit] in m/s^2 (or at least in the same units as x and y).
    Basically it lets you have changing acceleration limits based on outside variables
    like elevator height, arm angle, button presses, etc.
    The acceleration limit you want for a cycle should be encoded into [currAccelLimit].
    If you want a constant acceleration limit just set that to a constant.
    Otherwise you can set it to whatever function produces your wanted acceleration limit.
    I tried to program it a different way but apparently this is the best
    (The Genius Kai Dassonville strikes again)
     */

    /*
    Lmao just realized this is super flawed because the limits are done in robot space
    instead of field space where forces actually matter
     */

    val joystickX = Input::getJoystickX
    val joystickY = Input::getJoystickY
    val joystickZ = Input::getJoystickZ

    val inputRotOffset = Input::getRotOffset

    var lastX = 0.0
    var lastY = 0.0

    private var x = 0.0
    private var y = 0.0

    private var lastChassisSpeeds = ChassisSpeeds()

    private var angle = 0.0
    private var sqrMagnitude = 0.0

    private var newMagnitude = 0.0

    private var curvedChassisSpeeds = ChassisSpeeds()

    private var magChange = 0.0
    private var magFraction = 0.0

    private var appliedAccelLimit = 0.0

    // Oh man I bet this could be optimized but I'm eeeepy...
    fun readJoysticks(accelLimit:Double, rotOffset: Rotation2d,
                      deccelLimit:Double = accelLimit,
                      maxSpeed:Double = SwerveConstants.MAX_SPEED_MPS, fieldOriented:Boolean=true):ChassisSpeeds{
        /**
         * Returns the desired directions as a triplet of X, Y, and Rotation in robot coordinates
         * Should be called periodically.
         */
        x = joystickX()
        y = joystickY()

        if (x==0.0 && y==0.0){
            appliedAccelLimit = deccelLimit
        } else {
            appliedAccelLimit = accelLimit
        }

        angle = atan2(y, x)
        sqrMagnitude = x.pow(2) + y.pow(2)

        // Deadband + cubic curve
        newMagnitude = sqrt(
            MathUtil.applyDeadband(sqrMagnitude, ConfigConstants.POWER_DEADBAND.pow(2))
        ).pow(3)


        if (fieldOriented){
            curvedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                sin(angle) * newMagnitude * maxSpeed,
                cos(angle) * newMagnitude * maxSpeed,
                joystickZ().pow(3) * SwerveConstants.ROT_SPEED,
                Drivetrain.getPose().rotation.minus(rotOffset)
            )
        } else {
            curvedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                sin(angle) * newMagnitude * maxSpeed,
                cos(angle) * newMagnitude * maxSpeed,
                joystickZ().pow(3) * SwerveConstants.ROT_SPEED,
                Rotation2d()
            )
        }
        // X and Y are swapped because Y is left in robot coordinates and X is up
        // It's the other way around in controller coordinates


        // Total magnitude of change since last cycle
        // NOTE: NOT change of total magnitude: it is magnitude of change in 2D coordinates
        magChange = sqrt((lastChassisSpeeds.vxMetersPerSecond - curvedChassisSpeeds.vxMetersPerSecond).pow(2) + (lastChassisSpeeds.vyMetersPerSecond-curvedChassisSpeeds.vyMetersPerSecond).pow(2))

        // The fraction of the change in magnitude that should be applied
        magFraction = 1.0

        // Applies maximum magnitude of change of x and y
        // (divide by 50 so that currAccel can be in m/s^2)
        if (magChange > appliedAccelLimit/50.0){
            magFraction = (appliedAccelLimit/50.0)/magChange
        }

        lastChassisSpeeds.vxMetersPerSecond = MathUtil.interpolate(lastChassisSpeeds.vxMetersPerSecond, curvedChassisSpeeds.vxMetersPerSecond, magFraction)
        lastChassisSpeeds.vyMetersPerSecond = MathUtil.interpolate(lastChassisSpeeds.vyMetersPerSecond, curvedChassisSpeeds.vyMetersPerSecond, magFraction)
        lastChassisSpeeds.omegaRadiansPerSecond = curvedChassisSpeeds.omegaRadiansPerSecond

        return lastChassisSpeeds
    }

    fun accelLimitChassisSpeeds(fieldChassisSpeeds: ChassisSpeeds,
                                accelLimit: Double, rotOffset:Rotation2d):ChassisSpeeds{
        /**
         * Same as [readJoysticks] but has the speeds passed in through ChassisSpeeds.
         * Outputs calculated ChassisSpeeds once the acceleration limit has been calculated
         */

        curvedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldChassisSpeeds,
            Drivetrain.getPose().rotation.minus(rotOffset)
        )

        // Total magnitude of change since last cycle
        // NOTE: NOT change of total magnitude: it is magnitude of change in 2D coordinates
        magChange = sqrt((lastChassisSpeeds.vxMetersPerSecond - curvedChassisSpeeds.vxMetersPerSecond).pow(2) + (lastChassisSpeeds.vyMetersPerSecond-curvedChassisSpeeds.vyMetersPerSecond).pow(2))

        // The fraction of the change in magnitude that should be applied
        magFraction = 1.0

        // Applies maximum magnitude of change of x and y
        // (divide by 50 so that currAccel can be in m/s^2)
        if (magChange > accelLimit/50.0){
            Logger.recordOutput("Accel Limited", true)
            magFraction = (accelLimit/50.0)/magChange
        } else {
            Logger.recordOutput("Accel Limited", false)
        }

        lastChassisSpeeds.vxMetersPerSecond = MathUtil.interpolate(lastChassisSpeeds.vxMetersPerSecond, curvedChassisSpeeds.vxMetersPerSecond, magFraction)
        lastChassisSpeeds.vxMetersPerSecond = MathUtil.interpolate(lastChassisSpeeds.vyMetersPerSecond, curvedChassisSpeeds.vyMetersPerSecond, magFraction)
        lastChassisSpeeds.omegaRadiansPerSecond = fieldChassisSpeeds.omegaRadiansPerSecond

        // X and Y are swapped because Y is left in robot coordinates and X is up
        // It's the other way around in controller coordinates
        return lastChassisSpeeds
    }
}
