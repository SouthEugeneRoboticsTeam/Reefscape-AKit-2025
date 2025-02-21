package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.ConfigConstants
import org.sert2521.reefscape2025.Input
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import kotlin.math.*

open class ReadJoysticks : Command() {
    /*
    This math just works, trust.
    It's a two dimensional slew rate limiter with a changing rate based on the
    value of [currAccelLimit] in m/s^2 (or at least in the same units as x and y).
    Basically it lets you have changing acceleration limits based on outside variables
    like elevator height, arm angle, button presses, etc.
    The acceleration limit you want for a cycle should be encoded into [currAccelLimit].
    If you want a constant acceleration limit just set that to a constant.
    Otherwise you can set it to whatever function produces your wanted acceleration limit.
    The Genius Kai Dassonville strikes again
     */

    val joystickX = Input::getJoystickX
    val joystickY = Input::getJoystickY
    val joystickZ = Input::getJoystickZ

    val inputRotOffset = Input::getRotOffset

    private var lastX = 0.0
    private var lastY = 0.0

    private var x = 0.0
    private var y = 0.0

    private var angle = 0.0
    private var sqrMagnitude = 0.0

    private var newMagnitude = 0.0

    private var cubicChassisSpeeds = ChassisSpeeds()

    private var magChange = 0.0
    private var magFraction = 0.0

    init{
        addRequirements(Elevator)
    }

    // Oh man I bet this could be optimized but I'm eeeepy...
    fun readJoysticks(accelLimit:Double, rotOffset: Rotation2d):ChassisSpeeds{
        /**
         * Returns the desired directions as a triplet of X, Y, and Rotation in robot coordinates
         * Should be called periodically.
         */
        x = joystickX()
        y = joystickY()

        angle = atan2(y, x)
        sqrMagnitude = x.pow(2) + y.pow(2)

        // Deadband + cubic curve
        newMagnitude = sqrt(
            MathUtil.applyDeadband(sqrMagnitude, ConfigConstants.POWER_DEADBAND.pow(2))
        ).pow(3)

        // X and Y are swapped because Y is left in robot coordinates and X is up
        // It's the other way around in controller coordinates
        cubicChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            sin(angle) * newMagnitude * SwerveConstants.DRIVE_SPEED,
            cos(angle) * newMagnitude * SwerveConstants.DRIVE_SPEED,
            joystickZ().pow(3) * SwerveConstants.ROT_SPEED,
            Drivetrain.getPose().rotation.minus(rotOffset)
        )

        // Total magnitude of change since last cycle
        // NOTE: NOT change of total magnitude: it is magnitude of change in 2D coordinates
        magChange = sqrt((lastX - cubicChassisSpeeds.vxMetersPerSecond).pow(2) + (lastY-cubicChassisSpeeds.vyMetersPerSecond).pow(2))

        // The fraction of the change in magnitude that should be applied
        magFraction = 1.0

        // Applies maximum magnitude of change of x and y
        // (divide by 50 so that currAccel can be in m/s^2)
        if (magChange > accelLimit/50.0){
            magFraction = (accelLimit/50.0)/magChange
        }

        lastX = MathUtil.interpolate(lastX, cubicChassisSpeeds.vxMetersPerSecond, magFraction)
        lastY = MathUtil.interpolate(lastY, cubicChassisSpeeds.vyMetersPerSecond, magFraction)

        return ChassisSpeeds(lastX, lastY, cubicChassisSpeeds.omegaRadiansPerSecond)
    }

    fun readChassisSpeeds(fieldChassisSpeeds: ChassisSpeeds,
                          accelLimit: Double, totalRotOffset:Rotation2d):ChassisSpeeds{
        /**
         * Same as [readJoysticks] but has the speeds passed in through ChassisSpeeds.
         * Outputs calculated ChassisSpeeds once the acceleration limit has been calculated
         */

        cubicChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldChassisSpeeds,
            totalRotOffset
        )

        // Total magnitude of change since last cycle
        // NOTE: NOT change of total magnitude: it is magnitude of change in 2D coordinates
        magChange = sqrt((lastX - cubicChassisSpeeds.vxMetersPerSecond).pow(2) + (lastY-cubicChassisSpeeds.vyMetersPerSecond).pow(2))

        // The fraction of the change in magnitude that should be applied
        magFraction = 1.0

        // Applies maximum magnitude of change of x and y
        // (divide by 50 so that currAccel can be in m/s^2)
        if (magChange > accelLimit/50.0){
            magFraction = (accelLimit/50.0)/magChange
        }

        lastX = MathUtil.interpolate(lastX, cubicChassisSpeeds.vxMetersPerSecond, magFraction)
        lastY = MathUtil.interpolate(lastY, cubicChassisSpeeds.vyMetersPerSecond, magFraction)

        // X and Y are swapped because Y is left in robot coordinates and X is up
        // It's the other way around in controller coordinates
        return ChassisSpeeds(lastY, lastX, fieldChassisSpeeds.omegaRadiansPerSecond)
    }
}
