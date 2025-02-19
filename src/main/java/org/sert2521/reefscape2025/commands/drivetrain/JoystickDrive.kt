package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.ConfigConstants
import org.sert2521.reefscape2025.Input
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_ACCEL_FAST
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_ACCEL_SLOW
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import kotlin.math.*

class JoystickDrive(private val fieldOriented:Boolean = true) : Command() {
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

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Drivetrain)
    }

    // Oh man I bet this could be optimized but I'm eeeepy...
    override fun execute() {
        val x = joystickX()
        val y = joystickY()

        val angle = atan2(y, x)
        val sqrMagnitude = x.pow(2) + y.pow(2)

        val currAccelLimit = MathUtil.interpolate(DRIVE_ACCEL_FAST, DRIVE_ACCEL_SLOW,
            Elevator.getPosition()/SetpointConstants.ELEVATOR_L4)

        // Deadband + cubic curve
        val newMagnitude = sqrt(
            MathUtil.applyDeadband(sqrMagnitude, ConfigConstants.POWER_DEADBAND.pow(2))
        ).pow(3)

        //reconstructs the x and y components from polar coordinates
        val newX = cos(angle)*newMagnitude
        val newY = sin(angle)*newMagnitude

        val magChange = sqrt((lastX - newX).pow(2) + (lastY-newY).pow(2))

        //The fraction of the change in magnitude that should be applied
        var magFraction = 1.0

        // Applies maximum magnitude of change of x and y
        // (divide by 50 so that currAccel can be in m/s^2)
        if (magChange > currAccelLimit/50.0){
            magFraction = (currAccelLimit/50.0)/magChange
        }

        lastX = MathUtil.interpolate(lastX, newX, magFraction)
        lastY = MathUtil.interpolate(lastY, newY, magFraction)


        if (lastX == 0.0 && lastY == 0.0 && joystickZ() == 0.0){
            Drivetrain.stop()
        }
        else if (fieldOriented) {
            /* lastX and lastY are switched because robot coords put X as front
                and Y as left. In controller coords, Y is up and X is left. */
            Drivetrain.driveRobotOriented(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    lastY * SwerveConstants.DRIVE_SPEED,
                    lastX * SwerveConstants.DRIVE_SPEED,
                    joystickZ().pow(3) * SwerveConstants.ROT_SPEED,
                    Drivetrain.getPose().rotation.minus(inputRotOffset())
                )
            )
        }
        else {
            Drivetrain.driveRobotOriented(
                ChassisSpeeds(
                    lastY * SwerveConstants.DRIVE_SPEED,
                    lastX * SwerveConstants.DRIVE_SPEED,
                    joystickZ().pow(3) * SwerveConstants.ROT_SPEED
                )
            )
        }
    }
}
