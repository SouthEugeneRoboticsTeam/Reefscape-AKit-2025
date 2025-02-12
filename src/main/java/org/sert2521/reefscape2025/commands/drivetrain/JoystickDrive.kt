package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.sert2521.reefscape2025.ConfigConstants
import org.sert2521.reefscape2025.Input
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import java.text.Normalizer
import kotlin.math.*
import kotlin.time.times

class JoystickDrive(private val fieldOriented:Boolean = true) : Command() {
    val joystickX = Input::getJoystickX
    val joystickY = Input::getJoystickY
    val joystickZ = Input::getJoystickZ

    val inputRotOffset = Input::getRotOffset



    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Drivetrain)
    }

    override fun execute() {
        var x = joystickX()
        var y = joystickY()



        val angle = atan2(y, x)


        //this math works just trust
        //it normalizes the magnitude from a square output range to a circular one
        //new magnitude is true to actual distance from the center


        val sqrMagnitude = x.pow(2) + y.pow(2)
        if (sqrMagnitude > 1) {
            val magnitude = sqrt(sqrMagnitude)
            x /= magnitude
            y /= magnitude
        } else if (sqrMagnitude <= ConfigConstants.POWER_DEADBAND.pow(2)) {
            x = 0.0
            y = 0.0
        }
        var newMagnitude = Translation2d(x, y).norm

        //Do joystick curve

        newMagnitude = newMagnitude.pow(3)

        //reconstructs the x and y components from polar coordinates
        val newX = cos(angle)*newMagnitude
        val newY = sin(angle)*newMagnitude

        if (joystickX() == 0.0 && joystickY() == 0.0 && joystickZ() == 0.0){
            Drivetrain.stop()
        }
        else if (fieldOriented) {
            Drivetrain.driveRobotOriented(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    newY * SwerveConstants.DRIVE_SPEED,
                    newX * SwerveConstants.DRIVE_SPEED,
                    joystickZ().pow(3) * SwerveConstants.ROT_SPEED,
                    Drivetrain.getPose().rotation.minus(inputRotOffset())
                )
            )
        }
        else {
            Drivetrain.driveRobotOriented(
                ChassisSpeeds(
                    newY * SwerveConstants.DRIVE_SPEED,
                    newX * SwerveConstants.DRIVE_SPEED,
                    joystickZ().pow(3) * SwerveConstants.ROT_SPEED
                )
            )
        }
    }
}
