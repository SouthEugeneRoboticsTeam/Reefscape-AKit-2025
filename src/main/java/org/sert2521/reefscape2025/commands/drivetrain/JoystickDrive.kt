package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import kotlin.math.*

class JoystickDrive(private val fieldOriented:Boolean = true) : ReadJoysticks() {

    private var joystickAccelLimited = ChassisSpeeds()

    // Requirements are already passed through the ReadJoysticks class

    override fun execute() {

        if (joystickAccelLimited.vxMetersPerSecond == 0.0 && joystickAccelLimited.vyMetersPerSecond == 0.0 && joystickAccelLimited.omegaRadiansPerSecond == 0.0){
            Drivetrain.stop()
        }
        else if (fieldOriented) {
            joystickAccelLimited = readJoysticks(Elevator.getAccelLimit(), super.inputRotOffset())

            Drivetrain.driveRobotOriented(
                joystickAccelLimited
            )
        }
        else {
            joystickAccelLimited = readJoysticks(Elevator.getAccelLimit(), Rotation2d())

            Drivetrain.driveRobotOriented(
                joystickAccelLimited
            )
        }
    }
}
