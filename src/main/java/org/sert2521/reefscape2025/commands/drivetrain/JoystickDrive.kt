package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.Input
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain

class JoystickDrive(private val fieldOriented: Boolean = true) : Command() {

    private var joystickAccelLimited = ChassisSpeeds()

    // Requirements are already passed through the ReadJoysticks class

    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        if (fieldOriented) {
            joystickAccelLimited = AccelLimiterUtil.readJoysticks(
                Input.getAccelLimit(), AccelLimiterUtil.inputRotOffset(),
                Input.getDeccelLimit(), Input.getSpeedLimit()
            )

            if (joystickAccelLimited.vxMetersPerSecond == 0.0 && joystickAccelLimited.vyMetersPerSecond == 0.0 && joystickAccelLimited.omegaRadiansPerSecond == 0.0) {
                Drivetrain.stop()
            }
            Drivetrain.driveRobotOriented(
                joystickAccelLimited, false
            )
        } else {
            joystickAccelLimited = AccelLimiterUtil.readJoysticks(
                Input.getAccelLimit(), Rotation2d(),
                Input.getDeccelLimit(), Input.getSpeedLimit(), false
            )

            if (joystickAccelLimited.vxMetersPerSecond == 0.0 && joystickAccelLimited.vyMetersPerSecond == 0.0 && joystickAccelLimited.omegaRadiansPerSecond == 0.0) {
                Drivetrain.stop()
            }

            Logger.recordOutput("Input/Joystick Accel Limited", joystickAccelLimited)

            Drivetrain.driveRobotOriented(
                joystickAccelLimited, false
            )
        }
    }
}
