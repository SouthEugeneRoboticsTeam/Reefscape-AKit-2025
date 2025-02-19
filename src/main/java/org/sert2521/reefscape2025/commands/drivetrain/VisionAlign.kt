package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.reefscape2025.DrivetrainConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import kotlin.math.*

class VisionAlign(): Command() {

    private val drivePID = PIDController(DrivetrainConstants.DRIVE_P, DrivetrainConstants.DRIVE_I, DrivetrainConstants.DRIVE_D)
    private val anglePID = PIDController(DrivetrainConstants.ANGLE_P, DrivetrainConstants.ANGLE_I, DrivetrainConstants.ANGLE_D)

    private var xTarget = 0.0
    private var yTarget = 0.0

    private var xError = 0.0
    private var yError = 0.0
    private var angleError = 0.0
    private var error = 0.0

    private var angleTarget = 0.0

    private var angle = 0.0
    private var pidResult = 0.0
    private var xResult = 0.0
    private var yResult = 0.0
    private var angleResult = 0.0

    init{
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        xTarget = Drivetrain.getNearestTarget().x
        yTarget = Drivetrain.getNearestTarget().y
        angleTarget = Drivetrain.getNearestTarget().rotation.radians

    }

    override fun execute() {

        xError = xTarget - Drivetrain.getPose().x
        yError = yTarget - Drivetrain.getPose().y
        angleError = angleTarget - Drivetrain.getPose().rotation.radians

        angle = atan2(yError, xError)
        error = sqrt( xError.pow(2) + yError.pow(2) )

        pidResult = drivePID.calculate(error, 0.0)
        angleResult = anglePID.calculate(Drivetrain.getPose().rotation.radians, angleTarget)

        xResult = pidResult * cos(angle)
        yResult = pidResult * sin(angle)

        Drivetrain.driveRobotOriented(ChassisSpeeds(xResult, -yResult, angleResult))

    }
}
