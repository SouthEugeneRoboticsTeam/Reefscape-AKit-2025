package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.reefscape2025.SetpointConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import kotlin.math.*

class VisionAlign() : ReadJoysticks() {

    private val drivePID = PIDController(SwerveConstants.ALIGN_DRIVE_P, SwerveConstants.ALIGN_DRIVE_I, SwerveConstants.ALIGN_DRIVE_D)
    private val anglePID = PIDController(SwerveConstants.ALIGN_ROT_P, SwerveConstants.ALIGN_ROT_I, SwerveConstants.ALIGN_ROT_D)

    private var targetPose = Pose2d()


    private var xError = 0.0
    private var yError = 0.0

    private var angle = 0.0
    private var pidResult = 0.0
    private var angleResult = 0.0

    private var resetTarget = true

    private var accelLimitedChassisSpeeds = ChassisSpeeds()

    init{
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        targetPose = Drivetrain.getNearestTarget()
    }

    override fun execute() {
        if (super.joystickX() == 0.0 && super.joystickY()==0.0 && super.joystickZ()==0.0 && resetTarget){
            targetPose = Drivetrain.getNearestTarget()
        }
        xError = targetPose.x - Drivetrain.getPose().x
        yError = targetPose.y - Drivetrain.getPose().y

        angle = atan2(yError, xError)

        pidResult = drivePID.calculate(sqrt( xError.pow(2) + yError.pow(2)), 0.0)
        angleResult = anglePID.calculate(Drivetrain.getPose().rotation.radians, targetPose.rotation.radians)

        if (super.joystickX() == 0.0 && super.joystickY()==0.0 && super.joystickZ()==0.0) {
            accelLimitedChassisSpeeds = readChassisSpeeds(
                ChassisSpeeds(pidResult*cos(angle),pidResult*sin(angle), angleResult),
                MathUtil.interpolate(
                    SwerveConstants.DRIVE_ACCEL_FAST, SwerveConstants.DRIVE_ACCEL_SLOW,
                    Elevator.getPosition() / SetpointConstants.ELEVATOR_L4
                ),
                Rotation2d()
            )
        } else {
            accelLimitedChassisSpeeds = readJoysticks(
                Elevator.getAccelLimit(),
                Drivetrain.getPose().rotation.minus(super.inputRotOffset())
            )
        }
    }
}
