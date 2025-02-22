package org.sert2521.reefscape2025.commands.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger
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
    private var driveResult = 0.0
    private var angleResult = 0.0

    private var accelLimitedChassisSpeeds = ChassisSpeeds()

    init{
        addRequirements(Drivetrain)
        anglePID.enableContinuousInput(PI, -PI)
    }

    override fun initialize() {
        targetPose = Drivetrain.getNearestTarget()
    }

    override fun execute() {
        targetPose = Pose2d(4.99, 5.23, Rotation2d((-2.0* PI)/3.0))
        xError = Drivetrain.getPose().x - targetPose.x
        yError = Drivetrain.getPose().y - targetPose.y

        angle = -atan2(yError, xError)

        driveResult = drivePID.calculate(sqrt( xError.pow(2) + yError.pow(2)), 0.0)
        angleResult = anglePID.calculate(Drivetrain.getPose().rotation.radians, targetPose.rotation.radians)

        if (super.joystickX() == 0.0 && super.joystickY()==0.0) {
            accelLimitedChassisSpeeds = readChassisSpeeds(
                ChassisSpeeds(driveResult* cos(angle),-driveResult*sin(angle), angleResult),
                MathUtil.interpolate(
                    SwerveConstants.DRIVE_ACCEL_FAST, SwerveConstants.DRIVE_ACCEL_SLOW,
                    Elevator.getPosition() / SetpointConstants.ELEVATOR_L4
                ),
                Rotation2d()
            )
        } else {
            accelLimitedChassisSpeeds = readJoysticks(
                Elevator.getAccelLimit(),
                super.inputRotOffset()
            )
        }
        Logger.recordOutput("AnglePID Result", driveResult*sin(angle))
        Logger.recordOutput("DrivePID Result", driveResult*cos(angle))
        Logger.recordOutput("x error", xError)
        Logger.recordOutput("y error", yError)
        Logger.recordOutput("Angle", angle)
        Drivetrain.driveRobotOriented(ChassisSpeeds(
            0.0, //accelLimitedChassisSpeeds.vxMetersPerSecond,
            0.0,//accelLimitedChassisSpeeds.vyMetersPerSecond,
            0.0))
    }
}
