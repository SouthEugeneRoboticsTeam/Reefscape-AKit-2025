package org.sert2521.reefscape2025.subsystems.drive

import edu.wpi.first.math.estimator.PoseEstimator
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import java.util.concurrent.locks.ReentrantLock

object Drivetrain : SubsystemBase() {
    @JvmField
    val odometryLock:ReentrantLock = ReentrantLock()

    val gyroIoInputs = LoggedGyroIOInputs()
    val gyroIO = GyroIONavX()

    val modules = arrayOf(Module(0), Module(1), Module(2), Module(3))

    val sysId =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                null,
                null,
                { Logger.recordOutput("Drive/SysIdState", it.toString())},
            ),
            SysIdRoutine.Mechanism(
                { runCharacterization(it) }, null, this
            )
        )

    val gyroDisconnectedAlert =
        Alert("Disconnected Gyro, resorting to kinematics", Alert.AlertType.kError)

    val kinematics = SwerveDriveKinematics(
        SwerveConstants.moduleTranslations[0],
        SwerveConstants.moduleTranslations[1],
        SwerveConstants.moduleTranslations[2],
        SwerveConstants.moduleTranslations[3]
    )

    val lastModulePositions = arrayOf(
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition()
    )

    val rawGyroRotation = Rotation2d()

    val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        rawGyroRotation,
        lastModulePositions,
        Pose2d()
    )




}