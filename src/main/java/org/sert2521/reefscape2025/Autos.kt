package org.sert2521.reefscape2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import org.sert2521.reefscape2025.commands.drivetrain.DrivetrainFeedforwardSysId
import org.sert2521.reefscape2025.commands.elevator.SetElevator
import org.sert2521.reefscape2025.commands.wrist.SetWrist
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.ground_intake.GroundIntake
import org.sert2521.reefscape2025.utils.LocalADStarAK

object Autos
{
    private var autoChooser = SendableChooser<Command>()

    val namedCommandList = mapOf<String,Command>(
        "Wrist L1" to SetWrist(SetpointConstants.WRIST_L1),
        "Wrist Ground" to SetWrist(SetpointConstants.WRIST_GROUND),
        "Wrist Stow" to SetWrist(SetpointConstants.WRIST_STOW),
        "Wrist Algae" to SetWrist(SetpointConstants.WRIST_ALGAE),

        "Ground Intake" to GroundIntake.intakeCommand().withTimeout(3.0),
        "Ground Outtake" to GroundIntake.outtakeCommand().withTimeout(0.2),

        "Elevator Stow" to SetElevator(SetpointConstants.ELEVATOR_STOW),
        "Elevator L2" to SetElevator(SetpointConstants.ELEVATOR_L2),
        "Elevator L3" to SetElevator(SetpointConstants.ELEVATOR_L3),
        "Elevator L4" to SetElevator(SetpointConstants.ELEVATOR_L4),

        "Dispenser Intake" to Commands.none(),
        "Dispenser Outtake" to Dispenser.outtakeCommand().withTimeout(0.2),

        "Wait L1 Post-Outtake" to Commands.none(),
        "Wait L2-4 Pre-Outtake" to Commands.waitSeconds(0.4),
        "Wait L2-4 Post-Outtake" to Commands.none(),
        "Wait Human Player" to Commands.waitSeconds(1.0),
        "Wait Dispenser" to Commands.waitSeconds(2.0)
            .andThen(Commands.waitUntil{!Dispenser.getBlocked()})
    )

    init{
        NamedCommands.registerCommands(namedCommandList)

        AutoBuilder.configure(
            Drivetrain::getPose,
            Drivetrain::setPose,
            Drivetrain::getChassisSpeeds,
            Drivetrain::driveRobotOriented,
            PPHolonomicDriveController(
                SwerveConstants.autoTranslationPID,
                SwerveConstants.autoRotationPID
            ),
            RobotConfig(
                PhysicalConstants.robotMass,
                PhysicalConstants.momentOfInertia,
                ModuleConfig(
                    SwerveConstants.WHEEL_RADIUS_METERS,
                    SwerveConstants.MAX_SPEED_MPS,
                    SwerveConstants.WHEEL_COF,
                    SwerveConstants.driveMotorGearbox,
                    SwerveConstants.DRIVE_CURRENT_LIMIT_AUTO.toDouble(),
                    1
                ),
                *SwerveConstants.moduleTranslations
            ),
            {false}
        )
        Pathfinding.setPathfinder(LocalADStarAK())
        PathPlannerLogging.setLogActivePathCallback { activePath:List<Pose2d> ->
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toTypedArray()
            )
        }

        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d? ->
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        }

        autoChooser = AutoBuilder.buildAutoChooser()

        autoChooser.addOption("SysId quasistatic", DrivetrainFeedforwardSysId.get())
        autoChooser.addOption("SysId dynamic", Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))

        SmartDashboard.putData("Auto Chooser", autoChooser)
    }


    fun getAutonomousCommand(): Command
    {
        return autoChooser.selected
    }
}