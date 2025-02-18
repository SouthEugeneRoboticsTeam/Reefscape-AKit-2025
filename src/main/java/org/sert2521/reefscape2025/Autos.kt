package org.sert2521.reefscape2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.sert2521.reefscape2025.commands.drivetrain.DrivetrainFeedforwardSysId
import org.sert2521.reefscape2025.subsystems.wrist.Wrist
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import org.sert2521.reefscape2025.subsystems.ground_intake.GroundIntake

object Autos
{
    private var autoChooser:LoggedDashboardChooser<Command>

    val namedCommandList = mapOf<String,Command>(
        "Wrist L1" to Wrist.setWristCommand(SetpointConstants.WRIST_L1).asProxy(),
        "Wrist Ground" to Wrist.setWristCommand(SetpointConstants.WRIST_GROUND).asProxy(),
        "Wrist Stow" to Wrist.setWristCommand(SetpointConstants.WRIST_STOW).asProxy(),
        "Wrist Algae" to Wrist.setWristCommand(SetpointConstants.WRIST_ALGAE).asProxy(),

        "Ground Intake" to GroundIntake.intakeCommand().withTimeout(3.0).asProxy(),
        "Ground Outtake" to GroundIntake.outtakeCommand().withTimeout(0.2).asProxy(),

        "Elevator Stow" to Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_STOW).asProxy(),
        "Elevator L2" to Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_L2).asProxy(),
        "Elevator L3" to Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_L3).asProxy(),
        "Elevator L4" to Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_L4).asProxy(),

        "Dispenser Intake" to Commands.none(),
        "Dispenser Outtake" to Dispenser.outtakeCommand().withTimeout(0.2).asProxy(),

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
            {
                value:ChassisSpeeds ->
                Drivetrain.driveRobotOriented(value)
                println(value.vxMetersPerSecond)
                println(value.vyMetersPerSecond)
            },
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
            {false},
            Drivetrain
        )
        //Pathfinding.setPathfinder(LocalADStarAK())
//        PathPlannerLogging.setLogActivePathCallback { activePath:List<Pose2d> ->
//            Logger.recordOutput(
//                "Odometry/Trajectory", *activePath.toTypedArray()
//            )
//        }

//        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d ->
//            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
//        }

        autoChooser = LoggedDashboardChooser("Auto Chooser")//"Auto Chooser", AutoBuilder.buildAutoChooser())

        autoChooser.addDefaultOption("None", Commands.none())
        autoChooser.addOption("Leave", AutoBuilder.buildAuto("Leave"))
        autoChooser.addOption("SysId quasistatic", DrivetrainFeedforwardSysId.get())
        autoChooser.addOption("SysId dynamic", Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
    }


    fun getAutonomousCommand(): Command
    {
        return autoChooser.get()
    }
}