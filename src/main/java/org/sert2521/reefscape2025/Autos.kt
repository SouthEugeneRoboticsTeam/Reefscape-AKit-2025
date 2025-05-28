package org.sert2521.reefscape2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.sert2521.reefscape2025.commands.elevator.AlgaeAutoRemoveHigh
import org.sert2521.reefscape2025.commands.elevator.AlgaeAutoRemoveLow
import org.sert2521.reefscape2025.subsystems.wrist.Wrist
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants.DRIVE_GEAR_RATIO
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import org.sert2521.reefscape2025.subsystems.ground_intake.GroundIntake
import org.sert2521.reefscape2025.subsystems.ramp.Ramp
import kotlin.jvm.optionals.getOrElse

object Autos
{
    private var autoChooser:LoggedDashboardChooser<String>

    val namedCommandList = mapOf<String,Command>(
        "Wrist L1" to Wrist.setWristCommandFast(SetpointConstants.WRIST_L1).asProxy(),
        "Wrist Ground" to Wrist.setWristCommandFast(SetpointConstants.WRIST_GROUND).asProxy(),
        "Wrist Stow" to Wrist.setWristCommandFast(SetpointConstants.WRIST_STOW).asProxy(),
        "Wrist Algae" to Wrist.setWristCommandFast(SetpointConstants.WRIST_ALGAE_LOW).asProxy(),

        "Ground Intake" to GroundIntake.intakeCommand().withTimeout(3.0).asProxy(),
        "Ground Outtake" to GroundIntake.outtakeCommand().withTimeout(0.2).asProxy(),

        "Elevator Stow" to Elevator.setElevatorSafeCommand(SetpointConstants.ELEVATOR_STOW).asProxy(),
        "Elevator L2" to Elevator.setElevatorSafeCommand(SetpointConstants.ELEVATOR_L2).asProxy(),
        "Elevator L3" to Elevator.setElevatorSafeCommand(SetpointConstants.ELEVATOR_L3).asProxy(),
        "Elevator L4" to Elevator.setElevatorSafeCommand(SetpointConstants.ELEVATOR_L4).asProxy(),

        "Dispenser Intake" to Commands.none(),
//        "Dispenser Intake" to //Ramp.intakeCommand()
//            .raceWith(Commands.waitUntil{Dispenser.getBlocked()}
//                .andThen(Commands.waitUntil{!Dispenser.getBlocked()}))
//            .withTimeout(3.0).asProxy(),
        "Dispenser Outtake" to Dispenser.outtakeCommandNoWait().asProxy(),

        "Wait L1 Post-Outtake" to Commands.none(),
        "Wait L2-4 Pre-Outtake" to Commands.waitSeconds(0.4),
        "Wait L2-4 Post-Outtake" to Commands.none(),
        "Wait Human Player" to Commands.waitSeconds(1.0),
        "Wait Dispenser" to Commands.waitSeconds(2.0)
            .andThen(Commands.waitUntil{!Dispenser.getBlocked()}),

        "Ramp Intake" to Ramp.intakeCommand()
            .until{Dispenser.getBlocked()},

        "Stop Drivetrain" to Drivetrain.stopCommand().asProxy(),

        "Remove Algae High" to AlgaeAutoRemoveHigh().asProxy(),

        "Remove Algae Low" to AlgaeAutoRemoveLow().asProxy()
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
                    SwerveConstants.driveMotorGearbox.withReduction(DRIVE_GEAR_RATIO),
                    SwerveConstants.DRIVE_CURRENT_LIMIT_AUTO.toDouble(),
                    1
                ),
                *SwerveConstants.moduleTranslations
            ),
            {DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red},
            Drivetrain
        )
        //Pathfinding.setPathfinder(LocalADStarAK())
//        PathPlannerLogging.setLogActivePathCallback { activePath:List<Pose2d> ->
//            Logger.recordOutput(
//                "Odometry/Trajectory", *activePath.toTypedArray()
//            )
//        }

        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d ->
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        }

        autoChooser = LoggedDashboardChooser("Auto Chooser")

        autoChooser.addDefaultOption("None", "None")
        autoChooser.addOption("Leave", "Leave")
        autoChooser.addOption("Left 1 L4", "Left 1 L4")

        autoChooser.addOption("Center L1", "Center - L1")
        autoChooser.addOption("Left L1", "Left - L1")
        autoChooser.addOption("Center L4", "Center 1 L4")
        autoChooser.addOption("Right 3 L4 Test", "Right 3 L4 Test")

        autoChooser.addOption("Left 3", "Left 3 L4")
        autoChooser.addOption("Left 3 Test", "Left 3 L4 Test")
        autoChooser.addOption("Left Rot Test", "Test Rot")
        autoChooser.addOption("Left Rot Test Slow", "Test Rot Slow")
    }


    fun getAutonomousCommand(): Command
    {
        val chosenAuto = autoChooser.get()
        try{
            return if (chosenAuto == "None"){
                Commands.none()
            } else {
                AutoBuilder.buildAuto(chosenAuto) // This is the part where errors may occur
            }
        } catch (e:Throwable) {
            println(e)
            println("AUTO BUILDING ERROR FOR " + autoChooser.get())
            return Commands.none()
        }
    }
}