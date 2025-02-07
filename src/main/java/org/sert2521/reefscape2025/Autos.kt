package org.sert2521.reefscape2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.controllers.PathFollowingController
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants


object Autos
{
    private var autoChooser = SendableChooser<Command>()

    val namedCommandList = mapOf<String,Command>(
        "Wrist L1" to Commands.none(),
        "Wrist Ground" to Commands.none(),
        "Wrist Stow" to Commands.none(),
        "Wrist Algae" to Commands.none(),

        "Wrist Intake" to Commands.none(),
        "Wrist Outtake" to Commands.none(),

        "Elevator Stow" to Commands.none(),
        "Elevator L2" to Commands.none(),
        "Elevator L3" to Commands.none(),
        "Elevator L4" to Commands.none(),

        "Dispenser Intake" to Commands.none(),
        "Dispenser Outtake" to Commands.none(),

        "Wait L2-4 Pre-Outtake" to Commands.none(),
        "Wait L2-4 Post-Outtake" to Commands.none(),
        "Wait Human Player" to Commands.none()
    )

    init{
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
                )
            ),
            {false}
        )

        NamedCommands.registerCommands(namedCommandList)

        autoChooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData("Auto Chooser", autoChooser)
    }


    fun getAutonomousCommand(): Command
    {
        return autoChooser.selected
    }
}