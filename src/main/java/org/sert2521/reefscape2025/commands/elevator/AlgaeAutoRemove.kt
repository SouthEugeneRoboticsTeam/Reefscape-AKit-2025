package org.sert2521.reefscape2025.commands.elevator


import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.reefscape2025.SetpointConstants.ELEVATOR_STOW
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.elevator.Elevator

// TODO: Add your sequential commands in the super constructor call,
//       e.g. SequentialCommandGroup(OpenClawCommand(), MoveArmCommand())
class AlgaeAutoRemove : ParallelDeadlineGroup(
    Commands.waitSeconds(0.0).andThen(Elevator.setElevatorCommand(ELEVATOR_STOW)),
    Commands.waitSeconds(0.3).andThen(Commands.run({Drivetrain.driveRobotOriented(ChassisSpeeds(-0.7, 0.0, 0.0))}, Drivetrain))
)
