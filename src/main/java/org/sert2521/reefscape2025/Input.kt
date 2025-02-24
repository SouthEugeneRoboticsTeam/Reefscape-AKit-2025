package org.sert2521.reefscape2025

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.sert2521.reefscape2025.SetpointConstants.WRIST_STOW
import org.sert2521.reefscape2025.commands.drivetrain.JoystickDrive
import org.sert2521.reefscape2025.commands.drivetrain.VisionAlign
import org.sert2521.reefscape2025.commands.elevator.RemoveAlgae
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import org.sert2521.reefscape2025.subsystems.ground_intake.GroundIntake
import org.sert2521.reefscape2025.subsystems.wrist.Wrist
import kotlin.math.PI

// Bindings:
// Gunner:
// Stick:
// Wrist Intake -> Trigger [1]
// Wrist Outtake -> Thumb Bottom [2]
// Wrist Stow -> Thumb Left [3]
// Wrist L1 -> Thumb Right [4]
// Left Hand:
// Elevator L2 -> Top Right [7]
// Elevator L3 -> Top Middle [6]
// Elevator L4 -> Top Left [5]
// Elevator Stow -> Bottom Left [10]
// Dispenser (Manipulator) Reset -> Bottom Right [8]
// Right Hand:
// Wrist Algae -> Top Left [13]
// Wrist Ground -> Top Middle [12]
// Dispenser (Manipulator) Intake -> Top Right [11]
// Toggle Automatic Intaking -> Bottom Left [14]
// Toggle Elevator Safe Mode -> Bottom Middle [15]
// Driver:
// Drive -> Left Stick
// Turn -> Right Stick
// Reset Drivetrain -> Y
// Wrist Outtake -> Left Bumper (LB)
// Dispenser (Manipulator) Outtake -> Right Bumper (RB)
// Vision Align -> X

object Input {
    private val driverController = CommandXboxController(0)
    private val gunnerController = Joystick(1)


    // Button Assignment:
    // Drivetrain:
    private val resetRotOffset = driverController.y()
    private val resetGyroRawYaw = driverController.start()
    private val resetGyroVision = driverController.back()
    private val visionAlign = driverController.a()
    private val stopJoystickFieldOrientation = Trigger{driverController.leftTriggerAxis>0.3}

    // Wrist:
    private val wristGround = JoystickButton(gunnerController, 12)
    private val wristAlgae = JoystickButton(gunnerController, 13)
    private val wristL1 = JoystickButton(gunnerController, 15)
    private val wristStow = JoystickButton(gunnerController, 3)

    // Wrist Rollers:
    private val wristRollerIntake = JoystickButton(gunnerController, 14)
    private val wristRollerOuttakeDriver = driverController.leftBumper()
    private val wristRollerOuttakeGunner = JoystickButton(gunnerController, 2)

    // Elevator:
    private val elevatorStow = JoystickButton(gunnerController, 10)
    private val elevatorL2 = JoystickButton(gunnerController, 7)
    private val elevatorL3 = JoystickButton(gunnerController, 6)
    private val elevatorL4 = JoystickButton(gunnerController, 5)
    private val elevatorAlgae = JoystickButton(gunnerController, 9)
    private val toggleElevatorSafe = JoystickButton(gunnerController, 15)

    // Dispenser:
    private val toggleAutomaticIntake = JoystickButton(gunnerController, 14)
    private val dispenserManualIntake = JoystickButton(gunnerController, 11)
    private val dispenserOuttake = driverController.rightBumper()
    private val dispenserReset = JoystickButton(gunnerController, 8)


    init {

        // rumble.onTrue(runOnce({setRumble(0.8)}).andThen(WaitCommand(0.2).andThen(runOnce({ setRumble(0.0) }))))

        // Command Assignment
        // Drivetrain
        resetRotOffset.onTrue(runOnce({ rotationOffset=Drivetrain.getPose().rotation }))
        resetGyroRawYaw.onTrue(runOnce({ Drivetrain.setPose(
            Pose2d(Drivetrain.getPose().x, Drivetrain.getPose().y, Rotation2d())
        )}))
        visionAlign.whileTrue(VisionAlign())
        stopJoystickFieldOrientation.whileTrue(JoystickDrive(false))

        // Wrist
            wristStow.onTrue(Wrist.setWristCommand(SetpointConstants.WRIST_STOW))
            wristL1.whileTrue(Wrist.setWristCommand(SetpointConstants.WRIST_L1)
                .andThen(GroundIntake.outtakeCoralCommand()))
                .onFalse(Wrist.setWristCommand(SetpointConstants.WRIST_STOW))
            wristAlgae.whileTrue(Wrist.setWristCommand(SetpointConstants.WRIST_ALGAE_LOW)
                .andThen(GroundIntake.outtakeCommand()))
            wristAlgae.onFalse(Wrist.setWristCommand(SetpointConstants.WRIST_ALGAE_HIGH)
                .raceWith(GroundIntake.outtakeCommand()))
            wristGround.whileTrue(Wrist.setWristCommand(SetpointConstants.WRIST_GROUND)
                .andThen(GroundIntake.intakeCommand()))
                .onFalse(Wrist.setWristCommand(SetpointConstants.WRIST_STOW))
            wristRollerIntake.whileTrue(Wrist.setWristCommand(SetpointConstants.WRIST_ALGAE_LOW)
                .andThen(GroundIntake.intakeCommand()))
                .onFalse(Wrist.setWristCommand(WRIST_STOW))

        // Wrist Rollers
            wristRollerIntake.whileTrue(GroundIntake.intakeCommand())
            wristRollerOuttakeDriver.whileTrue(GroundIntake.outtakeCoralCommand())
            wristRollerOuttakeGunner.whileTrue(GroundIntake.outtakeCoralCommand())

        // Elevator
            elevatorStow.onTrue(Commands.waitUntil{!Dispenser.getBlocked()}
                .andThen(
                    Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_STOW)
                        .until{ Dispenser.getBlocked() }))

            elevatorL2.onTrue(Commands.waitUntil{!Dispenser.getBlocked()}
                .andThen(Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_L2)
                    .until { Dispenser.getBlocked() }))

            elevatorL3.onTrue(Commands.waitUntil{!Dispenser.getBlocked()}
                .andThen(Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_L3)
                    .until { Dispenser.getBlocked() }))

            elevatorL4.onTrue(Commands.waitUntil{!Dispenser.getBlocked()}
                .andThen(Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_L4)
                    .until { Dispenser.getBlocked() }))

            elevatorAlgae.onTrue(Commands.waitUntil{!Dispenser.getBlocked()}
                .andThen(Elevator.setElevatorCommand(SetpointConstants.ELEVATOR_ALGAE))
                    .until { Dispenser.getBlocked() }.andThen(RemoveAlgae())
                    .until { Dispenser.getBlocked() })
            // toggleElevatorSafe.onTrue(runOnce({Elevator.toggleSafeMode()}))

        // Dispenser
            // dispenserManualIntake.onTrue(DispenserManualIntake())
            dispenserOuttake.whileTrue(Dispenser.outtakeCommand().withTimeout(0.1)
                .andThen(Dispenser.stopCommand().withTimeout(0.4)))
            dispenserReset.onTrue(Dispenser.recenterCommand())
            // toggleAutomaticIntake.onTrue(runOnce({Dispenser.changeIntakeMode()}))

    }

    private var rotationOffset = Rotation2d(0.0)

    fun getJoystickX():Double{ return -driverController.leftX }

    fun getJoystickY():Double{ return -driverController.leftY }

    fun getJoystickZ():Double { return -driverController.rightX }

    fun getRotOffset(): Rotation2d { return rotationOffset }

    fun getGyroReset():Boolean{
        return resetGyroVision.asBoolean
    }

    fun setRumble(amount: Double) { driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount) }
}