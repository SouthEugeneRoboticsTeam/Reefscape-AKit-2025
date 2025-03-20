package org.sert2521.reefscape2025

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.sert2521.reefscape2025.SetpointConstants.ELEVATOR_L2
import org.sert2521.reefscape2025.SetpointConstants.ELEVATOR_L3
import org.sert2521.reefscape2025.SetpointConstants.ELEVATOR_L4
import org.sert2521.reefscape2025.SetpointConstants.ELEVATOR_STOW
import org.sert2521.reefscape2025.SetpointConstants.WRIST_STOW
import org.sert2521.reefscape2025.commands.drivetrain.JoystickDrive
import org.sert2521.reefscape2025.commands.drivetrain.SimpleVisionAlign
import org.sert2521.reefscape2025.commands.elevator.AlgaeAutoRemoveHigh
import org.sert2521.reefscape2025.commands.elevator.AlgaeAutoRemoveLow
import org.sert2521.reefscape2025.commands.elevator.RemoveAlgae
import org.sert2521.reefscape2025.subsystems.dispenser.Dispenser
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain
import org.sert2521.reefscape2025.subsystems.drivetrain.SwerveConstants
import org.sert2521.reefscape2025.subsystems.elevator.Elevator
import org.sert2521.reefscape2025.subsystems.ground_intake.GroundIntake
import org.sert2521.reefscape2025.subsystems.ramp.Ramp
import org.sert2521.reefscape2025.subsystems.wrist.Wrist
import kotlin.math.min

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
    private val algaeRemoveLow = driverController.pov(180)
    private val algaeRemoveHigh = driverController.pov(0)
    private val simpleVisionAlignLeft = driverController.x()
    private val simpleVisionAlignRight = driverController.b()
    private val stopJoystickFieldOrientation = Trigger{ driverController.leftTriggerAxis>0.3 }
    private val slowMode = Trigger{ driverController.rightTriggerAxis>0.3 }

    // Wrist:
    private val wristIntakeCoral = JoystickButton(gunnerController, 12)
    private val wristIntakeAlgae = JoystickButton(gunnerController, 11)
    private val wristOuttakeCoral = JoystickButton(gunnerController, 15)
    private val wristOuttakeAlgae = JoystickButton(gunnerController, 16)
    private val wristStow = JoystickButton(gunnerController, 3)

    // Wrist Rollers:
    private val wristCoralOuttakeDriver = driverController.leftBumper()

    // Elevator:
    private val elevatorStow = JoystickButton(gunnerController, 8)
    private val elevatorL2 = JoystickButton(gunnerController, 7)
    private val elevatorL3 = JoystickButton(gunnerController, 6)
    private val elevatorL4 = JoystickButton(gunnerController, 5)
    private val groundIntakeWhileUp = JoystickButton(gunnerController, 10)

    // Dispenser:
    private val dispenserManualIntake = JoystickButton(gunnerController, 11)
    private val dispenserOuttake = driverController.rightBumper()
    private val dispenserReset = JoystickButton(gunnerController, 14)
    private val rampIntake = JoystickButton(gunnerController, 13)

    private val increaseSpeed = JoystickButton(gunnerController, 1)
    private val decreaseSpeed = JoystickButton(gunnerController, 2)

    private val intakeRumble = Trigger{ Dispenser.getRampBeambreakBlocked() }


    init {

        // rumble.onTrue(runOnce({setRumble(0.8)}).andThen(WaitCommand(0.2).andThen(runOnce({ setRumble(0.0) }))))

        // Command Assignment
        intakeRumble.onTrue(rumbleBlip())

        /* Drivetrain */
        resetRotOffset.onTrue(runOnce({ rotationOffset=Drivetrain.getPose().rotation }))
        resetGyroRawYaw.onTrue(runOnce({ Drivetrain.setPose(
            Pose2d(Drivetrain.getPose().x, Drivetrain.getPose().y, Rotation2d())
        )}))

        algaeRemoveLow.onTrue(AlgaeAutoRemoveLow())
        algaeRemoveHigh.onTrue(AlgaeAutoRemoveHigh())

        stopJoystickFieldOrientation.whileTrue(JoystickDrive(false)
            .andThen(Dispenser.outtakeCommand()))
        simpleVisionAlignLeft.whileTrue(SimpleVisionAlign(true).andThen(Dispenser.outtakeCommand()))
        simpleVisionAlignRight.whileTrue(SimpleVisionAlign(false).andThen(Dispenser.outtakeCommand()))

        /* Wrist */
        wristStow.onTrue(Wrist.initWristCommand())
        wristOuttakeCoral.whileTrue(Wrist.setWristCommandSlow(SetpointConstants.WRIST_L1)
            .andThen(GroundIntake.outtakeCoralCommand()))
            .onFalse(Wrist.setWristCommandFast(SetpointConstants.WRIST_STOW))
        wristIntakeAlgae.whileTrue(Wrist.setWristCommandFast(SetpointConstants.WRIST_ALGAE_LOW)
            .andThen(GroundIntake.outtakeCommand()))
        wristIntakeAlgae.onFalse(Wrist.setWristCommandSlow(SetpointConstants.WRIST_ALGAE_HIGH)
            .raceWith(GroundIntake.outtakeCommand()).andThen(GroundIntake.holdAlgaeCommand()))
        wristIntakeCoral.whileTrue(Wrist.setWristCommandFast(SetpointConstants.WRIST_GROUND)
            .andThen(GroundIntake.intakeCommand()))
            .onFalse(Wrist.setWristCommandSlow(SetpointConstants.WRIST_STOW)
                .raceWith(GroundIntake.intakeCommand()))
        wristOuttakeAlgae.whileTrue(Wrist.setWristCommandFast(SetpointConstants.WRIST_ALGAE_LOW)
            .alongWith(GroundIntake.intakeCommand()))
            .onFalse(Wrist.setWristCommandFast(WRIST_STOW))

        groundIntakeWhileUp.whileTrue(GroundIntake.intakeCommand())


        /* Ground Intake */
        wristCoralOuttakeDriver.whileTrue(GroundIntake.outtakeCoralCommand())

        /* Elevator */
        elevatorStow.onTrue(Elevator.setElevatorSafeCommand(ELEVATOR_STOW))

        elevatorL2.onTrue(Elevator.setElevatorSafeCommand(ELEVATOR_L2))

        elevatorL3.onTrue(Elevator.setElevatorSafeCommand(ELEVATOR_L3))

        elevatorL4.onTrue(Elevator.setElevatorSafeCommand(ELEVATOR_L4))

        // toggleElevatorSafe.onTrue(runOnce({Elevator.toggleSafeMode()}))

        /* Dispenser */
        // dispenserManualIntake.onTrue(DispenserManualIntake())
        dispenserOuttake.whileTrue(Dispenser.outtakeCommand())
        dispenserReset.onTrue(Dispenser.recenterCommand().alongWith(Ramp.recenterCommand()))
        rampIntake.whileTrue(Ramp.intakeCommand())
        // toggleAutomaticIntake.onTrue(runOnce({Dispenser.changeIntakeMode()}))


        /* End Triggers */

    }

    private var rotationOffset = Rotation2d(0.0)

    fun getJoystickX():Double{ return -driverController.leftX }

    fun getJoystickY():Double{ return -driverController.leftY }

    fun getJoystickZ():Double { return -driverController.rightX }

    fun getRotOffset(): Rotation2d { return rotationOffset }

    fun getGyroReset():Boolean{
        return resetGyroVision.asBoolean
    }

    fun getAccelLimit():Double{
        val elevatorLimit = MathUtil.interpolate(
            SwerveConstants.DRIVE_ACCEL_FAST, SwerveConstants.DRIVE_ACCEL_SLOW,
            Elevator.getPosition()/SetpointConstants.ELEVATOR_L4)
        return MathUtil.interpolate(elevatorLimit, elevatorLimit, driverController.rightTriggerAxis)
    }

    fun getDeccelLimit():Double{
        val elevatorLimit = MathUtil.interpolate(
            SwerveConstants.DRIVE_DECCEL_FAST, SwerveConstants.DRIVE_DECCEL_SLOW,
            Elevator.getPosition()/SetpointConstants.ELEVATOR_L4)
        return MathUtil.interpolate(elevatorLimit, elevatorLimit, driverController.rightTriggerAxis)
    }

    fun getSpeedLimit():Double{
        val elevatorLimit = MathUtil.interpolate(
            SwerveConstants.DRIVE_SPEED_FAST, SwerveConstants.DRIVE_SPEED_SLOW,
            Elevator.getPosition()/SetpointConstants.ELEVATOR_L4)
        return MathUtil.interpolate(elevatorLimit, elevatorLimit/5.0, driverController.rightTriggerAxis)
    }

    fun setRumble(amount: Double) { driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount) }

    fun rumbleBlip(): Command {
        return runOnce({ setRumble(0.8) }).andThen(Commands.waitSeconds(0.2))
            .andThen(runOnce({ setRumble(0.0) }))
    }
}