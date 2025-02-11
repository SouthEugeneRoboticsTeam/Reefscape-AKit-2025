package org.sert2521.reefscape2025

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.sert2521.reefscape2025.MetaConstants.atEvent
import org.sert2521.reefscape2025.subsystems.drivetrain.Drivetrain


object Input {
    private val driverController = XboxController(0)
    private val operatorController = Joystick(1)

    

    private var rotationOffset = Rotation2d()

    private val rotationReset = JoystickButton(driverController, 4)

    init {
        rotationReset.onTrue(runOnce({ rotationOffset=Drivetrain.getPose().rotation }))
    }


    fun getJoystickX():Double{
        return -driverController.leftX
    }

    fun getJoystickY():Double{
        return -driverController.leftY
    }

    fun getJoystickZ():Double {
        return -driverController.rightX
    }

    fun getRotationOffset():Rotation2d{
        return rotationOffset
    }
}