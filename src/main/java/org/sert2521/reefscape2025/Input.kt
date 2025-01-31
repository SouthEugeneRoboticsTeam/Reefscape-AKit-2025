package org.sert2521.reefscape2025

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase;

object Input {
    val driverController = XboxController(0)
    val gunnerController = Joystick(1)

    init {

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
}