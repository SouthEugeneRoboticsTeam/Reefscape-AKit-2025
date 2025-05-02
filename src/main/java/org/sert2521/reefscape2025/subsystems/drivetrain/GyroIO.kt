package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged


interface GyroIO {
    /* If you're here and have a problem with the gyro, just call software lead */
    @Logged
    open class GyroIOInputs{
        var connected = false
        var yawPosition = Rotation2d()
        var yawVelocityRadPerSec = 0.0
    }

    open fun updateInputs(inputs:GyroIOInputs){}
}