package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface GyroIO {
    @Logged
    open class GyroIOInputs{
        var connected = false
        var yawPosition = Rotation2d()
        var yawVelocityRadPerSec = 0.0
        var odometryYawTimestamps = doubleArrayOf()
        var odometryYawPositions = arrayOf<Rotation2d>()
    }

    fun updateInputs(inputs:GyroIOInputs){}
}