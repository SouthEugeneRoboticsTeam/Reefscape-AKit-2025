package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.geometry.Pose2d
import org.team9432.annotation.Logged

interface VisionIO {

    @Logged
    open class VisionIOInputs {
        var estimatedPosition = Pose2d()
        var timestamp = 0.0
        var rejectEstimation = true
        var megatagTwo = false
    }

    fun updateInputs(inputs: VisionIOInputs) {}
}