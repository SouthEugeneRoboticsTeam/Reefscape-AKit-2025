package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.reefscape2025.utils.LimelightHelpers

object Vision : SubsystemBase() {
    override fun periodic() {
        var rejectUpdate = false
        LimelightHelpers.SetRobotOrientation(
            "limelight",
            Drivetrain.getRotation().degrees,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        )
        val mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight")

        if (Drivetrain.getGyroRateDegrees()>720){
            rejectUpdate = true
        }
        if (mt2Estimate.tagCount == 0){
            rejectUpdate = true
        }

        if (!rejectUpdate){
            Drivetrain.addVisionMeasurement(
                mt2Estimate.pose,
                mt2Estimate.timestampSeconds,
                VecBuilder.fill(0.7,0.7,9999999.9)
            )
        }
    }
}