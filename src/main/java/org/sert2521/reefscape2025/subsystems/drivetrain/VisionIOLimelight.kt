package org.sert2521.reefscape2025.subsystems.drivetrain

import edu.wpi.first.math.geometry.Pose2d
import org.sert2521.reefscape2025.utils.LimelightHelpers


class VisionIOLimelight: VisionIO {

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        val useMegaTag2 = Drivetrain.getGyroConnected() //set to false to use MegaTag1
        var doRejectUpdate = false;
        if(!useMegaTag2)
        {
            val mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if(mt1.tagCount == 1 && mt1.rawFiducials.size == 1)
            {
                if(mt1.rawFiducials[0].ambiguity > .7)
                {
                    doRejectUpdate = true;
                }
                if(mt1.rawFiducials[0].distToCamera > 3)
                {
                    doRejectUpdate = true;
                }
            }
            if(mt1.tagCount == 0)
            {
                doRejectUpdate = true;
            }

            if(!doRejectUpdate)
            {
                inputs.estimatedPosition = mt1.pose
                inputs.timestamp = mt1.timestampSeconds
                inputs.rejectEstimation = false
            } else {
                inputs.estimatedPosition = Pose2d()
                inputs.timestamp = 0.0
                inputs.rejectEstimation = true

            }
        }
        else
        {
            LimelightHelpers.SetRobotOrientation("limelight", Drivetrain.getPose().rotation.degrees, 0.0, 0.0, 0.0, 0.0, 0.0);
            val mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if(Drivetrain.getGyroRateDegrees() > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if(mt2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if(!doRejectUpdate)
            {
                inputs.estimatedPosition = mt2.pose
                inputs.timestamp = mt2.timestampSeconds
                inputs.rejectEstimation = false
            } else {
                inputs.estimatedPosition = Pose2d()
                inputs.timestamp = 0.0
                inputs.rejectEstimation = true
            }
        }
    }
}