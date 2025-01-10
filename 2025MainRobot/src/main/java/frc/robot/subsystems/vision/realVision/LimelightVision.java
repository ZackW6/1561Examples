package frc.robot.subsystems.vision.realVision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.VisionIO;

public class LimelightVision implements VisionIO{

    @Override
    public void setOrientation(String name, Rotation2d yaw) {
        LimelightHelpers.SetRobotOrientation(name, yaw.getDegrees(), 0, 0, 0, 0, 0);
    }

    @Override
    public PoseEstimate getPoseEstimate(String name) {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }
    
}
