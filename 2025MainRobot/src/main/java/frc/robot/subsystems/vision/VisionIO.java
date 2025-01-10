package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;

public interface VisionIO {

    /**
     * used to help megatag 2 find pose more accurately, do periodically
     * @param yaw
     */
    public void setOrientation(String name, Rotation2d yaw);

    public LimelightHelpers.PoseEstimate getPoseEstimate(String name);
}
