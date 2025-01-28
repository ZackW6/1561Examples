package frc.robot.subsystems.vision.realVision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.VisionIO;

public class LimelightVision implements VisionIO{

    private final String name;
    public LimelightVision(String name){
        this.name = name;
    }

    @Override
    public void setOrientation(Rotation2d yaw) {
        LimelightHelpers.SetRobotOrientation(name, yaw.getDegrees(), 0, 0, 0, 0, 0);
    }

    @Override
    public PoseEstimate getPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    @Override
    public String getName() {
        return name;
    }
    
}
