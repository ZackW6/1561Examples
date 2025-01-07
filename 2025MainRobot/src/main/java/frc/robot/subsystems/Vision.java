package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;

public class Vision {

    private final CommandSwerveDrivetrain drivetrain;
    private final String[] names;

    public Vision(CommandSwerveDrivetrain drivetrain, String... limelightNames){
        this.drivetrain = drivetrain;
        names = limelightNames;
    }

    private boolean isInBouds(LimelightHelpers.PoseEstimate currentPose){
        boolean isInBounds = currentPose.pose.getX() > 0 && currentPose.pose.getX() < 17.55 && currentPose.pose.getY() > 0 && currentPose.pose.getY() < 8.05;
        return isInBounds;
    }

    public void updateVisionPose(){
        double maxRotationalAcceleration = 60;

        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(names[0]);
        if(botPose.tagCount == 0) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(names[0], drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if(!isInBouds(botPose)) {
            return;
        }
        if(Math.abs(Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond)) > maxRotationalAcceleration){

            double[] rotationInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
            drivetrain.addVisionMeasurement(
                botPose.pose,
                botPose.timestampSeconds,VecBuilder.fill(rotationInterpolation[0],rotationInterpolation[1],999999));
            return;
        }
        
        double[] linearInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
        drivetrain.addVisionMeasurement(
            botPose.pose,
            botPose.timestampSeconds,VecBuilder.fill(linearInterpolation[0],linearInterpolation[1],999999));
    }

    /**
     * all in meters
     * @param pose
     * @param expected
     * @param error
     * @return
     */
    public static boolean poseWithinRange(Pose2d pose, Pose2d expected, double error){
        if (pose.getX()<expected.getX()+error && pose.getX()>expected.getX()-error && pose.getY()<expected.getY()+error && pose.getY()>expected.getY()-error){
            return true;
        }
        return false;
    }
}
