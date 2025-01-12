package frc.robot.subsystems.vision;

import java.util.List;

import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.swerve.SwerveDriveIO;
import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.subsystems.vision.realVision.LimelightVision;
import frc.robot.subsystems.vision.simVision.SimVision;

public class Vision {

    private final SwerveDriveIO drivetrain;
    private final String[] names;
    private final VisionIO visionIO;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable odom = robot.getSubTable("Odometry");

    private final StructPublisher<Pose2d> visionPubliser = odom
        .getStructTopic("VisionPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> seenPublisher = odom
        .getStructTopic("SeenPose", Pose2d.struct).publish();

    public Vision(SwerveDriveIO drivetrain, String... limelightNames){
        this.drivetrain = drivetrain;
        names = limelightNames;

        if (Robot.isSimulation()){
            if (drivetrain instanceof SimSwerve){
                visionIO = new SimVision(drivetrain);
            }else{
                visionIO = new SimVision(drivetrain);
            }
        }else{
            visionIO = new LimelightVision();
        }
    }

    private boolean isInBouds(LimelightHelpers.PoseEstimate currentPose){
        boolean isInBounds = currentPose.pose.getX() > 0 && currentPose.pose.getX() < 17.55 && currentPose.pose.getY() > 0 && currentPose.pose.getY() < 8.1;
        return isInBounds;
    }

    private Pose2d lastPose = null;

    public void updateVisionPose(){
        double maxRotationalAcceleration = 60;

        LimelightHelpers.PoseEstimate botPose = visionIO.getPoseEstimate(names[0]);//LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(names[0]);

        if(botPose.tagCount == 0) {
            return;
        }

        visionIO.setOrientation(names[0], drivetrain.getPose().getRotation());//LimelightHelpers.SetRobotOrientation(names[0], drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if(!isInBouds(botPose)) {
            return;
        }

        if (lastPose != null && !poseWithinRange(lastPose, botPose.pose, 1)){
            visionPubliser.accept(botPose.pose);
            lastPose = botPose.pose;
            return;
        }

        double ambiguity = 0;
        for (RawFiducial fiducial : botPose.rawFiducials){
            ambiguity+=fiducial.ambiguity;
        }

        if (ambiguity/botPose.rawFiducials.length > .2){
            visionPubliser.accept(botPose.pose);
            lastPose = botPose.pose;
            System.out.println(ambiguity/botPose.rawFiducials.length);
            return;
        }

        if(Math.abs(Units.radiansToDegrees(drivetrain.getSpeeds().omegaRadiansPerSecond)) > maxRotationalAcceleration){
            
            double[] rotationInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
            drivetrain.addVisionMeasurement(
                botPose.pose,
                botPose.timestampSeconds,VecBuilder.fill(rotationInterpolation[0],rotationInterpolation[1],999999));
        }else{
            double[] linearInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
            drivetrain.addVisionMeasurement(
                botPose.pose,
                botPose.timestampSeconds,VecBuilder.fill(linearInterpolation[0],linearInterpolation[1],999999));//linearInterpolation[0],linearInterpolation[1],999999));
        }
        
        visionPubliser.accept(botPose.pose);
        //TODO very slow for some reason, fix that
        Pose2d pose = new Pose2d();
        if (LimelightConstants.K_TAG_LAYOUT.getTagPose(botPose.rawFiducials[0].id).isPresent()){
            pose = LimelightConstants.K_TAG_LAYOUT.getTagPose(botPose.rawFiducials[0].id).get().toPose2d();
        }
        seenPublisher.accept(pose);

        lastPose = botPose.pose;
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
