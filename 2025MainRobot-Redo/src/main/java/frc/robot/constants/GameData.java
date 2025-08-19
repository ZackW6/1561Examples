package frc.robot.constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.PoseEX;

public class GameData {

    public static final BooleanSupplier isRed = ()-> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    private static final double coralPoseForwardOffset = -.5;
    private static final double coralPoseRightOffset = .16;

    private static final double feederPoseForwardOffset = -.25;
    private static final double feederPoseRightOffset = 0;

    public static final double optionalFeederRightOffset = -.38;

    private static final double processorPoseForwardOffset = -.4;
    private static final double processorPoseRightOffset = 0;

    private static final double reefAlgaeForwardOffset = -.7;
    private static final double reefAlgaeRightOffset = 0;

    private static final double netForwardOffset = -.45;
    private static final double netRightOffset = 0;

    private static final Pose2d[] aprilTagsPose2d;
    private static final Pose3d[] aprilTagsPose3d;
    
    private static final double L1Height = .409;
    private static final double L2Height = .793;
    private static final double L3Height = 1.18;
    private static final double L4Height = 1.829;

    private static final double[] branchHeights = new double[]{L1Height,L2Height,L3Height,L4Height};
    private static final Pose2d[] branchPoses = new Pose2d[12];

    private static final Pose2d[] coralPoses = new Pose2d[12];

    private static final Pose2d[] initAlgaePoses = new Pose2d[6];

    private static final Pose2d[] feederPoses = new Pose2d[2];

    private static final Pose2d netPose;

    private static final Pose2d reefCenter;

    //found with cad measurements, this is of the circle that contains all the reef branch points
    private static final double reefRadius = .7975966;//1.2957;

    private static final Pose2d processorPose;
    static{
        aprilTagsPose2d = new Pose2d[AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags().size()];
        aprilTagsPose3d = new Pose3d[AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags().size()];
        int i = 0;
        for (AprilTag tag : AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTags()){
            aprilTagsPose2d[i] = tag.pose.toPose2d();
            aprilTagsPose3d[i] = tag.pose;
            i++;
        }


        coralPoses[0] = getAprilTagPose2d(18).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[1] = getAprilTagPose2d(18).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[2] = getAprilTagPose2d(17).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[3] = getAprilTagPose2d(17).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[4] = getAprilTagPose2d(22).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[5] = getAprilTagPose2d(22).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[6] = getAprilTagPose2d(21).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[7] = getAprilTagPose2d(21).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[8] = getAprilTagPose2d(20).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[9] = getAprilTagPose2d(20).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[10] = getAprilTagPose2d(19).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));
        coralPoses[11] = getAprilTagPose2d(19).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset-.03, Rotation2d.fromDegrees(180)));

        initAlgaePoses[0] = getAprilTagPose2d(18).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[1] = getAprilTagPose2d(17).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[2] = getAprilTagPose2d(22).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[3] = getAprilTagPose2d(21).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[4] = getAprilTagPose2d(20).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[5] = getAprilTagPose2d(19).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));

        feederPoses[0] = getAprilTagPose2d(13).plus(new Transform2d(-feederPoseForwardOffset,feederPoseRightOffset, Rotation2d.fromDegrees(0)));
        feederPoses[1] = getAprilTagPose2d(12).plus(new Transform2d(-feederPoseForwardOffset,feederPoseRightOffset, Rotation2d.fromDegrees(0)));

        netPose = getAprilTagPose2d(14).plus(new Transform2d(-netForwardOffset,netRightOffset, Rotation2d.fromDegrees(180)));

        reefCenter = PoseEX.getInbetweenPose2d(getAprilTagPose2d(21), getAprilTagPose2d(18));

        processorPose = getAprilTagPose2d(16).plus(new Transform2d(-processorPoseForwardOffset,processorPoseRightOffset, Rotation2d.fromDegrees(180)));
        
        
        double initDegrees = -11.884033;
        for (int y = 1; y < 13; y++){
            branchPoses[y-1] = PoseEX.linearPlus(rotate(new Pose2d(reefRadius,0,new Rotation2d()), Units.degreesToRadians(initDegrees + 180)), reefCenter);
            if (y % 2 == 0){
                initDegrees+=36.231933;
                continue;
            }
            initDegrees+=23.7680668;
        }
    }
    public static final double fieldSizeX = Units.feetToMeters(57.573);
    public static final double fieldSizeY = Units.feetToMeters(26.417);

    public static Pose2d getAprilTagPose2d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose2d[fixedNum];
    }

    public static Pose3d getAprilTagPose3d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose3d[fixedNum];
    }

    public static Pose3d branchPose(int place, int level){
        int fixedNum = Math.max(Math.min(place,12),1)-1;
        int fixedLevel = MathUtil.clamp(level-1, 0, 3);

        Pose2d rotPose = branchPoses[fixedNum];
        if (isRed.getAsBoolean()){
            rotPose = PoseEX.pose180(rotPose);
        }
        return new Pose3d(rotPose.getX(), rotPose.getY(), branchHeights[fixedLevel], new Rotation3d());
    }

    public static Pose2d coralPose(int place){
        int fixedNum = Math.max(Math.min(place,12),1)-1;
        Pose2d pose = coralPoses[fixedNum];
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d algaeScorePose(int place){
        if (place == 2){
            return netPose();
        }
        return processorPose();
    }

    public static Pose2d algaePose(int place){
        int fixedNum = Math.max(Math.min(place,6),1)-1;
        Pose2d pose = initAlgaePoses[fixedNum];
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d netPose(){
        Pose2d pose = netPose;
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d feederPose(int place){
        int fixedNum = Math.max(Math.min(place,2),1)-1;
        Pose2d pose = feederPoses[fixedNum];
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d feederPose(int place, double rightOffset){
        int fixedNum = Math.max(Math.min(place,2),1)-1;
        Pose2d pose = feederPoses[fixedNum].plus(new Transform2d(0, rightOffset, new Rotation2d()));
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d processorPose(){
        Pose2d pose = processorPose;
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d reefCenterPose(){
        Pose2d pose = reefCenter;
        if (isRed.getAsBoolean()){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    /**
     * does not return a rotation, rotates around the 0,0 of poses
     * @param pose
     * @param angleRadians
     * @return
     */
    private static Pose2d rotate(Pose2d pose, double angleRadians) {
        double cosTheta = Math.cos(angleRadians);
        double sinTheta = Math.sin(angleRadians);
        double newX = pose.getX() * cosTheta - pose.getY() * sinTheta;
        double newY = pose.getX() * sinTheta + pose.getY() * cosTheta;
        return new Pose2d(newX, newY, pose.getRotation());
    }
}
