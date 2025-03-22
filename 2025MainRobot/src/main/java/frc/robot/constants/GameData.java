package frc.robot.constants;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimMotorState;
import org.ironmaple.utils.mathutils.MapleCommonMath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PoseEX;

public class GameData {

    private static final double coralPoseForwardOffset = -.42;
    private static final double coralPoseRightOffset = .16;

    private static final double feederPoseForwardOffset = -.35;
    private static final double feederPoseRightOffset = 0;

    public static final double optionalFeederRightOffset = -.4;

    private static final double processorPoseForwardOffset = -.4;
    private static final double processorPoseRightOffset = 0;

    private static final double reefAlgaeForwardOffset = -.45;
    private static final double reefAlgaeRightOffset = 0;

    private static final double netForwardOffset = -.45;
    private static final double netRightOffset = 0;

    private static final Pose2d[] aprilTagsPose2d;
    private static final Pose3d[] aprilTagsPose3d;

    private static final Pose2d[] coralPoses = new Pose2d[12];

    private static final Pose2d[] initAlgaePoses = new Pose2d[6];

    private static final Pose2d[] feederPoses = new Pose2d[2];

    private static final Pose2d[] netPoses = new Pose2d[2];

    private static final Pose2d processorPose;
    static{
        aprilTagsPose2d = new Pose2d[AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags().size()];
        aprilTagsPose3d = new Pose3d[AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags().size()];
        int i = 0;
        for (AprilTag tag : AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags()){
            aprilTagsPose2d[i] = tag.pose.toPose2d();
            aprilTagsPose3d[i] = tag.pose;
            i++;
        }
        coralPoses[0] = getAprilTagPose2d(18).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[1] = getAprilTagPose2d(18).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[2] = getAprilTagPose2d(17).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[3] = getAprilTagPose2d(17).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[4] = getAprilTagPose2d(22).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[5] = getAprilTagPose2d(22).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[6] = getAprilTagPose2d(21).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[7] = getAprilTagPose2d(21).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[8] = getAprilTagPose2d(20).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[9] = getAprilTagPose2d(20).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[10] = getAprilTagPose2d(19).plus(new Transform2d(-coralPoseForwardOffset,-coralPoseRightOffset, Rotation2d.fromDegrees(180)));
        coralPoses[11] = getAprilTagPose2d(19).plus(new Transform2d(-coralPoseForwardOffset,coralPoseRightOffset, Rotation2d.fromDegrees(180)));

        initAlgaePoses[0] = getAprilTagPose2d(18).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[1] = getAprilTagPose2d(17).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[2] = getAprilTagPose2d(22).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[3] = getAprilTagPose2d(21).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[4] = getAprilTagPose2d(20).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));
        initAlgaePoses[5] = getAprilTagPose2d(19).plus(new Transform2d(-reefAlgaeForwardOffset,reefAlgaeRightOffset, Rotation2d.fromDegrees(180)));

        feederPoses[0] = getAprilTagPose2d(13).plus(new Transform2d(-feederPoseForwardOffset,feederPoseRightOffset, Rotation2d.fromDegrees(0)));
        feederPoses[1] = getAprilTagPose2d(12).plus(new Transform2d(-feederPoseForwardOffset,feederPoseRightOffset, Rotation2d.fromDegrees(0)));

        netPoses[0] = getAprilTagPose2d(14).plus(new Transform2d(-netForwardOffset,netRightOffset, Rotation2d.fromDegrees(180)));
        netPoses[1] = getAprilTagPose2d(4).plus(new Transform2d(-netForwardOffset,netRightOffset, Rotation2d.fromDegrees(180)));

        processorPose = getAprilTagPose2d(16).plus(new Transform2d(-processorPoseForwardOffset,processorPoseRightOffset, Rotation2d.fromDegrees(180)));
    }
    public static final double fieldSizeX = Units.feetToMeters(57.573);
    public static final double fieldSizeY = Units.feetToMeters(26.417);

    public static final Translation2d centerOfBlueReef = new Translation2d(Units.feetToMeters(12), fieldSizeY/2);
    public static final Translation2d centerOfRedReef = new Translation2d(fieldSizeX-Units.feetToMeters(12), fieldSizeY/2);

    public static Pose2d getAprilTagPose2d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose2d[fixedNum];
    }

    public static Pose3d getAprilTagPose3d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose3d[fixedNum];
    }

    public static Pose2d coralPose(int place, boolean red){
        int fixedNum = Math.max(Math.min(place,12),1)-1;
        Pose2d pose = coralPoses[fixedNum];
        if (red){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d algaePose(int place, boolean red){
        int fixedNum = Math.max(Math.min(place,6),1)-1;
        Pose2d pose = initAlgaePoses[fixedNum];
        if (red){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d netPose(int place, boolean red){
        int fixedNum = Math.max(Math.min(place,2),1)-1;
        Pose2d pose = netPoses[fixedNum];
        if (red){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d feederPose(int place, boolean red){
        int fixedNum = Math.max(Math.min(place,2),1)-1;
        Pose2d pose = feederPoses[fixedNum];
        if (red){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d processorPose(boolean red){
        Pose2d pose = processorPose;
        if (red){
            pose = PoseEX.pose180(pose);
        }
        return pose;
    }
}
