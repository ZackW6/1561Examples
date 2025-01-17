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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class GameData {
    private static final Pose2d[] aprilTagsPose2d;
    private static final Pose3d[] aprilTagsPose3d;
    static{
        aprilTagsPose2d = new Pose2d[AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags().size()];
        aprilTagsPose3d = new Pose3d[AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags().size()];
        int i = 0;
        for (AprilTag tag : AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags()){
            aprilTagsPose2d[i] = tag.pose.toPose2d();
            aprilTagsPose3d[i] = tag.pose;
            i++;
        }
    }
    public static final double fieldSizeX = Units.feetToMeters(57.573);
    public static final double fieldSizeY = Units.feetToMeters(26.417);

    public static final Translation2d centerOfBlueReef = new Translation2d(Units.feetToMeters(12), fieldSizeY/2);
    public static final Translation2d centerOfRedReef = new Translation2d(fieldSizeX-Units.feetToMeters(12), fieldSizeY/2);

    public static Optional<Pose2d> getAprilTagPose2d(int id){
        if (id > aprilTagsPose2d.length){
            return Optional.empty();
        }
        return Optional.of(aprilTagsPose2d[id-1]);
    }

    public static Optional<Pose3d> getAprilTagPose3d(int id){
        if (id > aprilTagsPose3d.length){
            return Optional.empty();
        }
        return Optional.of(aprilTagsPose3d[id-1]);
    }
}
