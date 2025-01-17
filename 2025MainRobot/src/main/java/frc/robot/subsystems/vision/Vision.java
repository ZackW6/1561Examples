package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.GameData;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.swerve.SwerveDriveIO;
import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.subsystems.vision.realVision.LimelightVision;
import frc.robot.subsystems.vision.simVision.SimVision;

public class Vision {

    public static class VisionMeasurement{

        public final Pose2d pose;
        public final double timeStamp; 
        public final Vector<N3> stdDev;

        public VisionMeasurement(Pose2d pose, double timeStamp, Vector<N3> stdDev){
            this.pose = pose;
            this.timeStamp = timeStamp;
            this.stdDev = stdDev;
        }
    }

    private final SwerveDriveIO drivetrain;
    private final String[] names;
    private final VisionIO[] visionIO;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable odom = robot.getSubTable("Odometry");

    private final StructPublisher<Pose2d>[] visionPubliser;
    private final StructArrayPublisher<Pose3d>[] seenPublisher;

    private final Pose2d[][] poseMemory;

    @SuppressWarnings("unchecked")
    public Vision(SwerveDriveIO drivetrain, Transform3d[] transforms, String... limelightNames){
        
        this.drivetrain = drivetrain;
        names = limelightNames;
        visionIO = new VisionIO[limelightNames.length];

        if (Robot.isSimulation()){
            for (int i = 0; i < limelightNames.length; i++){
                visionIO[i] = new SimVision(drivetrain, limelightNames[i], transforms[i]);
            }
        }else{
            for (int i = 0; i < limelightNames.length; i++){
                visionIO[i] = new LimelightVision(limelightNames[i]);
            }
        }

        visionPubliser = new StructPublisher[limelightNames.length + 1];
        seenPublisher = new StructArrayPublisher[limelightNames.length];
        for (int i = 0; i < limelightNames.length; i++){
            visionPubliser[i] = odom
                .getStructTopic(limelightNames[i]+"VisionPose", Pose2d.struct).publish();
            seenPublisher[i] = odom
                .getStructArrayTopic(limelightNames[i]+"SeenPose", Pose3d.struct).publish();
        }
        visionPubliser[names.length] = odom
            .getStructTopic("averageVisionPose", Pose2d.struct).publish();

        poseMemory = new Pose2d[names.length][2];
        
    }

    private boolean isInBouds(LimelightHelpers.PoseEstimate currentPose){
        boolean isInBounds = currentPose.pose.getX() > 0 && currentPose.pose.getX() < 17.55 && currentPose.pose.getY() > 0 && currentPose.pose.getY() < 8.1;
        return isInBounds;
    }

    private final double maxRotationalAcceleration = 60;
    public void updateVisionPose(){
        ArrayList<VisionMeasurement> visionPoses = new ArrayList<>();
        for (int i = 0; i < visionIO.length; i++){
            VisionIO vision = visionIO[i];
            Optional<VisionMeasurement> pose = updateCameraPose(vision, i);
            if (pose.isPresent()){
                visionPoses.add(pose.get());
            }
        }
        if (visionPoses.size() == 0){
            return;
        }
        double totalX = 0;
        double totalY = 0;
        double totalRot = 0;
        for (VisionMeasurement measurement : visionPoses){
            Pose2d pose = measurement.pose;
            totalX += pose.getX();
            totalY += pose.getY();
            totalRot += pose.getRotation().getRadians();
        }
        Pose2d average = new Pose2d(totalX/visionPoses.size(), totalY/visionPoses.size(), Rotation2d.fromRadians(totalRot/visionPoses.size()));
        visionPubliser[visionPubliser.length - 1].accept(average);
        for (VisionMeasurement measurement : visionPoses){
            if (poseWithinRange(measurement.pose, average, .2)){
                drivetrain.addVisionMeasurement(
                    measurement.pose,
                    measurement.timeStamp,measurement.stdDev);
            }
        }
    }

    private Optional<VisionMeasurement> updateCameraPose(VisionIO visionIO, int index){
        LimelightHelpers.PoseEstimate botPose;
        
        try {
            botPose = visionIO.getPoseEstimate();
            visionIO.setOrientation(drivetrain.getPose().getRotation());
        } catch (Exception e) {
            System.out.println("A camera does not exist");
            return Optional.empty();
        }
        
        if(botPose.tagCount == 0) {
            return Optional.empty();
        }

        poseMemory[index][1] = poseMemory[index][0];
        poseMemory[index][0] = botPose.pose;
        if(!isInBouds(botPose)) {
            seenPublisher[index].accept(new Pose3d[]{});
            visionPubliser[index].accept(botPose.pose);
            return Optional.empty();
        }

        if (poseMemory[index][0] != null && !poseWithinRange(poseMemory[index][0], botPose.pose, .5)){
            seenPublisher[index].accept(new Pose3d[]{});
            visionPubliser[index].accept(botPose.pose);
            return Optional.empty();
        }

        if (poseMemory[index][1] != null && !poseWithinRange(poseMemory[index][1], botPose.pose, 1)){
            seenPublisher[index].accept(new Pose3d[]{});
            visionPubliser[index].accept(botPose.pose);
            return Optional.empty();
        }

        double ambiguity = 0;
        for (RawFiducial fiducial : botPose.rawFiducials){
            ambiguity+=fiducial.ambiguity;
        }

        if (ambiguity/botPose.rawFiducials.length > .2){
            seenPublisher[index].accept(new Pose3d[]{});
            visionPubliser[index].accept(botPose.pose);
            return Optional.empty();
        }
        VisionMeasurement measurement;
        if(Math.abs(Units.radiansToDegrees(drivetrain.getSpeeds().omegaRadiansPerSecond)) > maxRotationalAcceleration){
            
            double[] rotationInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
            // drivetrain.addVisionMeasurement(
            //     botPose.pose,
            //     botPose.timestampSeconds,VecBuilder.fill(rotationInterpolation[0]*2,rotationInterpolation[1]*2,999999));
            measurement = new VisionMeasurement(botPose.pose,
             botPose.timestampSeconds,
             VecBuilder.fill(rotationInterpolation[0]*2,rotationInterpolation[1]*2,999999));
        }else{
            double[] linearInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get((botPose.avgTagDist)/botPose.rawFiducials.length);
            // drivetrain.addVisionMeasurement(
            //     botPose.pose,
            //     botPose.timestampSeconds,VecBuilder.fill(linearInterpolation[0],linearInterpolation[1],.1));//linearInterpolation[0],linearInterpolation[1],999999));
            measurement = new VisionMeasurement(botPose.pose,
             botPose.timestampSeconds,
             VecBuilder.fill(linearInterpolation[0],linearInterpolation[1],linearInterpolation[2]));//linearInterpolation[0],linearInterpolation[1],999999));
        }

        visionPubliser[index].accept(botPose.pose);
        //TODO was very slow, needs testing
        Pose3d[] poses = new Pose3d[botPose.rawFiducials.length];
        int i = 0;
        for (RawFiducial fiducial : botPose.rawFiducials){
            Pose3d pose = new Pose3d();
            Optional<Pose3d> poseM = GameData.getAprilTagPose3d(fiducial.id);
            if (poseM.isPresent()){
                pose = poseM.get();
            }
            poses[i] = pose;
            i++;
        }
        
        seenPublisher[index].accept(poses);
        return Optional.of(measurement);
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
