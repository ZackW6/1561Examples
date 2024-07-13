package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.google.gson.Gson;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.util.PoseEX;

import com.choreo.lib.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic(){
        if (/*DriverStation.isTeleopEnabled() && */!Robot.isSimulation()){
            // updateVisionPose(LimelightConstants.LIMELIGHT_NAME);
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0.4;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // getState of the robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(5, 0, 0),
                                            new PIDConstants(5, 0, 0),
                    5.3,
            driveBaseRadius,
            new ReplanningConfig()),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                            
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            // Reference to this subsystem to set requirements // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble());
    }
    public Rotation2d getAngleFromPose(Pose2d pose) {
        return PoseEX.getYawFromPose(getPose(), pose);
    }
    public Rotation2d getAngleFromCorner() {
        Pose2d pose = new Pose2d(0,8.1026,new Rotation2d());
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
            pose = new Pose2d(16.4846,8.1026,new Rotation2d());
        }
        return getAngleFromPose(pose);
    }

    public Rotation2d getPoseAngle(Pose2d pose) {
        return PoseEX.getPoseAngle(getPose(), pose);
    }

    public double getDistanceFromPoseMeters(Pose2d pose) {
        return PoseEX.getDistanceFromPoseMeters(getPose(), pose);
    }

    public void seedFieldRelative(double degrees) {
        try {
            m_stateLock.writeLock().lock();
            m_fieldRelativeOffset = new Rotation2d(Units.degreesToRadians(getState().Pose.getRotation().getDegrees()+degrees));
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }


    private boolean isInBouds(LimelightHelpers.PoseEstimate currentPose){

        boolean isInBounds = currentPose.pose.getX() > 0 && currentPose.pose.getX() < 16.4846 && currentPose.pose.getY() > 0 && currentPose.pose.getY() < 8.1026;

        return isInBounds;
    }
    // private void updateVisionPose(String limelightName){
    //     double maxRotationalAcceleration = 60;

    //     LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    //     if(botPose.tagCount == 0) {
    //         return;
    //     }
    //     LimelightHelpers.SetRobotOrientation(limelightName, m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //     if(!isInBouds(botPose)) {
    //         return;
    //     }
    //     if(Math.abs(Units.radiansToDegrees(getState().speeds.omegaRadiansPerSecond)) > maxRotationalAcceleration){

    //         double[] rotationInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
    //         addVisionMeasurement(
    //             botPose.pose,
    //             botPose.timestampSeconds,VecBuilder.fill(rotationInterpolation[0],rotationInterpolation[1],999999));
    //         return;
    //     }
        
    //     double[] linearInterpolation = LimelightConstants.ONE_APRIL_TAG_LINEAR_INTERPOLATOR.get(botPose.avgTagDist);
    //     addVisionMeasurement(
    //         botPose.pose,
    //         botPose.timestampSeconds,VecBuilder.fill(linearInterpolation[0],linearInterpolation[1],999999));
        
    // }
    public Rotation2d getYawOffsetDegrees(){
        return m_fieldRelativeOffset;
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