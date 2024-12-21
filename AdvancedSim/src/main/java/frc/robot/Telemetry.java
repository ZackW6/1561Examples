package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.swerve.SwerveBase.DriveState;

public class Telemetry {
    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry() {
        SignalLogger.start();
    }
    
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable table = inst.getTable("Pose");

    private final DoubleArrayPublisher robotPose = table.getDoubleArrayTopic("robotPose").publish();
    private final ArrayList<DoubleArrayPublisher> visionTargets = new ArrayList<>();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();
      
    private final NetworkTable swerveTable = inst.getTable("Swerve");

    StructArrayPublisher<SwerveModuleState> actualStates = swerveTable
        .getStructArrayTopic("actualStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> goalStates = swerveTable
        .getStructArrayTopic("goalStates", SwerveModuleState.struct).publish();

    /* Robot speeds for general checking */
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();

    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();


    public void telemeterize(DriveState driveState) {

        Pose2d pose = driveState.drivePose;

        fieldTypePub.set("Field2d");
        robotPose.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        });


        SwerveModuleState[] actualState = new SwerveModuleState[driveState.modules.size()];
        SwerveModuleState[] goalState = new SwerveModuleState[driveState.modules.size()];

        for (int i = 0; i < driveState.modules.size(); i++){
            actualState[i] = driveState.modules.get(i).actualState;
            goalState[i] = driveState.modules.get(i).goalState;
        }

        actualStates.accept(actualState);
        goalStates.accept(goalState);

        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;

        speed.set(Math.sqrt(Math.pow((pose.getX()-m_lastPose.getX())/diffTime,2)+Math.pow((pose.getY()-m_lastPose.getY())/diffTime,2)));
        velocityX.set((pose.getX()-m_lastPose.getX())/diffTime);
        velocityY.set((pose.getY()-m_lastPose.getY())/diffTime);

        m_lastPose = pose;
    }
}
