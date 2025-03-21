package frc.robot.subsystems.swerve;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.GameData;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.PathplannerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.subsystems.vision.Vision;

public class SwerveDrive extends SubsystemBase{
    private final Vision cameras;

    private final SwerveDriveIO swerveIO;


    //NetworkTables stuff
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable robot = inst.getTable("Robot");
    private final NetworkTable odom = robot.getSubTable("Odometry");

    private final StructPublisher<Pose2d> posePublisher = odom
        .getStructTopic("RobotPose", Pose2d.struct).publish();

    private final Notifier visionUpdates;

    public SwerveDrive(){
        if (Robot.isSimulation()){
            this.swerveIO = new SimSwerve();
        }else{
            this.swerveIO = TunerConstants.createDrivetrain();
        }

        cameras = new Vision(swerveIO, new Transform3d[]{LimelightConstants.FR_LIMELIGHT_CAMERA_TRANSFORM,
            LimelightConstants.FL_LIMELIGHT_CAMERA_TRANSFORM
            //  LimelightConstants.BACKWARD_LIMELIGHT_CAMERA_TRANSFORM
            },
              LimelightConstants.FR_LIMELIGHT_NAME,
              LimelightConstants.FL_LIMELIGHT_NAME
            //    LimelightConstants.BACKWARD_LIMELIGHT_NAME
               );

        visionUpdates = new Notifier(()->{
            double time = Timer.getFPGATimestamp();
            try {
                //TODO this will be in a seperate thread eventually, I just want to profile it on a robot first
                cameras.updateVisionPose();
            } catch (Exception e) {
    
            }
            if (Timer.getFPGATimestamp() - time > .03){
                System.out.println("Vision Overrun");
            }
        });
        Runtime.getRuntime().addShutdownHook(new Thread(visionUpdates::close));
        visionUpdates.startPeriodic(.02);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return this.run(() -> swerveIO.setControl(requestSupplier.get()));
    }

    public void setControl(SwerveRequest request) {
        swerveIO.setControl(request);
    }

    @Override
    public void periodic(){
        posePublisher.accept(getPose());
    }

    public void resetPose(Pose2d pose){
        swerveIO.resetPose(pose);
    }

    public void seedFieldRelative(Rotation2d rot){
        swerveIO.seedFieldRelative(rot);
    }

    public void configurePathPlanner() {
        AutoBuilder.configure(
            this::getPose, // getState of the robot pose
            this::resetPose,  // Consumer for seeding pose against auto
            swerveIO::getSpeeds,
            (speeds, feedForward)->swerveIO.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds.times(.5))), // Consumer of ChassisSpeeds to drive the robot
            new PPHolonomicDriveController(new PIDConstants(5, 0, 0),
                                            new PIDConstants(5, 0, 0)),
            PathplannerConstants.pathingConfig,
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

    public Pose2d getPose(){
        return swerveIO.getPose();
    }

    public Rotation2d getYaw(){
        return swerveIO.getYaw();
    }

    public SwerveDriveIO getDriveIO(){
        return swerveIO;
    }

    public void registerTelemetry(Consumer<SwerveDriveState> object) {
        swerveIO.registerTelemetry(object);
    }
}
