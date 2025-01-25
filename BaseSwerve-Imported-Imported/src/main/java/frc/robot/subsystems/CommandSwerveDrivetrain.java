package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.commands.FactoryCommands;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.PathplannerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerveHelpers.MainDrive;
import frc.robot.subsystems.swerveHelpers.Vector2;
import frc.robot.util.DynamicObstacle;
import frc.robot.util.PoseEX;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final MainDrive mainRequest;

    private final Vision cameras;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        cameras = new Vision(this, LimelightConstants.LIMELIGHT_NAME);
        mainRequest = new MainDrive(this);
        setDefaultCommand(applyRequest(()->mainRequest).alongWith(Commands.print("HI")));
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        cameras = new Vision(this, LimelightConstants.LIMELIGHT_NAME);
        mainRequest = new MainDrive(this);
        setDefaultCommand(applyRequest(()->mainRequest));
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
            cameras.updateVisionPose();
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void configureTeleop(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vrot){
        setDefaultCommand(applyRequest(()->mainRequest).alongWith(
            // Commands.run(()->mainRequest.addFieldFacingSpeeds(5, 0, 10, "TELEOP"))));
                // Commands.run(()->comparison.withSpeeds(new ChassisSpeeds(5,0,10)))));
            Commands.run(()->mainRequest.addFieldFacingSpeeds(vx.getAsDouble(), vy.getAsDouble(), vrot.getAsDouble(), "TELEOP"))));
            // Commands.run(()->comparison.withVelocityX(vx.getAsDouble()).withVelocityY(vy.getAsDouble()).withRotationalRate(vrot.getAsDouble()))));

        new Trigger(()->DriverStation.isAutonomous()).onTrue(Commands.runOnce(()->{
            addFieldFacingSpeeds(0,0,0, "TELEOP");
        }));
    }

    public Command addFieldRelativeSpeeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr, String key){
        return Commands.run(()->mainRequest.addFieldSpeeds(vx.getAsDouble(), vy.getAsDouble(), vr.getAsDouble(), key))
        .finallyDo(()->{
            mainRequest.removeSource(key);
        });
    }

    public Command addFieldFacingSpeeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr, String key){
        return Commands.run(()->mainRequest.addFieldFacingSpeeds(vx.getAsDouble(), vy.getAsDouble(), vr.getAsDouble(), key))
        .finallyDo(()->{
            mainRequest.removeSource(key);
        });
    }

    public Command addRobotRelativeSpeeds(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier vr, String key){
        return Commands.run(()->mainRequest.addRobotSpeeds(vx.getAsDouble(), vy.getAsDouble(), vr.getAsDouble(), key))
        .finallyDo(()->{
            mainRequest.removeSource(key);
        });
    }

    public void addFieldRelativeSpeeds(double vx, double vy, double vr, String key){
        mainRequest.addFieldSpeeds(vx, vy, vr, key);
    }

    public void addFieldFacingSpeeds(double vx, double vy, double vr, String key){
        mainRequest.addFieldFacingSpeeds(vx, vy, vr, key);
    }

    public void addRobotRelativeSpeeds(double vx, double vy, double vr, String key){
        mainRequest.addRobotSpeeds(vx, vy, vr, key);
    }

    public void removeSource(String key){
        mainRequest.removeSource(key);
    }

    public void configurePathPlanner() {
        //EXAMPLE CODE
        Optional<FactoryCommands> commands = FactoryCommands.getInstance();
        Command pointTowardPiece;

        if (commands.isPresent()){
            pointTowardPiece = commands.get().activePointPose(new Pose2d(3,4.5, new Rotation2d()),5,.5,"AUTOPOINT");
        }else{
            pointTowardPiece = Commands.none();
        }

        new Trigger(()->DriverStation.isAutonomous()).onTrue(pointTowardPiece);
        
        new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
            addRobotRelativeSpeeds(0,0,0, "AUTO");
            pointTowardPiece.end(false);
        }));

        
        AutoBuilder.configure(
            this::getPose, // getState of the robot pose
            this::resetPose,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds, feedForward)->this.setControl(mainRequest.addRobotSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, "AUTO")), // Consumer of ChassisSpeeds to drive the robot
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

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getState().Speeds;
    }

    public Rotation2d getYaw() {
        return getState().RawHeading;
    }
    
    public Rotation2d getYawOffset(){
        return Rotation2d.fromRadians(SwerveJNI.JNI_GetOperatorForwardDirection(m_drivetrainId));
    }
}
