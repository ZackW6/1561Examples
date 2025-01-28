package frc.robot.subsystems.swerve.simSwerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDriveIO;
import frc.robot.subsystems.swerve.swerveHelpers.MainDrive;
import frc.robot.util.MapleSimWorld;

public class SimSwerve implements SwerveDriveIO {

    private final SelfControlledSwerveDriveSimulation simulatedDrive;

    private final NetworkTable instance = NetworkTableInstance.getDefault().getTable("RealData");

    private final StructPublisher<Pose2d> posePublisher = instance
        .getStructTopic("RealPose", Pose2d.struct).publish();



    private Rotation2d yawOffset = new Rotation2d();
    
    private Thread updateThread = new Thread();


    public SimSwerve() {
        // For your own code, please configure your drivetrain properly according to the documentation
        DriveTrainSimulationConfig config = SimSwerveConstants.driveTrainSimulationConfig;

        // Creating the SelfControlledSwerveDriveSimulation instance
        //I SPENT HOURS TO FIND THIS LINE!!!!!!!!!!!!!!!!!!!!!!!!! AHHHHHHHHHHHHHHHHHHHHHHHHHH.
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(7, 5, new Rotation2d())), VecBuilder.fill(.1,.1,0.025), VecBuilder.fill(0,0,0));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        updateThread = new Thread(()->{
            while(true){
                try {
                    // SimulatedArena.getInstance().simulationPeriodic();
                    try {
                        simulatedDrive.periodic();
                    } catch (Exception e) {
                        System.out.println("AHHHHH");
                    }
                    posePublisher.accept(simulatedDrive.getActualPoseInSimulationWorld());
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true);
        updateThread.start();
        MapleSimWorld.addMainDrive(simulatedDrive.getDriveTrainSimulation());
    }

    @Override
    public void setControl(SwerveRequest request) {
        //TODO this is probably really slow, if someone finds a better way please fix
        ChassisSpeeds speeds = new ChassisSpeeds();
        request.apply(new SwerveControlParameters(), new SwerveModule[]{});
        boolean fieldRelative = false;
        boolean discretize = true;
        if (request instanceof MainDrive){
            speeds = ((MainDrive)request).Speeds;
        }
        if (request instanceof SwerveRequest.ApplyFieldSpeeds){
            speeds = ((SwerveRequest.ApplyFieldSpeeds)request).Speeds;
            fieldRelative = true;
        }
        if (request instanceof SwerveRequest.ApplyRobotSpeeds){
            speeds = ((SwerveRequest.ApplyRobotSpeeds)request).Speeds;
        }
        if (request instanceof SwerveRequest.FieldCentric){
            speeds = new ChassisSpeeds(
                ((SwerveRequest.FieldCentric)request).VelocityX,
                ((SwerveRequest.FieldCentric)request).VelocityY,
                ((SwerveRequest.FieldCentric)request).RotationalRate);
            fieldRelative = true;
        }
        if (request instanceof SwerveRequest.FieldCentricFacingAngle){
            //TODO
            fieldRelative = true;
        }
        if (request instanceof SwerveRequest.RobotCentric){
            speeds = new ChassisSpeeds(
                ((SwerveRequest.RobotCentric)request).VelocityX,
                ((SwerveRequest.RobotCentric)request).VelocityY,
                ((SwerveRequest.RobotCentric)request).RotationalRate);
        }
        if (request instanceof SwerveRequest.PointWheelsAt){
            Rotation2d direction = ((SwerveRequest.PointWheelsAt)request).ModuleDirection;
            runSwerveStates(
                new SwerveModuleState[]{
                    new SwerveModuleState(0,direction),
                    new SwerveModuleState(0,direction),
                    new SwerveModuleState(0,direction),
                    new SwerveModuleState(0,direction)
            });
            return;
        }
        if (request instanceof SwerveRequest.Idle){
            speeds = new ChassisSpeeds();
        }
        if (request instanceof SwerveRequest.SwerveDriveBrake){
            runSwerveStates(
                new SwerveModuleState[]{
                    new SwerveModuleState(0,Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0,Rotation2d.fromDegrees(45))
            });
            return;
        }
        runChassisSpeeds(speeds, fieldRelative, discretize);
    }

    private void runChassisSpeeds(ChassisSpeeds speeds, boolean fieldCentric, boolean discretize){
        this.simulatedDrive.runChassisSpeeds(
                speeds,
                new Translation2d(),
                fieldCentric,
                discretize);
    }

    public void runSwerveStates(SwerveModuleState[] targets) {
        this.simulatedDrive.runSwerveStates(targets);
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    public Pose2d getRealPose(){
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    double resetTime = -1;
    @Override
    public void resetPose(Pose2d pose) {
        resetTime = Timer.getFPGATimestamp();
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void seedFieldRelative(Rotation2d rot) {
        yawOffset = rot;
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestep, Vector<N3> stdDev) {
        if (currentTimeToFPGA(timestep) < resetTime){
            return;
        }
        simulatedDrive.addVisionEstimation(pose, currentTimeToFPGA(timestep), stdDev);
    }

    private double currentTimeToFPGA(double currentTime) {
        return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + currentTime;
    }

    private Thread telemetryThread = new Thread();
    private Consumer<SwerveDriveState> consumer = null;
    @Override
    public void registerTelemetry(Consumer<SwerveDriveState> consumer) {
        if (this.consumer != consumer){
            // telemetryThread.interrupt();
            telemetryThread = new Thread(()->{
                while(true){
                    try {
                        consumer.accept(getState());
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                

            });
            telemetryThread.setDaemon(true);
            telemetryThread.start();
        }
    }

    @Override
    public Rotation2d getYaw() {
        return simulatedDrive.getActualPoseInSimulationWorld().getRotation();
    }

    @Override
    public Rotation2d getYawOffset() {
        return yawOffset;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return simulatedDrive.getActualSpeedsRobotRelative();
    }

    @Override
    public SwerveDriveState getState() {
        SwerveDriveState state = new SwerveDriveState();
        state.Pose = getPose();
        state.ModuleStates = simulatedDrive.getMeasuredStates();
        state.ModuleTargets = simulatedDrive.getSetPointsOptimized();
        state.ModulePositions = simulatedDrive.getLatestModulePositions();
        
        return state;
    }
}
