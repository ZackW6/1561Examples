 // final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = vision1.update();
        // if (optionalEstimatedPoseRight.isPresent()) {
        //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
        //     poseEstimator.updateVisionMeasurement(estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        // }
        
        // final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = vision2.update();
        // if (optionalEstimatedPoseLeft.isPresent()) {
        //     final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();          
        //     poseEstimator.updateVisionMeasurement(estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        // }
        
        // poseEstimator.update(/*ccw gyro rotation*/, /*module positions array*/);// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Set;

import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WaitAutos;
import frc.robot.commands.FactoryCommands;
import frc.robot.commands.WaitAutos.BranchInstruction;
import frc.robot.commands.WaitAutos.BranchInstruction.BeginPose;
import frc.robot.commands.WaitAutos.BranchInstruction.IntakePose;
import frc.robot.commands.WaitAutos.BranchInstruction.ShootPose;
import frc.robot.constants.GameData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.MainMechanism;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.realSwerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.subsystems.vision.objectDetection.ObjectDetection;
import frc.robot.util.ChoreoEX;
import frc.robot.util.CustomController;
import frc.robot.util.DynamicObstacle;
import frc.robot.util.MapleSimWorld;
import frc.robot.commands.WheelRadiusCommand;


public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CustomController customController = new CustomController(0);
  
  private final SwerveDrive drivetrain = new SwerveDrive(); // My drivetrain

  private final Elevator elevator = new Elevator();

  private final Intake intake = new Intake();

  private final Arm arm = new Arm();

  private final MainMechanism scoringMechanism = new MainMechanism(arm, intake, elevator);
  // private final ObjectDetection objectDetection = new ObjectDetection("Test",
  //   new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))), ()->drivetrain.getPose());

  private final FactoryCommands factoryCommands = new FactoryCommands(drivetrain, scoringMechanism);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.TRANSLATIONAL_DEADBAND).withRotationalDeadband(TunerConstants.ROTATIONAL_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.resetPose(new Pose2d(7,5,new Rotation2d()));
    customController.fixedButtonPressed(1).whileTrue(factoryCommands.autoScoreCoral(1,4));
    customController.fixedButtonPressed(2).whileTrue(factoryCommands.autoScoreCoral(2,4));
    customController.fixedButtonPressed(3).whileTrue(factoryCommands.autoScoreCoral(3,4));
    customController.fixedButtonPressed(4).whileTrue(factoryCommands.autoScoreCoral(4,4));
    customController.fixedButtonPressed(5).whileTrue(factoryCommands.autoScoreCoral(5,4));
    customController.fixedButtonPressed(6).whileTrue(factoryCommands.autoScoreCoral(6,4));
    customController.fixedButtonPressed(7).whileTrue(factoryCommands.autoScoreCoral(7,4));
    customController.fixedButtonPressed(8).whileTrue(factoryCommands.autoScoreCoral(8,4));
    customController.fixedButtonPressed(9).whileTrue(factoryCommands.autoScoreCoral(9,4));
    customController.fixedButtonPressed(10).whileTrue(factoryCommands.autoScoreCoral(10,4));
    customController.fixedButtonPressed(11).whileTrue(factoryCommands.autoScoreCoral(11,4));
    customController.fixedButtonPressed(12).whileTrue(factoryCommands.autoScoreCoral(12,4));

    driverController.a().whileTrue(elevator.reachGoal(0).alongWith(arm.reachGoal(0)));

    driverController.b().onTrue(new WheelRadiusCommand(drivetrain));
    driverController.povUp().whileTrue(scoringMechanism.grabAlgaeLower());
    driverController.povRight().whileTrue(scoringMechanism.grabAlgaeUpper());
    driverController.povDown().whileTrue(scoringMechanism.scoreAlgaeNet());
    driverController.povLeft().whileTrue(scoringMechanism.score(4));

    // driverController.x().whileTrue(drivetrain.passiveTowardPose(new Pose2d(3,4.5,Rotation2d.fromDegrees(90)), 5,3, 5, "TEST3"));
    driverController.x().whileTrue(factoryCommands.autoScoreCoral(12,3));
  
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * 1.00 * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * 1.00 * MaxSpeed)
            .withRotationalRate(-driverController.getRightX()/*driverController.getRawAxis(2)*/ * 2/3 * MaxAngularRate)
    ));

    /* Controller Bindings */

    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getPose().getRotation())));
    
    driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.leftTrigger(.5).whileTrue(factoryCommands.autoIntakeCoral(0));

    //TODO was deffered, i switched to not, make sure it works in all scenarios
    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
      // DynamicObstacle.setDynamicObstacles("testNodeSize", drivetrain.getPose().getTranslation());
      DynamicObstacle.setDynamicObstacles("avoidAlgae",drivetrain.getPose().getTranslation());
      if (Robot.isSimulation()){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(90));
        return;
      }
      if (DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(0));
      }
    }));
  }

  public RobotContainer() {
    // MapleSimWorld.instantiate();
    drivetrain.configurePathPlanner();
    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
    
    configureAutonomousCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

    //How you might make a choreo only path
    // autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
    
    autoChooser.addOption("WaitAuto",
      WaitAutos.createBranchCommand(new Pose2d(7.578,1.662,Rotation2d.fromDegrees(180)),
        BranchInstruction.of(BeginPose.BeginRight, ShootPose.PlaceF,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceA,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceB,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceE,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceG,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceH,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceI,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceJ,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceK,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceL,4)
        ));
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", Commands.none());
    NamedCommands.registerCommand("place", Commands.none());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}