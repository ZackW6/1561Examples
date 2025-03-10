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

import java.util.Set;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FactoryCommands.State;
import frc.robot.commands.PathOnTheFly.AutoToPoint;
import frc.robot.commands.PathOnTheFly.PathConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ConditionalAutos;
import frc.robot.commands.FactoryCommands;
import frc.robot.commands.OnTheFlyAutos;
import frc.robot.commands.PathOnTheFly;
import frc.robot.commands.WheelRadiusCommand;
import frc.robot.constants.LimelightConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.util.ChoreoEX;
import frc.robot.util.DynamicObstacle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.TRANSLATIONAL_DEADBAND).withRotationalDeadband(TunerConstants.ROTATIONAL_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static Rotation2d autoAngleOffset;
  /* Subsystems */
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  // private final Elevator elevator = new Elevator();
  private final ObjectDetection limelightObject = new ObjectDetection(LimelightConstants.AMP_CAM, LimelightConstants.AMP_CAM_TRANSFORM, ()->drivetrain.getPose());
  // private final Candle candle = new Candle();
  private final FactoryCommands groupCommands = new FactoryCommands(arm, shooter, intake, drivetrain, limelightObject, driverController);

  private final ConditionalAutos conditionalAutos = new ConditionalAutos(arm, shooter, intake, drivetrain, limelightObject, driverController, groupCommands);
  private final OnTheFlyAutos onTheFlyAutos = new OnTheFlyAutos(arm, shooter, intake, drivetrain, limelightObject, driverController, groupCommands);
  private void configureBindings() {
    // Command pid = new PIDTuningCommand(()->arm.getArmDegrees(), arm::setArmP,0,100000, arm);
    // driverController.a().onTrue(pid);
    /* Setup Default Commands */
    Command initState = groupCommands.switchState(State.Speaker);
    initState.initialize();
    initState.schedule();

    shooter.setIdleSpeed(0, 0);

    intake.setDefaultCommand(intake.stop());

    // elevator.setDefaultCommand(elevator.reachGoal(()->.5-driverController.getRawAxis(1)));
    // candle.setDefaultCommand(candle.idleLED());
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-driverController.getRightX()/*driverController.getRawAxis(2)*/ * 2/3 * MaxAngularRate)
    ));
    /* Controller Bindings */

    driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    // driverController.y().onTrue(groupCommands.findWheelRadius());
    driverController.rightBumper().whileTrue(groupCommands.intake());//.whileFalse(Commands.runOnce(()->arm.setArmRotation(arm.lastMainState())));
    driverController.leftBumper().onTrue(groupCommands.shoot());
    driverController.rightTrigger(.5).whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.leftTrigger(.5).whileTrue(groupCommands.align())
    .onTrue(Commands.runOnce(()->shooter.setIdleSpeed(70,70)))
    .onFalse(Commands.runOnce(()->shooter.setIdleSpeed(0,0)));
    driverController.rightStick().whileTrue(groupCommands.alignToCorner());
    driverController.a().whileTrue(groupCommands.alignToAmp());//.and(() -> driverController.x().getAsBoolean());
    driverController.b().whileTrue(groupCommands.getToPieceCommand());//Commands.either(Commands.none(), groupCommands.alignToPiece(),()-> intake.isPiecePresent()));
    driverController.x().whileTrue(AutoToPoint.getToPoint(new Pose2d(9,1.2,Rotation2d.fromDegrees(145)), new PathConfig(4,4,Rotation2d.fromDegrees(120),Rotation2d.fromDegrees(120),0,0)).andThen(groupCommands.shoot()));
    // driverController.x().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(60))).onFalse(Commands.runOnce(()->shooter.setIdleSpeed(0)));

    // operatorController.y().whileTrue(AutoToPoint.getToPoint(8.47,1.03,-25.16, new PathConfig(2,2,Rotation2d.fromDegrees(120),Rotation2d.fromDegrees(120),0,0),true));
    // operatorController.leftTrigger().whileTrue(AutoToPath.getToPath("To Amp", new PathConfig(2,2,Rotation2d.fromDegrees(120),Rotation2d.fromDegrees(120),0,0)));
    // driverController.getHID().setRumble(RumbleType.kBothRumble, 1);

    Command either = Commands.either(Commands.either(groupCommands.switchState(State.Amp),groupCommands.switchState(State.Speaker), ()->groupCommands.getState() == State.Speaker),Commands.none(),()->groupCommands.getState()!=State.Intake);
    operatorController.a().onTrue(either);
    operatorController.rightBumper().onTrue(new WheelRadiusCommand(drivetrain));
    operatorController.b().onTrue(Commands.runOnce(()->{
      drivetrain.resetPose(new Pose2d(3,5,new Rotation2d(0)));
    }));

    operatorController.leftTrigger().whileTrue(intake.setVelocity(-5));
    operatorController.rightTrigger().whileTrue(intake.setVelocity(10));
    // operatorController.x().whileTrue(Commands.deadline(Commands.waitSeconds(.1),intake.setVelocity(-5)).andThen(intake.setVelocity(10)));
    // operatorController.y().whileTrue(getAutoToPoint().andThen(groupCommands.loadAndShoot()));Command
    operatorController.leftBumper().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(70,70)));
    operatorController.rightBumper().onTrue(Commands.runOnce(()->shooter.setIdleSpeed(0,0)));
    
    if (Utils.isSimulation()) {
      drivetrain.resetPose(new Pose2d(new Translation2d(3,1), Rotation2d.fromDegrees(90)));
      drivetrain.seedFieldRelative(Rotation2d.fromDegrees(90));
      //TODO
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    limelightObject.registerTelemetry(logger::registerPieceTelemetry);
    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.defer(()->Commands.runOnce(()->{
      // DynamicObstacle.setDynamicObstacles("testNodeSize", drivetrain.getPose().getTranslation());
      DynamicObstacle.clearDynamicObstacles(drivetrain.getPose().getTranslation());
      if (Robot.isSimulation()){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(90));
        return;
      }
      if (DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(360));
      }}), Set.of()));
    //TODO

    // }),Set.of()));
  }

  public RobotContainer() {
    drivetrain.configurePathPlanner();
    ChoreoEX.setDrivetrain(drivetrain);

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);

    PathOnTheFly.PathConfig pathConfig = new PathOnTheFly.PathConfig(5,5,Rotation2d.fromDegrees(720),Rotation2d.fromDegrees(720),0,0);
    PathOnTheFly.addConfig(pathConfig,0);

    configureAutonomousCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
    autoChooser.addOption("Conditional Auto", conditionalAutos.getConditionalAuto());
    autoChooser.addOption("OnTheFly Auto", onTheFlyAutos.getOnTheFlyAuto(new Pose2d(1.4818934202194214,7.29,Rotation2d.fromDegrees(0)),"Note3Correct",1,2,4,7,8,3,0));
    autoChooser.addOption("Pre876", onTheFlyAutos.getOnTheFlyAuto(new Pose2d(1.4818934202194214,3.43,Rotation2d.fromDegrees(0)),"AvoidStage",8,7,6,0));
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", groupCommands.intakeMainAuto());
    
    NamedCommands.registerCommand("setIdleSpeed", Commands.runOnce(()->shooter.setIdleSpeed(46,46)));
    NamedCommands.registerCommand("loadAndShoot", groupCommands.speakerShoot(60,80));
    NamedCommands.registerCommand("loadAndShootLinear", groupCommands.speakerShoot(60,80));
    NamedCommands.registerCommand("loadAndShootThree", groupCommands.speakerShoot(50,35));
    NamedCommands.registerCommand("loadAndShootFour", groupCommands.speakerShoot(55,37));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}