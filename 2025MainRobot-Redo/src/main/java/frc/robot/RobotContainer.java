// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WaitAutos;
import frc.robot.commands.WaitAutos.BranchInstruction;
import frc.robot.commands.WaitAutos.BranchInstruction.BeginPose;
import frc.robot.commands.WaitAutos.BranchInstruction.IntakePose;
import frc.robot.commands.WaitAutos.BranchInstruction.ShootPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbMechanism;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.BaseMechanism.MainStates;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.ramp.Ramp;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Telemetry;
import frc.robot.util.ChoreoEX;
import frc.robot.util.CustomController;
import frc.robot.util.MutSlewRateLimiter;
import frc.robot.util.PoseEX;
import frc.robot.util.SendableConsumer;


public class RobotContainer {

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Command> algaeEnd;
  private SendableChooser<Command> teenyPush;


  private MutSlewRateLimiter[] limiter = new MutSlewRateLimiter[]{new MutSlewRateLimiter(.5),new MutSlewRateLimiter(.5),new MutSlewRateLimiter(.5)};
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = .6;
  private double rotationPercent = .6;


  private final CommandXboxController driverController = new CommandXboxController(0);

  private final CustomController customController = new CustomController(1);
  
  private final SwerveDrive drivetrain = new SwerveDrive();

  private final Elevator elevator = new Elevator();

  private final Intake intake = new Intake();

  private final Arm arm = new Arm();
  
  private final Climber climber = new Climber();

  private final Ramp ramp = new Ramp();

  private final CommandMechanism scoringMechanism = new CommandMechanism(arm, intake, elevator, ramp, drivetrain);

  private final ClimbMechanism climbMechanism = new ClimbMechanism(arm, climber, ramp);
  // private final ObjectDetection objectDetection = new ObjectDetection("Test",
  //   new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))), ()->drivetrain.getPose());
  
  private final OptionController optionController = new OptionController(customController, scoringMechanism, ()-> intake.hasCoral(), ()-> intake.hasAlgae(), ()->drivetrain.getPose());

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.resetPose(new Pose2d(7,5,Rotation2d.fromDegrees(180)));
  
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(limiter[0].calculate(-driverController.getLeftY() * MaxSpeed * MathUtil.clamp(1/Math.abs(elevator.getPosition()),.1,1)))
            .withVelocityY(limiter[1].calculate(-driverController.getLeftX() * speedPercent * MaxSpeed * MathUtil.clamp(1/(Math.abs(elevator.getPosition())),.1,1)))
            .withRotationalRate(limiter[2].calculate(-driverController.getRightX()/*driverController.getRawAxis(2)*/ * rotationPercent * MaxAngularRate * MathUtil.clamp(1/(Math.abs(elevator.getPosition())),.1,1)))
    ));

    SendableConsumer.createSendableChooser("AccelerationLimit", (data)->{
      limiter[0].setRateLimit(data);
      limiter[1].setRateLimit(data);}, 100);
    SendableConsumer.createSendableChooser("RotAccelLimit", (data)->{
      limiter[2].setRateLimit(data);}, 100);

    SendableConsumer.createSendableChooser("DeccelerationLimit", (data)->{
      limiter[0].setDecelLimit(data);
      limiter[1].setDecelLimit(data);}, 100);
    SendableConsumer.createSendableChooser("RotDeccelLimit", (data)->{
      limiter[2].setDecelLimit(data);}, 100);
    
    drivetrain.getDriveIO().registerTelemetry((log)->logger.telemeterize(log));
    

    createPresetControls();

    driverController.start().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getPose().getRotation())));

    //TODO was deffered, i switched to not, make sure it works in all scenarios
    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(0));
      }
    }));
  }

  public void createPresetControls(){
    driverController.povUp().onTrue(Commands.runOnce(()->optionController.setReefLevel(1)));
    driverController.povRight().onTrue(Commands.runOnce(()->optionController.setReefLevel(2)));
    driverController.povDown().onTrue(Commands.runOnce(()->optionController.setReefLevel(3)));
    driverController.povLeft().onTrue(Commands.runOnce(()->optionController.setReefLevel(4)));

    driverController.rightTrigger(.2).whileTrue(optionController.getScoreLevel());
    driverController.y().whileTrue(optionController.getAlgaeLevel());
    driverController.b().whileTrue(climbMechanism.prepare());
    driverController.x().whileTrue(climbMechanism.climb());
    driverController.leftTrigger(.2).whileTrue(optionController.getAlgaeIntakeLevel().alongWith(intake.reachGoal(-60)));
    // driverController.a().whileTrue(intake.setVelocity(30).alongWith(elevator.reachGoal(0).alongWith(arm.reachGoal(-.22)).alongWith(ramp.reachGoal(0))));
    driverController.rightBumper().whileTrue(intake.reachGoal(60));
    Supplier<Pose2d> prospectivePose = ()->{
      Pose2d pose = drivetrain.getObjectPose().isPresent() ? drivetrain.getObjectPose().get() : new Pose2d(-500,-500, new Rotation2d());
      pose = new Pose2d(pose.getX(), pose.getY(), PoseEX.correctedRotation(PoseEX.getPoseAngle(drivetrain.getPose(),pose).rotateBy(Rotation2d.k180deg)));
      return pose;
    };
    driverController.back().whileTrue(Commands.defer(()->drivetrain.toPose((prospectivePose.get())
      , .3,1,5),Set.of()));
    driverController.leftBumper().whileTrue(optionController.resetOrIntake());
    driverController.a().whileTrue(drivetrain.applyRequest(()->brake)).onTrue(Commands.runOnce(()->{limiter[0].reset(0);
      limiter[1].reset(0);
      limiter[2].reset(0);}));
    // driverController.start().whileTrue(optionController.getAutoAlgae());
    driverController.leftStick().whileTrue(optionController.getAutoCoral(1));
    driverController.rightStick().whileTrue(optionController.getAutoCoral(2));
    // driverController.back().whileTrue(optionController.getAutoCoralPosition());

    customController.fixedButtonPressed(1).onTrue(Commands.runOnce(()->arm.setDefaultCommand(arm.reachGoal(0))));
    customController.fixedButtonPressed(2).onTrue(Commands.runOnce(()->arm.setDefaultCommand(arm.reachGoal(MainStates.Intake.armRotation))));

    customController.fixedButtonPressed(19).onTrue(Commands.runOnce(()->{speedPercent = .2;
    rotationPercent = .2;}));
    customController.fixedButtonPressed(20).onTrue(Commands.runOnce(()->{speedPercent = .8;
    rotationPercent = .65;}));
  }

  public RobotContainer() {
    drivetrain.configurePathPlanner();
    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
    
    configureAutonomousCommands();

    //TODO this could cause auto errors, if so just comment

    WaitAutos waitAutos = new WaitAutos(scoringMechanism);

    autoChooser = buildAutoChooser("", (data) -> data);

    // autoChooser.onChange((data)->{
    //   try{
    //     if (AutoBuilder.getAllAutoNames().contains(data.getName())){
    //       PathPlannerAuto auto = new PathPlannerAuto(data.getName());
    //       drivetrain.resetPose(auto.getStartingPose());
    //     }else{
    //       Pose2d autoStart = waitAutos.getStartingPose(data.getName());
    //       if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
    //         autoStart = PoseEX.pose180(autoStart);
    //       }
    //       drivetrain.resetPose(autoStart);
    //     }
    //   } catch(Exception e){
        
    //   }
    // });

    

    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // SmartDashboard.putData("Push?", teenyPush);

    // SmartDashboard.putData("GrabAlgae?", algaeEnd);

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();

    //How you might make a choreo only path
    autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
    autoChooser.addOption("RightAuto",
      waitAutos.createBranchCommand("RightAuto", new Pose2d(7.2,2.5,Rotation2d.fromDegrees(180)), "", true,
        BranchInstruction.of(BeginPose.BeginRight, ShootPose.PlaceE,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceC,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceD,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceB,4),
        BranchInstruction.of(IntakePose.FeederTwo, ShootPose.PlaceA,4)
    ));
    autoChooser.addOption("MiddleAuto",
      waitAutos.createBranchCommand("MiddleAuto", new Pose2d(7.2,4.025,Rotation2d.fromDegrees(180)), "", true,
        BranchInstruction.of(BeginPose.BeginMiddle, ShootPose.PlaceH,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceA,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceB,4)
    ));
    autoChooser.addOption("LeftAuto",
      waitAutos.createBranchCommand("LeftAuto", new Pose2d(7.2,5.55,Rotation2d.fromDegrees(180)), "", true,
        BranchInstruction.of(BeginPose.BeginLeft, ShootPose.PlaceJ,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceK,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceL,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceA,4),
        BranchInstruction.of(IntakePose.FeederOne, ShootPose.PlaceB,4)
    ));
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", scoringMechanism.intake());
    NamedCommands.registerCommand("presetL4", scoringMechanism.setState(MainStates.L4));
    NamedCommands.registerCommand("shoot", intake.reachGoal(60));
  }

  public Command getAutonomousCommand() {
    // CommandScheduler.getInstance().removeComposedCommand(autoChooser.getSelected());
    // CommandScheduler.getInstance().removeComposedCommand(teenyPush.getSelected());
    // CommandScheduler.getInstance().removeComposedCommand(algaeEnd.getSelected());
    // return autoChooser.getSelected().beforeStarting(teenyPush.getSelected()).andThen(algaeEnd.getSelected()).andThen(drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-1)).withTimeout(.4));
    // return teenyPush.getSelected().andThen(algaeEnd.getSelected());//.andThen(algaeEnd.getSelected()).andThen(drivetrain.applyRequest(()->new SwerveRequest.RobotCentric().withVelocityX(-1)).withTimeout(.4));
    return autoChooser.getSelected();
    // return Commands.none();
  }

  public static SendableChooser<Command> buildAutoChooser(
      String defaultAutoName,
      Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<Command> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    PathPlannerAuto defaultOption = null;
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto;
      try {
        auto = new PathPlannerAuto(autoName);
      } catch (Exception e) {
        auto = new PathPlannerAuto(Commands.none());
      }
      
      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", Commands.none());
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.addOption("None", Commands.none());
    }

    optionsModifier
        .apply(options.stream())
        .forEach(auto -> chooser.addOption(auto.getName(), auto));

    return chooser;
  }
}